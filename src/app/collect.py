#!/usr/bin/env python3
import argparse
import sys
import time
import subprocess
from pathlib import Path
from typing import Optional, Tuple

# Prevent __pycache__ creation (avoids root-owned pyc on bind mounts)
sys.dont_write_bytecode = True

from app.wifi_linux import detect_wifi_interface, read_wifi_status
from app.probes import ping
from app.logger_sqlite import SqliteLogger


def now_unix_ms() -> int:
    return int(time.time() * 1000)


def _parse_snmp_value(snmpget_stdout: str) -> Optional[str]:
    """
    Parse net-snmp output like:
      SNMPv2-SMI::... = STRING: "-49"
      SNMPv2-SMI::... = INTEGER: 44
    Return the value as a string (without surrounding quotes), or None.
    """
    line = snmpget_stdout.strip().splitlines()
    if not line:
        return None
    s = line[-1]
    if "=" not in s:
        return None
    rhs = s.split("=", 1)[1].strip()
    if ":" not in rhs:
        return None
    val = rhs.split(":", 1)[1].strip()
    if val.startswith('"') and val.endswith('"') and len(val) >= 2:
        val = val[1:-1]
    if val in ("", "N/A"):
        return None
    return val


def snmpget_value(host: str, community: str, oid: str, timeout_s: int = 1) -> Optional[str]:
    """
    Calls external `snmpget`. Requires net-snmp tools (snmpget) available in runtime container/host.
    """
    cmd = ["snmpget", "-v2c", "-c", community, host, oid]
    try:
        r = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except FileNotFoundError:
        raise RuntimeError("snmpget not found. Install net-snmp tools or run in a container that has snmpget.")
    except subprocess.TimeoutExpired:
        return None

    if r.returncode != 0:
        return None

    return _parse_snmp_value(r.stdout)


def snmp_read_wifi(
    host: str,
    community: str,
    rssi_oid: str,
    ssid_oid: str = "",
    bssid_oid: str = "",
    timeout_s: int = 1,
) -> Tuple[Optional[int], Optional[str], Optional[str]]:
    """
    Returns: (rssi_dbm, ssid, bssid)
    """
    rssi_s = snmpget_value(host, community, rssi_oid, timeout_s=timeout_s)
    ssid_s = snmpget_value(host, community, ssid_oid, timeout_s=timeout_s) if ssid_oid else None
    bssid_s = snmpget_value(host, community, bssid_oid, timeout_s=timeout_s) if bssid_oid else None

    rssi_dbm: Optional[int] = None
    if rssi_s is not None:
        try:
            rssi_dbm = int(float(rssi_s))
        except Exception:
            rssi_dbm = None

    return rssi_dbm, ssid_s, bssid_s


def main():
    p = argparse.ArgumentParser(description="Lexx500 Wi-Fi heatmapping collector (pose + Wi-Fi RSSI)")

    # Output
    p.add_argument("--db", default="data/run.sqlite")

    # Wi-Fi source (Linux interface). On AMR NUC (no Wi-Fi NIC), set --interface lo and use SNMP args.
    p.add_argument("--interface", default="auto")

    # Sampling
    p.add_argument("--hz", type=float, default=2.0)
    p.add_argument("--commit_every", type=int, default=50)

    # Pose selection
    p.add_argument("--pose_source", default="manual", choices=["manual", "ros1"])
    p.add_argument("--pose_topic", default="/amcl_pose")
    p.add_argument("--manual_pose", action="store_true", help="Enable manual pose entry (ManualPoseProvider).")

    # Optional ping probe
    p.add_argument("--probe_host", default="", help="Optional ping target (e.g. gateway IP).")
    p.add_argument("--probe_count", type=int, default=3)
    p.add_argument("--probe_timeout_s", type=int, default=1)

    # --- SNMP (MOXA) Wi-Fi telemetry ---
    # If --snmp_host is set, collector will use SNMP RSSI/SSID/BSSID instead of Linux iw.
    p.add_argument("--snmp_host", default="", help="SNMP host (e.g. MOXA IP like 192.168.1.254)")
    p.add_argument("--snmp_community", default="public", help="SNMP v2c community string (default: public)")
    p.add_argument("--snmp_timeout_s", type=int, default=1, help="SNMP request timeout seconds")

    # OIDs you discovered on AWK-1137C:
    # RSSI:  .1.3.6.1.4.1.8691.15.35.1.11.17.1.4.1.1  (STRING: "-49")
    # BSSID: .1.3.6.1.4.1.8691.15.35.1.11.17.1.3.1.1  (STRING: "50:91:E3:DC:7F:3E")
    # SSID:  .1.3.6.1.4.1.8691.15.35.1.11.17.1.6.1.1  (STRING: "lexxpluss_klab_demo")
    p.add_argument("--snmp_rssi_oid", default="", help="OID for RSSI (dBm) value (string/int)")
    p.add_argument("--snmp_ssid_oid", default="", help="OID for SSID value")
    p.add_argument("--snmp_bssid_oid", default="", help="OID for BSSID value")

    args = p.parse_args()

    # Resolve DB path and ensure directory exists
    db_path = Path(args.db)
    db_path.parent.mkdir(parents=True, exist_ok=True)

    # Interface resolution (only needed for Linux iw mode)
    iface = (args.interface or "").strip()

    # SNMP mode if snmp_host provided
    use_snmp = bool(args.snmp_host.strip())

    if not use_snmp:
        if iface in ("", "auto"):
            iface = detect_wifi_interface()
            if not iface:
                raise RuntimeError(
                    "Could not auto-detect Wi-Fi interface. "
                    "Use --interface wlp9s0 (etc). "
                    "On AMR NUC (no Wi-Fi NIC), use --interface lo for pose-only runs, "
                    "or set --snmp_host ... to read RSSI from MOXA via SNMP."
                )
    else:
        # In SNMP mode, interface can be lo (or anything) — we don't call iw.
        if iface in ("", "auto"):
            iface = "lo"

        # Sanity: require RSSI OID if using SNMP
        if not args.snmp_rssi_oid.strip():
            raise RuntimeError("SNMP mode enabled (--snmp_host set) but --snmp_rssi_oid is empty.")

    print(f"Using interface: {iface}")
    print(f"Logging to: {db_path}")
    print(f"Sample rate: {args.hz} Hz")
    print(f"Pose source: {args.pose_source}")
    if args.pose_source == "ros1":
        print(f"ROS1 pose topic: {args.pose_topic}")
    if use_snmp:
        print(f"Wi-Fi source: SNMP host={args.snmp_host} community={args.snmp_community}")
        print(f"  RSSI OID:  {args.snmp_rssi_oid}")
        if args.snmp_ssid_oid:
            print(f"  SSID OID:  {args.snmp_ssid_oid}")
        if args.snmp_bssid_oid:
            print(f"  BSSID OID: {args.snmp_bssid_oid}")
    else:
        print("Wi-Fi source: Linux iw")

    # -------------------------
    # Pose provider selection
    # -------------------------
    pose_provider = None

    if args.pose_source == "manual":
        # Manual pose (terminal prompt)
        if not args.manual_pose:
            print("NOTE: manual pose source selected but --manual_pose not set. Enabling manual pose anyway.")
        from app.pose_sources import ManualPoseProvider

        pose_provider = ManualPoseProvider()
        pose_provider.start()

    elif args.pose_source == "ros1":
        # Lazy import so non-ROS users don't need rospy installed
        from app.pose_ros1 import Ros1AmclPoseProvider

        pose_provider = Ros1AmclPoseProvider(topic=args.pose_topic)
        pose_provider.start()

        print("Waiting for first pose message...")
        t0 = time.time()
        while not pose_provider.has_pose():
            time.sleep(0.05)
            if time.time() - t0 > 10.0:
                print("Still waiting for pose... (check ROS vars + roscore + topic name)")
                t0 = time.time()

    else:
        raise RuntimeError(f"Unknown pose_source: {args.pose_source}")

    # -------------------------
    # Logging setup
    # -------------------------
    logger = SqliteLogger(str(db_path))
    commit_every = max(1, int(args.commit_every))
    period_s = 1.0 / max(0.1, float(args.hz))

    # Wi-Fi state tracking for roam/disconnect markers (best-effort)
    last_bssid: Optional[str] = None
    last_connected: bool = False
    disconnect_streak_s: float = 0.0

    samples = 0
    last_commit = 0

    try:
        while True:
            if pose_provider.should_stop():
                break

            pose = pose_provider.get_pose()

            # --- Wi-Fi read ---
            connected: bool = False
            ssid: Optional[str] = None
            bssid: Optional[str] = None
            rssi_dbm: Optional[int] = None
            freq_mhz: Optional[int] = None

            if use_snmp:
                rssi_dbm, ssid, bssid = snmp_read_wifi(
                    host=args.snmp_host.strip(),
                    community=args.snmp_community.strip(),
                    rssi_oid=args.snmp_rssi_oid.strip(),
                    ssid_oid=args.snmp_ssid_oid.strip(),
                    bssid_oid=args.snmp_bssid_oid.strip(),
                    timeout_s=int(args.snmp_timeout_s),
                )
                connected = rssi_dbm is not None
            else:
                # If user sets interface=lo (pose-only), skip iw and log wifi down.
                if iface == "lo":
                    connected = False
                else:
                    ws = read_wifi_status(iface)
                    connected = bool(ws.connected)
                    ssid = ws.ssid
                    bssid = ws.bssid
                    rssi_dbm = ws.rssi_dbm
                    freq_mhz = ws.freq_mhz

            # --- Derived events ---
            roam = 0
            disconnect = 0

            if connected:
                disconnect_streak_s = 0.0
                if last_connected and last_bssid and bssid and bssid != last_bssid:
                    roam = 1
            else:
                disconnect_streak_s += period_s
                if last_connected:
                    disconnect = 1

            last_connected = connected
            if bssid:
                last_bssid = bssid

            # --- Optional probe ---
            probe_ok: Optional[int] = None
            probe_rtt_ms: Optional[float] = None
            if args.probe_host:
                pr = ping(args.probe_host, count=args.probe_count, timeout_s=args.probe_timeout_s)
                probe_ok = 1 if pr.ok else 0
                probe_rtt_ms = pr.rtt_ms

            # --- Write sample ---
            logger.log_sample(
                t_ms=now_unix_ms(),
                x_m=float(pose.x_m),
                y_m=float(pose.y_m),
                theta_rad=float(pose.theta_rad),
                rssi_dbm=int(rssi_dbm) if rssi_dbm is not None else None,
                connected=1 if connected else 0,
                ssid=ssid,
                bssid=bssid,
                freq_mhz=int(freq_mhz) if freq_mhz is not None else None,
                roam=roam,
                disconnect=disconnect,
                disconnect_streak_s=float(disconnect_streak_s),
                probe_ok=probe_ok,
                probe_rtt_ms=probe_rtt_ms,
            )

            samples += 1

            # Commit periodically
            if samples % commit_every == 0:
                logger.commit()
                last_commit = samples
                wifi_state = "OK" if connected else "DOWN"
                print(
                    f"Collected {samples} samples... "
                    f"pose=({pose.x_m:.2f},{pose.y_m:.2f}) wifi={wifi_state} rssi={rssi_dbm}"
                )

            time.sleep(period_s)

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")

    finally:
        logger.commit()
        last_commit = samples
        print(f"✅ Done. Samples committed (last_commit={last_commit}). DB: {db_path}")


if __name__ == "__main__":
    main()
