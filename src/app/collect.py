#!/usr/bin/env python3
import argparse
import shutil
import subprocess
import time
from pathlib import Path
from typing import Optional, Tuple, Any

from app.wifi_linux import detect_wifi_interface, read_wifi_status
from app.probes import ping
from app.logger_sqlite import SqliteLogger


def now_unix_ms() -> int:
    return int(time.time() * 1000)


def _pose_to_xytheta(pose: Any) -> Tuple[float, float, float]:
    """
    Normalizes different pose shapes into (x, y, theta_rad).

    Supported:
      - pose.x_m / pose.y_m / pose.theta_rad
      - pose.x / pose.y / pose.theta
      - pose.yaw_rad
      - pose as tuple/list: (x, y, theta)
    """
    # tuple/list support
    if isinstance(pose, (tuple, list)) and len(pose) >= 3:
        return float(pose[0]), float(pose[1]), float(pose[2])

    # common attribute name variants
    x = getattr(pose, "x_m", None)
    if x is None:
        x = getattr(pose, "x", None)

    y = getattr(pose, "y_m", None)
    if y is None:
        y = getattr(pose, "y", None)

    theta = getattr(pose, "theta_rad", None)
    if theta is None:
        theta = getattr(pose, "theta", None)
    if theta is None:
        theta = getattr(pose, "yaw_rad", None)
    if theta is None:
        theta = getattr(pose, "yaw", None)

    if x is None or y is None or theta is None:
        raise AttributeError(
            f"Pose object is missing expected fields. "
            f"Need (x_m,y_m,theta_rad) or (x,y,theta). Got: {pose!r}"
        )

    return float(x), float(y), float(theta)


def _snmpget_raw(host: str, community: str, oid: str, timeout_s: float = 1.0) -> str:
    """
    Returns the RHS string from `snmpget` output.

    Uses -Oqv when available (value-only).
    """
    if shutil.which("snmpget") is None:
        raise RuntimeError(
            "snmpget not found in PATH. "
            "Install SNMP tools in the runtime environment (e.g., apt-get install -y snmp), "
            "or run inside a container/image that includes snmpget."
        )

    try:
        r = subprocess.run(
            ["snmpget", "-v2c", "-c", community, "-t", str(timeout_s), "-r", "0", "-Oqv", host, oid],
            capture_output=True,
            text=True,
            check=True,
        )
        return r.stdout.strip()
    except subprocess.CalledProcessError:
        r = subprocess.run(
            ["snmpget", "-v2c", "-c", community, "-t", str(timeout_s), "-r", "0", host, oid],
            capture_output=True,
            text=True,
        )
        out = (r.stdout or "").strip()
        err = (r.stderr or "").strip()
        if r.returncode != 0 or not out:
            raise RuntimeError(f"snmpget failed for OID {oid}: rc={r.returncode} stdout={out} stderr={err}")

        if " = " in out:
            rhs = out.split(" = ", 1)[1].strip()
        else:
            rhs = out
        if ": " in rhs:
            rhs = rhs.split(": ", 1)[1].strip()
        return rhs.strip().strip('"')


def _snmp_parse_float(s: str) -> Optional[float]:
    try:
        return float(str(s).strip().strip('"'))
    except Exception:
        return None


def snmp_read_wifi(
    host: str,
    community: str,
    rssi_oid: str,
    ssid_oid: Optional[str] = None,
    bssid_oid: Optional[str] = None,
    timeout_s: float = 1.0,
) -> Tuple[bool, Optional[float], Optional[str], Optional[str]]:
    """
    Returns: (connected, rssi_dbm, ssid, bssid)
    """
    rssi_raw = _snmpget_raw(host, community, rssi_oid, timeout_s=timeout_s)
    rssi_dbm = _snmp_parse_float(rssi_raw)

    ssid = None
    bssid = None

    if ssid_oid:
        try:
            ssid = _snmpget_raw(host, community, ssid_oid, timeout_s=timeout_s).strip().strip('"')
        except Exception:
            ssid = None

    if bssid_oid:
        try:
            bssid = _snmpget_raw(host, community, bssid_oid, timeout_s=timeout_s).strip().strip('"')
        except Exception:
            bssid = None

    connected = rssi_dbm is not None
    return connected, rssi_dbm, ssid, bssid


def _make_manual_pose_provider():
    """
    Support either ManualPoseProvider or ClickMapPoseProvider depending on what exists.
    """
    from app import pose_sources

    if hasattr(pose_sources, "ManualPoseProvider"):
        return pose_sources.ManualPoseProvider()
    if hasattr(pose_sources, "ClickMapPoseProvider"):
        return pose_sources.ClickMapPoseProvider()

    raise RuntimeError(
        "No manual pose provider found. Expected ManualPoseProvider or ClickMapPoseProvider in app.pose_sources."
    )


def main():
    p = argparse.ArgumentParser()

    # Output
    p.add_argument("--db", default="data/run.sqlite")

    # Sampling
    p.add_argument("--hz", type=float, default=2.0)
    p.add_argument("--commit_every", type=int, default=10)

    # Pose selection
    p.add_argument("--pose_source", default="manual", choices=["manual", "ros1"])
    p.add_argument("--pose_topic", default="/amcl_pose")
    p.add_argument(
        "--manual_pose",
        action="store_true",
        help="Enable manual pose entry (used in laptop/manual workflows).",
    )

    # Wi-Fi selection (Linux interface) — used when SNMP is not enabled
    p.add_argument("--interface", default="auto", help="Linux Wi-Fi interface (auto, wlp9s0, etc).")

    # SNMP Wi-Fi selection (MOXA)
    p.add_argument("--snmp_host", default="", help="Enable SNMP Wi-Fi read from this host (e.g. 192.168.1.254).")
    p.add_argument("--snmp_community", default="public")
    p.add_argument("--snmp_timeout_s", type=float, default=1.0)
    p.add_argument("--snmp_rssi_oid", default="", help="OID that returns RSSI dBm (string or numeric).")
    p.add_argument("--snmp_ssid_oid", default="", help="OID that returns SSID (optional).")
    p.add_argument("--snmp_bssid_oid", default="", help="OID that returns BSSID/MAC (optional).")

    # Optional probe
    p.add_argument("--probe_host", default="", help="Optional ping target (e.g. gateway IP).")
    p.add_argument("--probe_count", type=int, default=3)
    p.add_argument("--probe_timeout_s", type=int, default=1)

    args = p.parse_args()

    # -------------------------
    # Pose provider selection
    # -------------------------
    if args.pose_source == "manual":
        pose_provider = _make_manual_pose_provider()
        if args.manual_pose and hasattr(pose_provider, "start"):
            pose_provider.start()
    else:
        from app.pose_ros1 import Ros1AmclPoseProvider  # requires ROS env

        pose_provider = Ros1AmclPoseProvider(topic=args.pose_topic)
        if hasattr(pose_provider, "start"):
            pose_provider.start()

    # -------------------------
    # Wi-Fi source selection
    # -------------------------
    use_snmp = bool(args.snmp_host.strip())
    iface = None

    if use_snmp:
        if not args.snmp_rssi_oid.strip():
            raise RuntimeError("--snmp_host was provided but --snmp_rssi_oid is empty.")
    else:
        iface = args.interface
        if iface in ("", "auto"):
            iface = detect_wifi_interface()
            if not iface:
                raise RuntimeError(
                    "Could not auto-detect Wi-Fi interface. "
                    "Use --interface wlp9s0 (etc), OR use --snmp_host for MOXA SNMP mode."
                )

    # -------------------------
    # Print config
    # -------------------------
    print(f"Logging to: {args.db}")
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
        print(f"Using interface: {iface}")

    # -------------------------
    # Logger setup
    # -------------------------
    logger: Optional[SqliteLogger] = None
    opened = False
    prev_bssid = None
    disconnected_since = None
    last_commit = 0
    printed_waiting_pose = False

    try:
        logger = SqliteLogger(Path(args.db))
        logger.open()
        opened = True

        i = 0
        period = 1.0 / float(args.hz)

        while True:
            if hasattr(pose_provider, "should_stop") and pose_provider.should_stop():
                break

            if args.pose_source == "ros1" and hasattr(pose_provider, "has_pose") and not pose_provider.has_pose():
                if not printed_waiting_pose:
                    print("Waiting for first pose message...", flush=True)
                    printed_waiting_pose = True

            ts_ms = now_unix_ms()
            pose = pose_provider.get_pose()
            x, y, theta = _pose_to_xytheta(pose)

            # -------- Wi-Fi read --------
            connected = False
            ssid = None
            bssid = None
            rssi_dbm = None
            freq_mhz = None

            if use_snmp:
                try:
                    connected, rssi_dbm, ssid, bssid = snmp_read_wifi(
                        host=args.snmp_host,
                        community=args.snmp_community,
                        rssi_oid=args.snmp_rssi_oid,
                        ssid_oid=args.snmp_ssid_oid or None,
                        bssid_oid=args.snmp_bssid_oid or None,
                        timeout_s=args.snmp_timeout_s,
                    )
                except Exception:
                    connected, rssi_dbm, ssid, bssid = False, None, None, None
            else:
                wifi = read_wifi_status(iface)
                connected = bool(wifi.connected)
                ssid = wifi.ssid
                bssid = wifi.bssid
                rssi_dbm = wifi.rssi_dbm
                freq_mhz = wifi.freq_mhz

            # -------- events --------
            roam_event = 0
            disconnect_event = 0
            disconnect_streak_s = 0.0

            if connected and bssid and prev_bssid and bssid != prev_bssid:
                roam_event = 1
            if connected and bssid:
                prev_bssid = bssid

            if not connected:
                if disconnected_since is None:
                    disconnected_since = time.time()
                    disconnect_event = 1
                disconnect_streak_s = float(time.time() - disconnected_since)
            else:
                disconnected_since = None

            # -------- optional probe --------
            ping_loss = None
            ping_avg = None
            if args.probe_host:
                pr = ping(args.probe_host, count=args.probe_count, timeout_s=args.probe_timeout_s)
                ping_loss = pr.loss_pct
                ping_avg = pr.avg_ms

            row = {
                "ts_unix_ms": ts_ms,
                "x_m": x,
                "y_m": y,
                "theta_rad": theta,
                "connected": 1 if connected else 0,
                "ssid": ssid,
                "bssid": bssid,
                "rssi_dbm": rssi_dbm,
                "freq_mhz": freq_mhz,
                "roam_event": roam_event,
                "disconnect_event": disconnect_event,
                "disconnect_streak_s": disconnect_streak_s,
                "ping_loss_pct": ping_loss,
                "ping_avg_ms": ping_avg,
            }

            logger.insert_sample(row)

            i += 1
            if i % int(args.commit_every) == 0:
                logger.commit()
                last_commit = i
                print(
                    f"Collected {i} samples... pose=({x:.2f},{y:.2f}) "
                    f"wifi={'ok' if connected else 'DOWN'} rssi={rssi_dbm}",
                    flush=True,
                )

            time.sleep(period)

    except KeyboardInterrupt:
        pass

    finally:
        if logger is not None and opened:
            try:
                logger.commit()
            except Exception:
                pass
            try:
                logger.close()
            except Exception:
                pass
        print(f"✅ Done. Samples committed (last_commit={last_commit}). DB: {args.db}")


if __name__ == "__main__":
    main()
