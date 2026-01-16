#!/usr/bin/env python3
import argparse
import shutil
import subprocess
import time
from pathlib import Path
from typing import Optional, Tuple

from app.logger_sqlite import SqliteLogger
from app.probes import ping


def now_unix_ms() -> int:
    return int(time.time() * 1000)


def snmpget_value(host: str, community: str, oid: str, timeout_s: int = 1) -> str:
    """
    Returns the SNMP value as a string (best-effort parsing).
    Uses numeric OID so it works without MIB files.
    """
    if shutil.which("snmpget") is None:
        raise RuntimeError("snmpget not found. Install SNMP tools (apt-get install -y snmp) or run in a container that has snmpget.")

    # Example output:
    # SNMPv2-SMI::enterprises.... = STRING: "-48"
    cmd = [
        "snmpget",
        "-v2c",
        "-c",
        community,
        "-Oqv",                 # output only the value (quiet, value only)
        "-t",
        str(timeout_s),
        host,
        oid,
    ]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"snmpget failed ({r.returncode}): {r.stderr.strip()}")
    return r.stdout.strip().strip('"')


def snmp_read_wifi(
    host: str,
    community: str,
    rssi_oid: str,
    ssid_oid: Optional[str],
    bssid_oid: Optional[str],
    timeout_s: int = 1,
) -> Tuple[bool, Optional[str], Optional[str], Optional[float]]:
    """
    Returns: (connected, ssid, bssid, rssi_dbm)
    """
    # RSSI is required; SSID/BSSID optional.
    rssi_s = snmpget_value(host, community, rssi_oid, timeout_s=timeout_s)
    ssid = snmpget_value(host, community, ssid_oid, timeout_s=timeout_s) if ssid_oid else None
    bssid = snmpget_value(host, community, bssid_oid, timeout_s=timeout_s) if bssid_oid else None

    rssi_dbm: Optional[float] = None
    try:
        # Example "-48"
        rssi_dbm = float(rssi_s)
    except Exception:
        rssi_dbm = None

    connected = rssi_dbm is not None
    return connected, ssid, bssid, rssi_dbm


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

    # Manual pose entry (legacy / home walking)
    p.add_argument("--manual_pose", action="store_true", help="Enable manual pose entry in terminal (ClickMapPoseProvider).")

    # Wi-Fi source selection:
    # - default: Linux interface (iw)
    # - SNMP: specify --snmp_host and required OIDs
    p.add_argument("--interface", default="auto", help="Linux Wi-Fi interface (ignored if SNMP is enabled).")
    p.add_argument("--snmp_host", default="", help="If set, use SNMP for Wi-Fi metrics (e.g., MOXA).")
    p.add_argument("--snmp_community", default="public")
    p.add_argument("--snmp_rssi_oid", default="", help="SNMP OID for RSSI (required if --snmp_host is set).")
    p.add_argument("--snmp_ssid_oid", default="", help="SNMP OID for SSID (optional).")
    p.add_argument("--snmp_bssid_oid", default="", help="SNMP OID for BSSID (optional).")
    p.add_argument("--snmp_timeout_s", type=int, default=1)

    # Optional ping probe
    p.add_argument("--probe_host", default="", help="Optional ping target (e.g. gateway IP).")
    p.add_argument("--probe_count", type=int, default=3)
    p.add_argument("--probe_timeout_s", type=int, default=1)

    args = p.parse_args()

    use_snmp = bool(args.snmp_host)

    # -------------------------
    # Pose provider selection
    # -------------------------
    if args.pose_source == "manual":
        from app.pose_sources import ClickMapPoseProvider
        pose_provider = ClickMapPoseProvider()
        if args.manual_pose:
            pose_provider.start()
    else:
        from app.pose_ros1 import Ros1AmclPoseProvider  # requires ROS env in container
        pose_provider = Ros1AmclPoseProvider(topic=args.pose_topic)
        pose_provider.start()

    # -------------------------
    # Wi-Fi config print
    # -------------------------
    print(f"Logging to: {args.db}")
    print(f"Sample rate: {args.hz} Hz")
    print(f"Pose source: {args.pose_source}")
    if args.pose_source == "ros1":
        print(f"ROS1 pose topic: {args.pose_topic}")

    if use_snmp:
        if not args.snmp_rssi_oid:
            raise SystemExit("ERROR: --snmp_rssi_oid is required when --snmp_host is set.")
        print(f"Wi-Fi source: SNMP host={args.snmp_host} community={args.snmp_community}")
        print(f"  RSSI OID:  {args.snmp_rssi_oid}")
        if args.snmp_ssid_oid:
            print(f"  SSID OID:  {args.snmp_ssid_oid}")
        if args.snmp_bssid_oid:
            print(f"  BSSID OID: {args.snmp_bssid_oid}")
    else:
        # Linux interface path
        from app.wifi_linux import detect_wifi_interface, read_wifi_status

        iface = args.interface
        if iface in ("", "auto"):
            iface = detect_wifi_interface()
            if not iface:
                raise RuntimeError("Could not auto-detect Wi-Fi interface. Use --interface wlp9s0 (etc).")
        args.interface = iface
        print(f"Wi-Fi source: Linux interface={args.interface}")

    # -------------------------
    # Logger setup
    # -------------------------
    logger = SqliteLogger(Path(args.db))
    logger.open()

    prev_bssid = None
    disconnected_since = None
    last_commit = 0
    printed_waiting_pose = False

    try:
        i = 0
        period = 1.0 / args.hz

        while True:
            # Stop support for manual mode (press 'q' in ClickMapPoseProvider)
            if hasattr(pose_provider, "should_stop") and pose_provider.should_stop():
                print("Stopping (manual 'q').")
                break

            if args.pose_source == "ros1" and hasattr(pose_provider, "has_pose") and not pose_provider.has_pose():
                if not printed_waiting_pose:
                    print("Waiting for first pose message...", flush=True)
                    printed_waiting_pose = True

            ts_ms = now_unix_ms()
            pose = pose_provider.get_pose()

            # Read Wi-Fi
            if use_snmp:
                connected, ssid, bssid, rssi_dbm = snmp_read_wifi(
                    host=args.snmp_host,
                    community=args.snmp_community,
                    rssi_oid=args.snmp_rssi_oid,
                    ssid_oid=args.snmp_ssid_oid or None,
                    bssid_oid=args.snmp_bssid_oid or None,
                    timeout_s=args.snmp_timeout_s,
                )
                freq_mhz = None
            else:
                from app.wifi_linux import read_wifi_status
                wifi = read_wifi_status(args.interface)
                connected = wifi.connected
                ssid = wifi.ssid
                bssid = wifi.bssid
                rssi_dbm = wifi.rssi_dbm
                freq_mhz = wifi.freq_mhz

            # Events
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

            # Optional ping probe
            ping_loss = None
            ping_avg = None
            if args.probe_host:
                pr = ping(args.probe_host, count=args.probe_count, timeout_s=args.probe_timeout_s)
                ping_loss = pr.loss_pct
                ping_avg = pr.avg_ms

            row = {
                "ts_unix_ms": ts_ms,
                "x_m": pose.x_m,
                "y_m": pose.y_m,
                "theta_rad": pose.theta_rad,
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

            # IMPORTANT: logger API is insert_sample()
            logger.insert_sample(row)

            i += 1
            if i % args.commit_every == 0:
                logger.commit()
                last_commit = i
                print(
                    f"Collected {i} samples... pose=({pose.x_m:.2f},{pose.y_m:.2f}) "
                    f"wifi={'ok' if connected else 'DOWN'} rssi={rssi_dbm}",
                    flush=True,
                )

            time.sleep(period)

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")

    finally:
        # Be defensive: only commit/close if open succeeded
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
