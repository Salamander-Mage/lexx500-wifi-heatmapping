#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

from app.wifi_linux import detect_wifi_interface, read_wifi_status
from app.probes import ping
from app.pose_sources import ClickMapPoseProvider
from app.logger_sqlite import SqliteLogger


def now_unix_ms() -> int:
    return int(time.time() * 1000)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--db", default="data/run.sqlite")
    p.add_argument("--interface", default="auto")
    p.add_argument("--hz", type=float, default=2.0)

    # Pose selection
    p.add_argument("--pose_source", default="manual", choices=["manual", "ros1"])
    p.add_argument("--pose_topic", default="/amcl_pose")

    # Manual pose entry (legacy / home walking)
    p.add_argument("--manual_pose", action="store_true", help="Enable manual pose entry in terminal (ClickMapPoseProvider).")

    # Optional probe
    p.add_argument("--probe_host", default="", help="Optional ping target (e.g. gateway IP).")
    p.add_argument("--probe_count", type=int, default=3)
    p.add_argument("--probe_timeout_s", type=int, default=1)

    # DB batching
    p.add_argument("--commit_every", type=int, default=10)

    args = p.parse_args()

    # -------------------------
    # Wi-Fi interface detection
    # -------------------------
    iface = args.interface
    if iface in ("", "auto"):
        iface = detect_wifi_interface()
        if not iface:
            # On robot NUC, there may be no Wi-Fi interface; user may switch to MOXA SNMP later.
            raise RuntimeError("Could not auto-detect Wi-Fi interface. Use --interface wlp9s0 (etc).")

    print(f"Using interface: {iface}")
    print(f"Logging to: {args.db}")
    print(f"Sample rate: {args.hz} Hz")
    print(f"Pose source: {args.pose_source}")

    # -------------------------
    # Pose provider selection
    # -------------------------
    if args.pose_source == "manual":
        pose_provider = ClickMapPoseProvider()
        if args.manual_pose:
            pose_provider.start()
        else:
            # Not started => pose stays at (0,0,0)
            pass
    else:
        # ROS1 pose provider (AMCL pose in map frame)
        from app.pose_ros1 import Ros1AmclPoseProvider  # requires ROS env in container
        pose_provider = Ros1AmclPoseProvider(topic=args.pose_topic)
        pose_provider.start()
        print(f"ROS1 pose topic: {args.pose_topic}")

    # -------------------------
    # Logger setup
    # -------------------------
    log = SqliteLogger(Path(args.db))
    log.open()

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

            # If ROS pose provider hasn't received first message yet, warn once
            if args.pose_source == "ros1" and hasattr(pose_provider, "has_pose") and not pose_provider.has_pose():
                if not printed_waiting_pose:
                    print("Waiting for first pose message...", flush=True)
                    printed_waiting_pose = True

            ts_ms = now_unix_ms()
            pose = pose_provider.get_pose()
            wifi = read_wifi_status(iface)

            roam_event = 0
            disconnect_event = 0
            disconnect_streak_s = 0.0

            if wifi.connected and wifi.bssid and prev_bssid and wifi.bssid != prev_bssid:
                roam_event = 1
            prev_bssid = wifi.bssid if wifi.connected else prev_bssid

            # Disconnect tracking
            if not wifi.connected:
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

                "connected": 1 if wifi.connected else 0,
                "ssid": wifi.ssid,
                "bssid": wifi.bssid,
                "rssi_dbm": wifi.rssi_dbm,
                "freq_mhz": wifi.freq_mhz,

                "roam_event": roam_event,
                "disconnect_event": disconnect_event,
                "disconnect_streak_s": disconnect_streak_s,

                "ping_loss_pct": ping_loss,
                "ping_avg_ms": ping_avg,
            }

            log.insert_sample(row)

            i += 1
            if i % args.commit_every == 0:
                log.commit()
                last_commit = i
                print(
                    f"Collected {i} samples... "
                    f"pose=({pose.x_m:.2f},{pose.y_m:.2f}) "
                    f"wifi={'ok' if wifi.connected else 'DOWN'} "
                    f"rssi={wifi.rssi_dbm}",
                    flush=True,
                )

            time.sleep(period)

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")

    finally:
        log.commit()
        log.close()
        print(f"✅ Done. Samples committed (last_commit={last_commit}). DB: {args.db}")


if __name__ == "__main__":
    main()
