# src/app/collect.py
import argparse
import time
from pathlib import Path

from app.wifi_linux import detect_wifi_interface, read_wifi_status
from app.probes import ping
from app.logger_sqlite import SqliteLogger
from app.pose_sources import ClickMapPoseProvider


def now_unix_ms() -> int:
    return int(time.time() * 1000)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--db", default="data/run.sqlite")
    p.add_argument("--interface", default="auto")
    p.add_argument("--hz", type=float, default=2.0)

    # Pose source selection
    p.add_argument("--pose_source", choices=["click", "ros_tf"], default="click")
    p.add_argument("--ros_map_frame", default="map")
    p.add_argument("--ros_base_frame", default="base_link")
    p.add_argument("--ros_odom_frame", default="odom")
    p.add_argument("--ros_tf_timeout_s", type=float, default=0.25)

    # Optional ping probe
    p.add_argument("--probe_host", default="", help="Optional ping target (e.g. gateway IP).")
    p.add_argument("--probe_count", type=int, default=3)
    p.add_argument("--probe_timeout_s", type=int, default=1)

    # DB commit behavior
    p.add_argument("--commit_every", type=int, default=10)

    args = p.parse_args()

    iface = args.interface
    if iface in ("", "auto"):
        iface = detect_wifi_interface()
        if not iface:
            raise RuntimeError("Could not auto-detect Wi-Fi interface. Use --interface wlp9s0 (etc).")

    print(f"Using interface: {iface}")
    print(f"Logging to: {args.db}")
    print(f"Sample rate: {args.hz} Hz")
    print(f"Pose source: {args.pose_source}")

    # Pose provider init
    pose_provider = None
    ros_pose = None

    if args.pose_source == "click":
        pose_provider = ClickMapPoseProvider()
        pose_provider.start()
        print("Click pose mode: use the click-map window. Press 'q' to stop.")
    else:
        from app.pose_ros_tf import RosTfPoseProvider

        ros_pose = RosTfPoseProvider(
            map_frame=args.ros_map_frame,
            base_frame=args.ros_base_frame,
            odom_frame=args.ros_odom_frame,
            timeout_s=args.ros_tf_timeout_s,
            allow_fallback_odom=True,
        )
        ros_pose.start()
        print(f"ROS TF mode: using TF {args.ros_map_frame}->{args.ros_base_frame} (fallback {args.ros_odom_frame}->{args.ros_base_frame})")

    log = SqliteLogger(Path(args.db))
    log.open()

    prev_bssid = None
    disconnected_since = None
    last_commit = 0

    period = 1.0 / args.hz
    i = 0

    try:
        while True:
            # Stop condition depending on pose source
            if args.pose_source == "click":
                if pose_provider.should_stop():
                    print("Stopping (manual 'q').")
                    break
            else:
                if not ros_pose.ok():
                    print("Stopping (ROS shutdown).")
                    break

            ts_ms = now_unix_ms()

            # Pose read
            if args.pose_source == "click":
                pose = pose_provider.get_pose()
            else:
                pose = ros_pose.get_pose()

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
                print(f"Collected {i} samples... pose=({pose.x_m:.2f},{pose.y_m:.2f}) wifi={'ok' if wifi.connected else 'DOWN'} rssi={wifi.rssi_dbm}")

            time.sleep(period)

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")

    finally:
        log.commit()
        log.close()
        print(f"✅ Done. Samples committed (last_commit={last_commit}). DB: {args.db}")


if __name__ == "__main__":
    main()
