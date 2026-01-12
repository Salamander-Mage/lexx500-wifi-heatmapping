import subprocess
import re


def detect_wifi_interface() -> str | None:
    """
    Returns the first wireless interface name found by `iw dev`.
    """
    try:
        proc = subprocess.run(["iw", "dev"], capture_output=True, text=True, timeout=1.0)
        out = proc.stdout
        m = re.search(r"Interface\s+(\S+)", out)
        return m.group(1) if m else None
    except Exception:
        return None


def read_wifi_iw(interface: str):
    """
    Reads current Wi-Fi link info using `iw`.
    Returns (rssi_dbm, bssid, ssid) or (None, None, None) if unavailable.
    """
    try:
        proc = subprocess.run(
            ["iw", "dev", interface, "link"],
            capture_output=True,
            text=True,
            timeout=1.0,
        )
        out = proc.stdout

        if "Not connected" in out:
            return None, None, None

        bssid_match = re.search(r"Connected to ([0-9a-f:]{17})", out)
        ssid_match = re.search(r"SSID:\s*(.+)", out)
        signal_match = re.search(r"signal:\s*(-?\d+)\s*dBm", out)

        if not signal_match:
            return None, None, None

        rssi = int(signal_match.group(1))
        bssid = bssid_match.group(1) if bssid_match else "UNKNOWN"
        ssid = ssid_match.group(1).strip() if ssid_match else "UNKNOWN"

        return rssi, bssid, ssid

    except Exception:
        return None, None, None
