import subprocess
import re
from dataclasses import dataclass
from typing import Optional


@dataclass
class WifiStatus:
    connected: bool
    ssid: Optional[str]
    bssid: Optional[str]
    rssi_dbm: Optional[int]
    freq_mhz: Optional[int]


def detect_wifi_interface() -> Optional[str]:
    try:
        proc = subprocess.run(["iw", "dev"], capture_output=True, text=True, timeout=1.0)
        m = re.search(r"Interface\s+(\S+)", proc.stdout)
        return m.group(1) if m else None
    except Exception:
        return None


def read_wifi_status(interface: str) -> WifiStatus:
    """
    Uses `iw dev <iface> link`.
    """
    try:
        proc = subprocess.run(
            ["iw", "dev", interface, "link"],
            capture_output=True,
            text=True,
            timeout=1.0,
        )
        out = proc.stdout

        if "Not connected" in out or out.strip() == "":
            return WifiStatus(False, None, None, None, None)

        bssid_match = re.search(r"Connected to ([0-9a-f:]{17})", out)
        ssid_match = re.search(r"SSID:\s*(.+)", out)
        signal_match = re.search(r"signal:\s*(-?\d+)\s*dBm", out)
        freq_match = re.search(r"freq:\s*(\d+)", out)

        ssid = ssid_match.group(1).strip() if ssid_match else None
        bssid = bssid_match.group(1) if bssid_match else None
        rssi = int(signal_match.group(1)) if signal_match else None
        freq = int(freq_match.group(1)) if freq_match else None

        return WifiStatus(True, ssid, bssid, rssi, freq)

    except Exception:
        return WifiStatus(False, None, None, None, None)
