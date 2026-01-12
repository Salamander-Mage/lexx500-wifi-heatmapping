import subprocess
import re
from dataclasses import dataclass
from typing import Optional


@dataclass
class PingResult:
    ok: bool
    loss_pct: Optional[float]
    avg_ms: Optional[float]


def ping(host: str, count: int = 3, timeout_s: int = 1) -> PingResult:
    """
    Linux ping summary parser. Returns loss% and avg ms if available.
    """
    try:
        proc = subprocess.run(
            ["ping", "-c", str(count), "-W", str(timeout_s), host],
            capture_output=True,
            text=True,
            timeout=max(3, count * (timeout_s + 1)),
        )
        out = proc.stdout + "\n" + proc.stderr

        loss_m = re.search(r"(\d+)%\s*packet loss", out)
        rtt_m = re.search(r"rtt .* = [\d\.]+/([\d\.]+)/", out)  # avg is 2nd field

        loss = float(loss_m.group(1)) if loss_m else None
        avg = float(rtt_m.group(1)) if rtt_m else None

        ok = (loss is not None and loss < 100.0)
        return PingResult(ok, loss, avg)

    except Exception:
        return PingResult(False, None, None)
