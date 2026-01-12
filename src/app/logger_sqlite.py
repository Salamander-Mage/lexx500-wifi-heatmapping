import sqlite3
from pathlib import Path
from typing import Optional, Dict, Any


SCHEMA = """
CREATE TABLE IF NOT EXISTS samples (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  ts_unix_ms INTEGER NOT NULL,

  x_m REAL NOT NULL,
  y_m REAL NOT NULL,
  theta_rad REAL NOT NULL,

  connected INTEGER NOT NULL,
  ssid TEXT,
  bssid TEXT,
  rssi_dbm INTEGER,
  freq_mhz INTEGER,

  roam_event INTEGER NOT NULL,
  disconnect_event INTEGER NOT NULL,
  disconnect_streak_s REAL NOT NULL,

  ping_loss_pct REAL,
  ping_avg_ms REAL
);
"""


class SqliteLogger:
    def __init__(self, path: Path):
        self.path = path
        self.conn: Optional[sqlite3.Connection] = None

    def open(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(self.path)
        self.conn.execute("PRAGMA journal_mode=WAL;")
        self.conn.execute(SCHEMA)
        self.conn.commit()

    def insert_sample(self, row: Dict[str, Any]):
        assert self.conn is not None
        cols = ",".join(row.keys())
        qs = ",".join(["?"] * len(row))
        self.conn.execute(f"INSERT INTO samples ({cols}) VALUES ({qs})", list(row.values()))

    def commit(self):
        assert self.conn is not None
        self.conn.commit()

    def close(self):
        if self.conn is not None:
            self.conn.commit()
            self.conn.close()
            self.conn = None
