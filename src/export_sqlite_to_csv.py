import argparse
import sqlite3
from pathlib import Path
import pandas as pd


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--db", required=True, help="Path to SQLite DB")
    p.add_argument("--out", default="data/export.csv", help="Output CSV path")
    args = p.parse_args()

    conn = sqlite3.connect(args.db)
    df = pd.read_sql_query("SELECT * FROM samples", conn)
    conn.close()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out, index=False)

    print(f"✅ exported {len(df)} rows -> {out}")


if __name__ == "__main__":
    main()
