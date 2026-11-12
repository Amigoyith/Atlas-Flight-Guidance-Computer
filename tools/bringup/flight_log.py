"""Convert an SD flight log (FLTnnnn.BIN) to CSV.

The flight image appends one 512-byte snapshot every 100 ms, in the radio telemetry
format (App/Src/atlas_flight_app.c, flight_log_step). Each record is checked with its
CRC and decoded by remote_telemetry.decode_snapshot; damaged records are skipped.

Usage: python tools/bringup/flight_log.py FLT0001.BIN [out.csv]
"""
from __future__ import annotations
import csv
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from remote_telemetry import decode_snapshot

RECORD = 512


def flatten(value, prefix='') -> dict:
    """Nested dicts/lists to flat columns: flight.body_rate_mdps.0, ..."""
    if isinstance(value, dict):
        out = {}
        for k, v in value.items():
            out.update(flatten(v, f'{prefix}{k}.'))
        return out
    if isinstance(value, list):
        out = {}
        for i, v in enumerate(value):
            out.update(flatten(v, f'{prefix}{i}.'))
        return out
    return {prefix[:-1]: value}


def read_log(data: bytes) -> tuple[list[dict], int]:
    """Decode every whole record. @return (rows, damaged record count)."""
    rows, damaged = [], 0
    for at in range(0, len(data) - RECORD + 1, RECORD):
        try:
            d = decode_snapshot(data[at:at + RECORD].hex())
        except ValueError:
            d = None
        if d is None:
            damaged += 1
        else:
            rows.append(flatten(d))
    return rows, damaged + (1 if len(data) % RECORD else 0)


def convert(source: Path, target: Path) -> tuple[int, int]:
    rows, damaged = read_log(source.read_bytes())
    columns = list(rows[0]) if rows else []
    with target.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    return len(rows), damaged


if __name__ == '__main__':
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    src = Path(sys.argv[1])
    dst = Path(sys.argv[2]) if len(sys.argv) == 3 else src.with_suffix('.csv')
    good, bad = convert(src, dst)
    print(f'{good} records written to {dst}; {bad} damaged or partial records skipped')
