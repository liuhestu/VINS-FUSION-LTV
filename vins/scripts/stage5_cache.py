#!/usr/bin/env python3
import csv
import hashlib
from pathlib import Path


def require_strictly_increasing(values, label):
    previous = None
    for value in values:
        if not isinstance(value, int) or value < 0:
            raise RuntimeError(f"{label} has an invalid timestamp")
        if previous is not None and value <= previous:
            raise RuntimeError(f"{label} timestamps are not strictly increasing")
        previous = value


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_csv(path, header, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(header)
        writer.writerows(rows)
