#!/usr/bin/env python3
"""Download the DCF77 replay recordings into samples/.

The recordings are too large for the repository.  They are published as one
tarball; its SHA-256 is pinned below, so the download is verified against this
repo and not against the server that hosts it.

    python3 simulation/scripts/fetch_samples.py            # download + unpack
    python3 simulation/scripts/fetch_samples.py --url URL  # other mirror
    python3 simulation/scripts/fetch_samples.py --file F   # use a local tarball

Afterwards run `python3 simulation/scripts/wav_to_iq.py --all` to prepare the
replay artefacts.  See samples/PROVENANCE.md for the origin of each file.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
import tarfile
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SAMPLES_DIR = ROOT / "samples"

# Bump the version directory and SHA256 together when the tarball changes.
DEFAULT_URL = "https://ib-erckenbrecht.de/files/dcf77/v1/dcf77-samples.tar.gz"
SHA256 = "5e8648c71a21e3ce7a58feea86ea839461b5523b654012d445ad0c403a89a758"

ALLOWED_SUFFIXES = (".wav", ".md")


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def download(url: str, dest: Path) -> None:
    with urllib.request.urlopen(url, timeout=60) as resp, dest.open("wb") as out:
        total = int(resp.headers.get("Content-Length") or 0)
        done = 0
        while True:
            block = resp.read(1 << 20)
            if not block:
                break
            out.write(block)
            done += len(block)
            if total:
                print(f"\r  {done >> 20} / {total >> 20} MiB", end="", file=sys.stderr)
    print(file=sys.stderr)


def safe_extract(tar_path: Path, dest: Path) -> list[str]:
    """Extract only plain top-level .wav/.md files; refuse anything else."""
    names = []
    with tarfile.open(tar_path, "r:gz") as tar:
        for m in tar.getmembers():
            if (not m.isfile() or os.path.basename(m.name) != m.name
                    or not m.name.endswith(ALLOWED_SUFFIXES)):
                raise SystemExit(f"refusing unexpected archive member: {m.name!r}")
        for m in tar.getmembers():
            target = dest / m.name
            with tar.extractfile(m) as src, target.open("wb") as out:
                while block := src.read(1 << 20):
                    out.write(block)
            names.append(m.name)
    return names


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--url", default=os.environ.get("DCF77_SAMPLES_URL", DEFAULT_URL))
    ap.add_argument("--file", type=Path, help="use this local tarball instead of downloading")
    ap.add_argument("--force", action="store_true", help="overwrite existing files")
    args = ap.parse_args(argv)

    SAMPLES_DIR.mkdir(exist_ok=True)
    tar_path = args.file or (SAMPLES_DIR / ".dcf77-samples.tar.gz.part")
    try:
        if args.file is None:
            print(f"downloading {args.url}", file=sys.stderr)
            download(args.url, tar_path)
        digest = sha256_of(tar_path)
        if digest != SHA256:
            print(f"SHA-256 mismatch: got {digest}, expected {SHA256}", file=sys.stderr)
            return 1
        if not args.force:
            with tarfile.open(tar_path, "r:gz") as tar:
                clash = [m.name for m in tar.getmembers() if (SAMPLES_DIR / m.name).exists()]
            if clash:
                print("already present (use --force to overwrite): " + ", ".join(clash),
                      file=sys.stderr)
                return 1
        names = safe_extract(tar_path, SAMPLES_DIR)
    finally:
        if args.file is None and tar_path.exists():
            tar_path.unlink()

    print(f"unpacked {len(names)} files into {SAMPLES_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
