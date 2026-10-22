#!/usr/bin/env python3
"""Fetch pinned documented NMOS 6502 tests (Python 3 standard library only).

Paths are relative to the current directory; the opcode list is repo-relative.
Rerun to resume: valid cached files are reused only with a matching REVISION.
REVISION records provenance, not completeness. Do not mix other data into the
output directory. Optional samples preserve case values, not JSON whitespace.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from http.client import HTTPException
import json
from pathlib import Path
import sys
import tempfile
import time
from urllib.error import URLError
from urllib.request import urlopen

FIXTURES = Path(__file__).resolve().parents[1] / "tests/assets/nmos6502"
REVISION = (FIXTURES / "REVISION").read_text().strip()
BASE_URL = f"https://raw.githubusercontent.com/SingleStepTests/65x02/{REVISION}"
OPCODES = FIXTURES / "opcodes.json"
WORKERS = 4
ATTEMPTS = 3
TIMEOUT = 30  # Socket timeout, in seconds.


def atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".nmos-", delete=False) as f:
            temporary = Path(f.name)
            f.write(data)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def parse_cases(data):
    cases = json.loads(data)
    if not isinstance(cases, list) or len(cases) != 10000:
        raise ValueError("expected a JSON array of 10,000 cases")
    if any(not isinstance(case, dict) or not {"name", "initial", "final", "cycles"} <= case.keys()
           for case in cases):
        raise ValueError("test case is missing required fields")
    return cases


def fetch_opcode(opcode, output):
    filename = f"{opcode:02x}.json"
    path = output / filename
    if path.exists():
        try:
            cases = parse_cases(path.read_bytes())
            return cases[:8], "cached"
        except (ValueError, OSError) as error:
            print(f"{filename}: replacing invalid cache: {error}", file=sys.stderr, flush=True)
    url = f"{BASE_URL}/6502/v1/{filename}"
    for attempt in range(1, ATTEMPTS + 1):
        try:
            with urlopen(url, timeout=TIMEOUT) as response:
                data = response.read()
            cases = parse_cases(data)
            atomic_write(path, data)
            return cases[:8], "downloaded"
        except (OSError, URLError, HTTPException, ValueError) as error:
            if attempt == ATTEMPTS:
                raise RuntimeError(f"{url}: failed after {ATTEMPTS} attempts: {error}") from error
            print(f"{filename}: attempt {attempt}/{ATTEMPTS}: {error}; retrying",
                  file=sys.stderr, flush=True)
            time.sleep(attempt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("target/nmos6502/v1"),
                        help="data directory (default: target/nmos6502/v1)")
    parser.add_argument("--sample-output", type=Path,
                        help="write the first 8 cases per opcode as one JSON array")
    args = parser.parse_args()
    opcodes = json.loads(OPCODES.read_text())
    if (not isinstance(opcodes, list) or len(opcodes) != 151
            or any(type(op) is not int or not 0 <= op <= 255 for op in opcodes)
            or len(set(opcodes)) != 151):
        raise ValueError(f"{OPCODES}: expected 151 distinct opcode bytes")

    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    revision_path = output / "REVISION"
    if revision_path.exists():
        if revision_path.read_text().strip() != REVISION:
            raise ValueError(f"{revision_path}: revision mismatch; use a fresh --output directory")
    elif any(output.glob("*.json")):
        raise ValueError(f"{output}: JSON files without REVISION; use a fresh --output directory")
    # Written before downloads so interrupted runs can safely resume per file.
    atomic_write(revision_path, (REVISION + "\n").encode())

    samples = {}
    failures = []
    with ThreadPoolExecutor(max_workers=WORKERS) as pool:
        futures = {pool.submit(fetch_opcode, op, output): op for op in opcodes}
        for future in as_completed(futures):
            opcode = futures[future]
            try:
                samples[opcode], status = future.result()
                print(f"[{len(samples)}/{len(opcodes)}] {opcode:02x}.json {status}", flush=True)
            except Exception as error:
                failures.append(f"{opcode:02x}.json: {error}")
                print(f"ERROR: {failures[-1]}", file=sys.stderr, flush=True)
    if failures:
        raise RuntimeError(f"{len(failures)} downloads failed; rerun to retry (valid files are cached)")
    if args.sample_output:
        sample = [case for op in opcodes for case in samples[op]]
        # One case per line keeps the vendored fixture compact and diffable.
        text = "[\n" + ",\n".join(json.dumps(case, separators=(",", ":")) for case in sample) + "\n]\n"
        atomic_write(args.sample_output, text.encode())
        print(f"Wrote {len(sample)} unchanged cases to {args.sample_output}")
    print(f"Complete: {len(opcodes)} opcode files at {output}, revision {REVISION}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        sys.exit(f"ERROR: {error}")
