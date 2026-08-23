#!/usr/bin/env python3
"""Read current PIPER flange feedback and print the calibrated probe-tip pose."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import time

from lib.piper_sdk_control_utils import (
    DEFAULT_TCP_CALIBRATION,
    connect_piper,
    load_tcp_offset_m_rad,
    read_probe_tip_pose,
)


ROOT = Path(__file__).resolve().parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-name", default="can0")
    parser.add_argument("--tcp-calibration", type=Path, default=DEFAULT_TCP_CALIBRATION)
    parser.add_argument("--output-root", type=Path, default=ROOT / "outputs")
    parser.add_argument("--feedback-wait", type=float, default=0.5)
    args = parser.parse_args()

    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    output = output_root / f"probe_tip_pose_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

    try:
        tcp_offset, tcp_status = load_tcp_offset_m_rad(args.tcp_calibration)
        piper = connect_piper(args.can_name, enable=False, piper_init=False)
        try:
            time.sleep(args.feedback_wait)
            report = read_probe_tip_pose(piper, tcp_offset)
            report["tcp_calibration"] = str(args.tcp_calibration.expanduser().resolve())
            report["tcp_calibration_status"] = tcp_status
            print(json.dumps(report, ensure_ascii=False, indent=2))
            output.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"Saved: {output}")
        finally:
            piper.DisconnectPort(thread_timeout=0.5)
        return 0
    except (OSError, RuntimeError, ValueError, TimeoutError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
