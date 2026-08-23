#!/usr/bin/env python3
"""Check or restore PiPER SDK command-control mode.

Default mode is read-only. Real mode switching requires:

    --execute --confirm RESTORE_CONTROL_MODE
"""

from __future__ import annotations

import argparse
import time

from lib.piper_sdk_control_utils import (
    arm_status_summary,
    command_ready_problems,
    connect_piper,
    move_mode_code,
    print_status,
)


CONFIRMATION_TOKEN = "RESTORE_CONTROL_MODE"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-name", default="can0")
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--move-mode", choices=("j", "l", "p"), default="j")
    parser.add_argument("--repeat", type=int, default=3, help="End-teach command repeat count")
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if not (1 <= args.speed_percent <= 100):
            raise ValueError("--speed-percent must be in range 1..100")
        if args.repeat < 1:
            raise ValueError("--repeat must be >= 1")
        if args.execute and args.confirm != CONFIRMATION_TOKEN:
            raise ValueError(f"Real mode switching requires --confirm {CONFIRMATION_TOKEN}")

        piper = connect_piper(args.can_name, enable=False, enable_timeout=0.0)
        before = arm_status_summary(piper)
        print_status("before", before)
        problems = command_ready_problems(before)
        print(f"execute={args.execute}")

        if not args.execute:
            if problems:
                print("status check: not ready for SDK command control.")
                for problem in problems:
                    print(f"  - {problem}")
            else:
                print("status check: already looks ready for SDK command control.")
            print(
                "dry-run only. Add "
                f"--execute --confirm {CONFIRMATION_TOKEN} to restore control mode."
            )
            return 0

        for index in range(args.repeat):
            piper.MotionCtrl_1(0x00, 0x00, 0x02)
            print(f"sent end-teach command {index + 1}/{args.repeat}")
            time.sleep(0.15)

        time.sleep(args.settle_time)
        after_end_teach = arm_status_summary(piper)
        print_status("after end-teach", after_end_teach)

        mode_code = move_mode_code(args.move_mode)
        piper.ModeCtrl(0x00, mode_code, 0, 0x00)
        print(f"sent ModeCtrl STANDBY move_mode=0x{mode_code:02x}")
        time.sleep(args.settle_time)
        print_status("after standby", arm_status_summary(piper))

        piper.ModeCtrl(0x01, mode_code, args.speed_percent, 0x00)
        print(
            "sent ModeCtrl CAN command mode "
            f"move_mode=0x{mode_code:02x} speed_percent={args.speed_percent}"
        )
        time.sleep(args.settle_time)
        after = arm_status_summary(piper)
        print_status("after", after)
        after_problems = command_ready_problems(after)
        if after_problems:
            print("ERROR: control mode was not restored completely.")
            for problem in after_problems:
                print(f"  - {problem}")
            return 2
        print("OK: PiPER is ready for SDK command control.")
        return 0
    except (RuntimeError, ValueError, TimeoutError) as error:
        print(f"ERROR: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
