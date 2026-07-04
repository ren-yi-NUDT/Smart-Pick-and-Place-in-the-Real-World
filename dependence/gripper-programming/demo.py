#!/usr/bin/env python3
"""
Standalone CLI for the Robotiq 85 / 2F-85 gripper.

Examples:
    python demo.py activate                # first-time activation (only once per power-up)
    python demo.py open                    # fully open the fingers
    python demo.py close                   # fully close the fingers
    python demo.py pos 128                 # move to mid position
    python demo.py cycle                   # open -> close -> open, used for quick testing
    python demo.py status                  # print status once
    python demo.py watch                   # stream status until Ctrl-C
    python demo.py --port /dev/ttyUSB1 status

Run inside the `anygrasp` conda env so pymodbus 2.5.3 is available:
    conda activate anygrasp
"""

import argparse
import sys
import time

from robotiq_driver import Robotiq85, describe_status


def cmd_activate(g: Robotiq85, args) -> int:
    print(f"[activate] sending reset + activate sequence on {g.port} ...")
    ok = g.activate(reset_first=True, timeout=args.activate_timeout)
    if ok:
        print("[activate] OK, gripper is now activated (gSTA=3).")
        return 0
    print(
        "[activate] FAILED: gripper did not report activation complete within "
        f"{args.activate_timeout}s. Check: (1) power supply connected, "
        "(2) LED on the coupling - red blinking = fault, "
        "(3) baud rate / slave address correct."
    )
    return 2


def cmd_move_open(g: Robotiq85, args) -> int:
    if not g.is_activated():
        print("[error] gripper not activated. Run `activate` first.")
        return 2
    print(f"[open] pos=0 speed={args.speed} force={args.force}")
    g.open(speed=args.speed, force=args.force)
    final = g.wait_until_idle(timeout=args.timeout)
    print(f"[open] settled: {describe_status(final)}")
    return 0


def cmd_move_close(g: Robotiq85, args) -> int:
    if not g.is_activated():
        print("[error] gripper not activated. Run `activate` first.")
        return 2
    print(f"[close] pos=255 speed={args.speed} force={args.force}")
    g.close(speed=args.speed, force=args.force)
    final = g.wait_until_idle(timeout=args.timeout)
    print(f"[close] settled: {describe_status(final)}")
    return 0


def cmd_move_pos(g: Robotiq85, args) -> int:
    if not g.is_activated():
        print("[error] gripper not activated. Run `activate` first.")
        return 2
    print(f"[pos] target={args.position}/255 speed={args.speed} force={args.force}")
    g.move_to(args.position, speed=args.speed, force=args.force)
    final = g.wait_until_idle(timeout=args.timeout)
    print(f"[pos] settled: {describe_status(final)}")
    return 0


def cmd_cycle(g: Robotiq85, args) -> int:
    if not g.is_activated():
        print("[error] gripper not activated. Run `activate` first.")
        return 2
    for label, target in [("open", 0), ("close", 255), ("open", 0)]:
        print(f"[cycle] moving to {label} (pos={target})")
        g.move_to(target, speed=args.speed, force=args.force)
        final = g.wait_until_idle(timeout=args.timeout)
        print(f"[cycle] {label}: {describe_status(final)}")
        time.sleep(0.5)
    return 0


def cmd_release(g: Robotiq85, args) -> int:
    """Auto-release: PWM-limited slow open. Works even when 24V is missing."""
    print(f"[release] running auto-release for {args.duration}s ...")
    g.auto_release(direction_open=True, duration=args.duration)
    print(f"[release] done: {describe_status(g.read_status())}")
    return 0


def cmd_status(g: Robotiq85, args) -> int:
    s = g.read_status()
    print(describe_status(s))
    print("raw:", s)
    return 0


def cmd_watch(g: Robotiq85, args) -> int:
    print(f"[watch] streaming status every {args.interval}s, Ctrl-C to quit")
    try:
        while True:
            s = g.read_status()
            line = describe_status(s)
            print(f"{time.strftime('%H:%M:%S')}  {line}", flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[watch] stopped")
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port (default /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="baudrate (default 115200)")
    parser.add_argument("--slave", type=int, default=9, help="modbus slave address (default 9)")
    parser.add_argument("--speed", type=int, default=200, help="0..255 (default 200)")
    parser.add_argument("--force", type=int, default=150, help="0..255 (default 150)")
    parser.add_argument("--timeout", type=float, default=5.0, help="wait_until_idle timeout seconds (default 5)")
    parser.add_argument("--activate-timeout", type=float, default=10.0, help="activate timeout seconds (default 10)")

    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("activate", help="first-time activation (reset + activate)")
    sub.add_parser("open", help="fully open")
    sub.add_parser("close", help="fully close")
    p_pos = sub.add_parser("pos", help="move to a specific position 0..255")
    p_pos.add_argument("position", type=int, help="0=open, 255=closed")
    sub.add_parser("cycle", help="open -> close -> open")
    p_release = sub.add_parser("release", help="auto-release (slow open, works even on weak 24V)")
    p_release.add_argument("duration", type=float, nargs="?", default=2.0, help="seconds to keep auto-release active (default 2.0)")
    sub.add_parser("status", help="read status once")
    p_watch = sub.add_parser("watch", help="stream status every N seconds")
    p_watch.add_argument("interval", type=float, nargs="?", default=0.1, help="seconds between reads (default 0.1)")

    args = parser.parse_args()

    handlers = {
        "activate": cmd_activate,
        "open": cmd_move_open,
        "close": cmd_move_close,
        "pos": cmd_move_pos,
        "cycle": cmd_cycle,
        "release": cmd_release,
        "status": cmd_status,
        "watch": cmd_watch,
    }
    handler = handlers[args.command]

    g = Robotiq85(port=args.port, baudrate=args.baud, slave=args.slave)
    try:
        g.connect()
    except ConnectionError as e:
        print(f"[error] {e}")
        return 1
    except Exception as e:
        print(f"[error] unexpected: {e}")
        return 1

    try:
        return handler(g, args)
    except KeyboardInterrupt:
        print("\n[interrupt] user cancelled")
        return 130
    except IOError as e:
        print(f"[error] modbus communication failed: {e}")
        print("        check: power, cable, baud rate, slave address")
        return 3
    finally:
        g.disconnect()


if __name__ == "__main__":
    sys.exit(main())
