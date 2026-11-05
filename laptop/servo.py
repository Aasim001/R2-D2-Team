#!/usr/bin/env python3
"""
Simple servo control: rotate the robot's servo (D2) to any angle. Nothing else moves.

  python servo.py --robot 1 90         # robot 1: go to 90 degrees, then exit
  python servo.py --robot 2            # robot 2: type commands (below)
  python servo.py --host 192.168.43.93 # or give the robot's IP yourself

Commands (type, then Enter):
  90        go to 90 degrees (0 - 180)
  +10 / -5  turn 10 more / 5 less from where it is
  sweep 20 120   go slowly from 20 to 120 and back
  q         quit

The robot moves the servo smoothly (1 degree every 8 ms: 0 -> 180 takes about 1.5 s).
Careful with the pickup arm fitted: the scissor arm only swings about 54 degrees from its folded position,
and pushing it past its end jams the arm - the stalled servo can then reset the ESP32.
"""
import argparse, json, os, socket, sys, time

PORT = 4210
HERE = os.path.dirname(os.path.abspath(__file__))


def robot_ip(n):
    """The IP tools/bind_robots.py saved for robot n (arena_calib_carN.json)."""
    try:
        return json.load(open(os.path.join(HERE, f"arena_calib_car{n}.json")))["host"]
    except (OSError, KeyError, ValueError):
        sys.exit(f"Don't know robot {n}'s IP - use --host <IP> instead.")


class Servo:
    def __init__(self, ip):
        self.addr = (ip, PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)

    def angle(self):
        """The angle the robot says the servo is at now (None if it doesn't answer)."""
        self.sock.sendto(b"status", self.addr)
        try:
            while True:
                reply = self.sock.recvfrom(4096)[0].decode(errors="replace")
                if reply.startswith("{"):
                    return json.loads(reply)["servo"]
        except (socket.timeout, ConnectionResetError, ValueError):
            return None

    def go(self, deg):
        """Send the servo to deg (0-180) and wait until it is there."""
        deg = max(0, min(180, int(round(deg))))
        self.sock.sendto(f"servo {deg}".encode(), self.addr)
        t0 = time.time()
        now = None
        while time.time() - t0 < 3:
            now = self.angle()
            if now == deg:
                break
            time.sleep(0.05)
        print(f"  servo at {now} deg" + ("" if now == deg else f" (asked for {deg})"))
        return now if now is not None else deg


def main():
    ap = argparse.ArgumentParser(description="Rotate the robot's servo to any angle.")
    ap.add_argument("angle", nargs="?", type=float, help="go to this angle and exit (else: type commands)")
    ap.add_argument("--robot", type=int, choices=(1, 2, 3), help="robot number (its IP from arena_calib_carN.json)")
    ap.add_argument("--host", help="robot IP (instead of --robot)")
    a = ap.parse_args()
    if not (a.host or a.robot):
        sys.exit("Which robot? Add --robot 1 (or --host <IP>).")
    servo = Servo(a.host or robot_ip(a.robot))

    now = servo.angle()
    if now is None:
        sys.exit(f"No answer from {servo.addr[0]} - is the robot on, and this PC on the same Wi-Fi?")
    print(f"Robot {servo.addr[0]}: servo at {now} deg")

    if a.angle is not None:                       # one angle, then exit
        servo.go(a.angle)
        return

    print("Type an angle (0-180), +N / -N, 'sweep A B', or q to quit.")
    while True:
        try:
            cmd = input(f"[{now} deg] > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if cmd in ("q", "quit", "exit"):
            break
        try:
            if cmd.startswith("sweep"):
                lo, hi = (float(v) for v in cmd.split()[1:3])
                for target in (lo, hi, lo):
                    now = servo.go(target)
            elif cmd[:1] in "+-":
                now = servo.go(now + float(cmd))
            elif cmd:
                now = servo.go(float(cmd))
        except ValueError:
            print("  ? type a number like 90, +10, -5, 'sweep 20 120' or q")


if __name__ == "__main__":
    main()
