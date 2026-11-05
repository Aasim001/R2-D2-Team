"""Wheel balance: make all four wheels turn at the same speed for the same command.

LIFT THE ROBOT FIRST (wheels off the ground, free to spin). Then:
  python tools/wheel_balance.py --robot 2            # measure, try new gains, keep them only if they help
  python tools/wheel_balance.py --robot 2 --measure  # only measure (changes nothing)
  python tools/wheel_balance.py --robot 2 --skip fl  # leave a wheel alone (e.g. one with a bad encoder)
  python tools/wheel_balance.py --robot 2 --reset    # all gains back to 1.0

How: all four wheels run forward, then backward, at the same PWM; each wheel's speed comes from its encoder.
A wheel that is faster than the average gets a gain below 1 (slowed down), a slower one above 1. The firmware
(robot_main.ino, wheelGain) keeps the gains in the ESP32's flash, so they survive restarts.
Safety rules (2026-10-10 review):
  - a wheel whose encoder misses counts (count far below its C1 edge count), reads very differently forward and
    backward, or hardly counts at all is NOT balanced - its encoder can't be trusted (check it with bot.py watch)
  - a correction of more than 25 % is refused: N20s differ by 10-20 %, so more means a mechanical/encoder fault
  - new gains are confirmed on the robot, measured again, and saved only if the wheels match better than before
Lifted balance only equalises the free-running speeds; on the floor the arena software still steers.
"""
import argparse, json, os, socket, struct, sys, time

PORT = 4210
NAMES = ["FL", "FR", "RL", "RR"]
LAPTOP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAX_CORRECTION = 0.25


class Robot:
    def __init__(self, ip):
        self.addr, self.seq = (ip, PORT), 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)

    def ask(self, text, want_json=False):
        self.sock.sendto(text.encode(), self.addr)
        try:
            while True:
                r = self.sock.recvfrom(4096)[0].decode(errors="replace")
                if not want_json or r.startswith("{"):
                    return json.loads(r) if want_json else r
        except (socket.timeout, ConnectionResetError, ValueError):
            return None

    def drive(self, left, right, secs):
        t = time.time()
        while time.time() - t < secs:
            self.seq = (self.seq + 1) & 0xFFFF
            self.sock.sendto(b"DR" + struct.pack("<hhH", left, right, self.seq), self.addr)
            time.sleep(0.05)

    def stop(self):
        for _ in range(3):
            self.drive(0, 0, 0.05)
        self.ask("stop")


def measure(robot, pwm, secs):
    """Per wheel and direction: (counts/s, |counts| / C1 edges) - the first 0.6 s (speeding up) left out."""
    out = {}
    for name, sign in (("forward", 1), ("backward", -1)):
        robot.drive(sign * pwm, sign * pwm, 0.6)
        a, ta = robot.ask("status", True), time.time()
        robot.drive(sign * pwm, sign * pwm, secs)
        b, tb = robot.ask("status", True), time.time()
        robot.stop()
        time.sleep(0.8)
        if a is None or b is None:
            sys.exit("The robot stopped answering - Wi-Fi?")
        speed = [abs(b["cnt"][i] - a["cnt"][i]) / (tb - ta) for i in range(4)]
        edges = [b["edges"][i] - a["edges"][i] for i in range(4)] if "edges" in a else [None] * 4
        ratio = [abs(b["cnt"][i] - a["cnt"][i]) / e if e else None for i, e in enumerate(edges)]
        out[name] = (speed, ratio)
    return out


def trusted(m, skip):
    """Which wheels' encoders can be believed for balancing (and why not)."""
    fwd, back = m["forward"][0], m["backward"][0]
    avg = [(f + b) / 2 for f, b in zip(fwd, back)]
    med = sorted(avg)[len(avg) // 2]
    why = {}
    for i in range(4):
        if NAMES[i].lower() in skip:
            why[i] = "skipped (--skip)"
        elif avg[i] < 0.3 * med:
            why[i] = "its encoder hardly counted"
        elif any(r is not None and r < 0.9 for r in (m["forward"][1][i], m["backward"][1][i])):
            why[i] = "its count is far below its C1 edges (C2 wire?)"
        elif abs(fwd[i] - back[i]) > 0.15 * max(fwd[i], back[i]):
            why[i] = f"forward {fwd[i]:.0f} vs backward {back[i]:.0f} counts/s differ too much"
    return avg, why


def spread(avg, ok):
    v = [a for a, o in zip(avg, ok) if o]
    return (max(v) - min(v)) / max(v) if len(v) >= 2 else 0.0


def show(m, gains, why=None):
    print("  wheel   forward  backward  (counts/s)   gain")
    for i, n in enumerate(NAMES):
        note = f"   <- {why[i]}" if why and i in why else ""
        print(f"  {n}    {m['forward'][0][i]:8.0f}  {m['backward'][0][i]:8.0f}            {gains[i]:.3f}{note}")


def set_gains(robot, gains):
    """Send the gains (RAM only) and check the robot really has them."""
    for _ in range(3):
        for i in range(4):
            robot.ask(f"gain {NAMES[i].lower()} {gains[i]:.3f}")
        st = robot.ask("status", True)
        if st and all(abs(st["gain"][i] - gains[i]) < 0.0015 for i in range(4)):
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, choices=(1, 2, 3))
    ap.add_argument("--host")
    ap.add_argument("--pwm", type=int, default=130, help="test speed (lane driving uses about 95-160)")
    ap.add_argument("--secs", type=float, default=2.0)
    ap.add_argument("--measure", action="store_true", help="only measure")
    ap.add_argument("--reset", action="store_true", help="set all gains back to 1.0")
    ap.add_argument("--skip", default="", help="wheels to leave alone, e.g. fl or fl,rr")
    ap.add_argument("--yes", action="store_true", help="don't ask whether the robot is lifted")
    a = ap.parse_args()
    if not 100 <= a.pwm <= 200 or not 0.5 <= a.secs <= 4:
        sys.exit("--pwm 100-200 and --secs 0.5-4, please")
    ip = a.host
    if not ip and a.robot:
        try:
            ip = json.load(open(os.path.join(LAPTOP, f"arena_calib_car{a.robot}.json")))["host"]
        except (OSError, KeyError, ValueError):
            sys.exit(f"Don't know robot {a.robot}'s IP - use --host")
    if not ip:
        sys.exit("Which robot? --robot N or --host IP")
    robot = Robot(ip)
    st = robot.ask("status", True)
    if st is None:
        sys.exit(f"No answer from {ip} (or a broken status reply).")
    if "gain" not in st:
        sys.exit("This robot's firmware has no wheel balance yet - upload the new firmware first.")
    old = [float(g) for g in st["gain"]]
    print(f"{st.get('name', ip)}: gains now " + "  ".join(f"{n} {g:.3f}" for n, g in zip(NAMES, old)))
    if a.reset:
        print(robot.ask("gain reset"))
        return
    if not a.yes and input("Is the robot LIFTED, wheels free to spin? (y/n) ").strip().lower() != "y":
        sys.exit("Lift it first - otherwise it would drive off.")
    skip = [s.strip().lower() for s in a.skip.split(",") if s.strip()]
    robot.ask("assist off")
    try:
        m = measure(robot, a.pwm, a.secs)
        avg, why = trusted(m, skip)
        ok = [i not in why for i in range(4)]
        print("Measured:")
        show(m, old, why)
        before = spread(avg, ok)
        print(f"  trusted wheels: fastest vs slowest differ by {100 * before:.0f} %")
        if a.measure:
            return
        if sum(ok) < 2:
            sys.exit("Fewer than 2 wheels can be trusted - nothing to balance. Check the encoders (bot.py watch).")
        target = sum(v for v, o in zip(avg, ok) if o) / sum(ok)
        new = list(old)
        for i in range(4):
            if not ok[i]:
                continue
            change = target / avg[i]
            if abs(change - 1) > MAX_CORRECTION:
                print(f"  {NAMES[i]}: would need {100 * (change - 1):+.0f} % - too much for motor spread, left alone"
                      " (check this wheel: encoder, wiring, something rubbing?)")
                ok[i] = False
                continue
            new[i] = round(max(0.6, min(1.4, old[i] * change)), 3)
        if new == old:
            print("Nothing to change.")
            return
        if not set_gains(robot, new):
            set_gains(robot, old)
            sys.exit("The robot didn't take the new gains - old ones kept.")
        print("Again with the new gains (not saved yet):")
        m2 = measure(robot, a.pwm, a.secs)
        avg2, _ = trusted(m2, skip)
        after = spread(avg2, ok)
        show(m2, new)
        if after < before:
            print(robot.ask("gain save"))
            print(f"Saved: trusted wheels now differ by {100 * after:.0f} % (was {100 * before:.0f} %). "
                  "Run it again to fine-tune.")
        else:
            set_gains(robot, old)
            print(f"Not better ({100 * before:.0f} % -> {100 * after:.0f} %) - old gains restored, nothing saved.")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
