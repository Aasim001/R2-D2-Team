#!/usr/bin/env python3
"""
Talk to the robot over UDP (same port as follow_tape.py, works with robot_main.ino).

  python3 bot.py status                 # one status reading
  python3 bot.py watch                  # live wheel speeds, PWM, trims, battery (Ctrl+C to stop)
  python3 bot.py drive 180 180 3        # left right seconds  -> straight-drive test
  python3 bot.py drive -150 -150 3      # backwards
  python3 bot.py arm reach              # stow | reach | carry | release
  python3 bot.py servo 90               # raw servo angle (calibration)
  python3 bot.py assist on|off          # straight-drive assist
  python3 bot.py trim reset
  python3 bot.py stop

  --host 10.67.197.57   if car1.local doesn't resolve
"""
import argparse, json, socket, struct, sys, time

PORT = 4210
NAMES = ["FL", "FR", "RL", "RR"]


def connect(host):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    return sock, (socket.gethostbyname(host), PORT)


def text(sock, addr, msg):
    sock.sendto(msg.encode(), addr)
    try:
        return sock.recvfrom(2048)[0].decode()
    except socket.timeout:
        return "(no reply - is the robot on and on the same Wi-Fi?)"


def show(d):
    print(f"battery {d['batt']:.2f} V{'  LOW - motors off' if d['low'] else ''}   "
          f"servo {d['servo']} deg   assist {'on' if d['assist'] else 'off'}   wifi {d['rssi']} dBm")
    print("wheel    count   rate/s   pwm  trim  encoder")
    for i, n in enumerate(NAMES):
        print(f"{n}   {d['cnt'][i]:8d} {d['rate'][i]:8d} {d['pwm'][i]:5d} {d['trim'][i]:5d}  "
              f"{'ok' if d['ok'][i] else 'NOT COUNTING'}")


def status(sock, addr):
    r = text(sock, addr, "status")
    try:
        return json.loads(r)
    except json.JSONDecodeError:
        print(r)
        sys.exit(1)


def drive(sock, addr, left, right, secs):
    start = status(sock, addr)
    seq = 0
    t_end = time.time() + secs
    print(f"driving L={left} R={right} for {secs} s ...")
    while time.time() < t_end:
        seq = (seq + 1) & 0xFFFF
        sock.sendto(b"DR" + struct.pack("<hhH", left, right, seq), addr)
        time.sleep(0.033)
    for _ in range(3):
        seq = (seq + 1) & 0xFFFF
        sock.sendto(b"DR" + struct.pack("<hhH", 0, 0, seq), addr)
        time.sleep(0.02)
    end = status(sock, addr)
    moved = [abs(end["cnt"][i] - start["cnt"][i]) for i in range(4)]
    print("counts moved:  " + "  ".join(f"{NAMES[i]} {moved[i]}" for i in range(4)))
    left_side, right_side = (moved[0] + moved[2]) / 2, (moved[1] + moved[3]) / 2
    if max(left_side, right_side) > 0:
        diff = 100 * (left_side - right_side) / max(left_side, right_side)
        print(f"left side {left_side:.0f}, right side {right_side:.0f}  ->  difference {diff:+.1f} %"
              "   (within about 2 % = straight)")
    show(end)


def watch(sock, addr):
    print(text(sock, addr, "tele on"))
    try:
        while True:
            try:
                d = json.loads(sock.recvfrom(2048)[0].decode())
            except (socket.timeout, json.JSONDecodeError):
                continue
            print("\033[2J\033[H", end="")          # clear screen
            show(d)
    except KeyboardInterrupt:
        print(text(sock, addr, "tele off"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="car1.local")
    ap.add_argument("cmd", nargs="+")
    a = ap.parse_args()
    sock, addr = connect(a.host)
    c = a.cmd
    if c[0] == "status":
        show(status(sock, addr))
    elif c[0] == "watch":
        watch(sock, addr)
    elif c[0] == "drive" and len(c) == 4:
        drive(sock, addr, int(c[1]), int(c[2]), float(c[3]))
    else:
        print(text(sock, addr, " ".join(c)))


if __name__ == "__main__":
    main()
