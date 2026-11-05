"""Pick-and-drop test with the robot STANDING STILL (the wheels never move): scan the cube's QR code, pick the cube
with the claw, hold it, drop it - with a picture at every step, so you can see exactly what happened.

  python tools/pick_test.py --robot 1                # camera = the robot's ESP32-CAM (cam_host in arena_calib_car1.json)
  python tools/pick_test.py --robot 1 --camera 0     # a webcam on this PC instead (0 = usually the laptop's own)
  python tools/pick_test.py --robot 1 --hold 5       # hold the cube 5 s before dropping it
  python tools/pick_test.py --robot 1 --skip-qr      # pick even if the QR code can't be read

Set the claw ONCE on the robot's web page (http://<robot IP>/, section "Pick & drop"):
  Open the claw fully -> Save open;  put the cube in, Close until the fingers just touch it -> Save touch.
PICK then closes to the touch point + the grip strength (the "squeeze", gentle: SG90 plastic gears), DROP opens again.

Steps: 1 open  2 scan QR  3 pick  4 hold  5 drop. Ctrl+C stops the claw where it is.
Pictures: laptop/runs/pick-<time>-carN/ (one per step + overview.jpg, also shown in a window).
"""
import argparse, json, os, socket, sys, time, urllib.request

LAPTOP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, LAPTOP)
import arena_drive as ad                                  # (before cv2 - see CLAUDE.md)
import cv2
import numpy as np

PORT = 4210


class Robot:
    def __init__(self, ip):
        self.addr = (ip, PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)

    def _drain(self):
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recvfrom(4096)
        except OSError:
            pass
        self.sock.settimeout(0.5)

    def ask(self, text):
        """Send a text command, return the reply (None if no answer)."""
        for _ in range(3):
            self._drain()
            self.sock.sendto(text.encode(), self.addr)
            try:
                while True:
                    r = self.sock.recvfrom(4096)[0].decode(errors="replace")
                    if text == "status" and not r.startswith("{"):
                        continue
                    return r
            except (socket.timeout, ConnectionResetError):
                continue
        return None

    def status(self):
        r = self.ask("status")
        try:
            return json.loads(r) if r else None
        except ValueError:
            return None

    def claw(self, what):
        r = self.ask("claw " + what)
        if r is None:
            sys.exit(f"The robot didn't answer 'claw {what}'.")
        if r.startswith("err"):
            sys.exit(f"The robot says: {r[4:]}")
        return r

    def wait_servo(self, secs=6):
        """Wait until the servo is at its target; returns (angle, reached)."""
        t0, st = time.time(), None
        while time.time() - t0 < secs:
            st = self.status()
            if st and st["servo"] == st.get("target"):
                return st["servo"], True
            time.sleep(0.1)
        return (st["servo"] if st else None), False


class Camera:
    """The robot's ESP32-CAM (full-resolution pictures, optional flash) or a webcam on this PC."""
    def __init__(self, cam_host=None, index=None):
        self.host, self.cap = cam_host, None
        if index is not None:
            self.cap = ad.open_camera(index)
            if self.cap is None:
                sys.exit(f"Can't open camera {index} on this PC.")

    def grab(self, flash=False):
        if self.cap is not None:
            frame = None
            for _ in range(5):                            # (skip old frames in the buffer)
                ok, f = self.cap.read()
                if ok:
                    frame = f
            return frame
        url = f"http://{self.host}/capture?size=uxga" + ("&flash=1" if flash else "")
        try:
            b = urllib.request.urlopen(url, timeout=15).read()
            return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
        except Exception as e:
            print("   (camera:", e, ")")
            return None

    def close(self):
        if self.cap is not None:
            self.cap.release()


def label(img, lines, ok=True):
    """Picture with a caption strip on top."""
    if img is None:
        img = np.zeros((600, 800, 3), np.uint8)
    out = cv2.resize(img, (800, int(800 * img.shape[0] / img.shape[1])), interpolation=cv2.INTER_AREA)
    bar = np.zeros((34 * len(lines) + 12, out.shape[1], 3), np.uint8)
    for i, t in enumerate(lines):
        cv2.putText(bar, t, (10, 30 + 34 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (80, 255, 80) if ok or i else (80, 80, 255), 2)
    return np.vstack([bar, out])


def seen_qr(cam, img):
    """Is the QR code readable in this picture (or, if not, in one with the flash)?"""
    text = ad.decode_qr(img)[0] if img is not None else None
    if not text and cam.host:
        flash = cam.grab(flash=True)
        text = ad.decode_qr(flash)[0] if flash is not None else None
    return text


def main():
    ap = argparse.ArgumentParser(description="Pick-and-drop test with the robot standing still.")
    ap.add_argument("--robot", type=int, choices=(1, 2, 3), required=True)
    ap.add_argument("--camera", type=int, default=None, help="use webcam N on this PC instead of the robot's ESP32-CAM")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds to hold the cube before dropping it")
    ap.add_argument("--qr-secs", type=float, default=12.0, help="how long to try reading the QR code")
    ap.add_argument("--skip-qr", action="store_true", help="pick even if the QR code was not read")
    ap.add_argument("--show-secs", type=float, default=60.0, help="how long to show the overview window")
    a = ap.parse_args()

    try:
        calib = json.load(open(os.path.join(LAPTOP, f"arena_calib_car{a.robot}.json")))
    except (OSError, ValueError):
        sys.exit(f"No arena_calib_car{a.robot}.json (robot IP).")
    robot = Robot(calib["host"])
    st = robot.status()
    if not st:
        sys.exit(f"car{a.robot} ({calib['host']}) doesn't answer - is it on, and this PC on the same Wi-Fi?")
    if "touch" not in st:
        sys.exit("This robot's firmware has no PICK / DROP yet - flash the new firmware first.")
    if st["open"] < 0 or st["touch"] < 0 or st["open"] == st["touch"]:
        sys.exit(f"Set the claw first on http://{calib['host']}/ (section 'Pick & drop'):\n"
                 "  1. hold Open until the claw is fully open -> press Save open\n"
                 "  2. put the cube between the fingers, hold Close until they just touch it -> press Save touch\n"
                 f"  (now: open {st['open']}, touch {st['touch']})")
    robot.ask("stop")                                    # wheels off (they are never driven by this test)
    if a.camera is not None:
        cam, cam_name = Camera(index=a.camera), f"PC webcam {a.camera}"
    else:
        host = calib.get("cam_host")
        if not host:
            sys.exit("No cam_host in the calib file - use --camera N for a webcam on this PC.")
        cam, cam_name = Camera(cam_host=host), f"ESP32-CAM {host}"
        try:
            page = urllib.request.urlopen(f"http://{host}/", timeout=5).read().decode(errors="replace")
        except Exception:
            sys.exit(f"The robot's camera {host} doesn't answer (is it powered?). "
                     "Or use a webcam on this PC: --camera 0")
        if "ready" not in page:
            sys.exit(f"The robot's camera {host} answers but its camera chip did not start (ribbon cable?).")
    run = os.path.join(LAPTOP, "runs", time.strftime("pick-%Y%m%d-%H%M%S") + f"-car{a.robot}")
    os.makedirs(run, exist_ok=True)
    print(f"car{a.robot}: claw open {st['open']} deg, touch {st['touch']} deg, squeeze {st['squeeze']} deg "
          f"-> picks at {min(170, max(10, st['touch'] + (1 if st['touch'] > st['open'] else -1) * st['squeeze']))} deg. "
          f"Camera: {cam_name}.")
    shots, results = [], {}

    def shot(name, img, lines, ok=True):
        if img is not None:
            cv2.imwrite(os.path.join(run, f"{len(shots) + 1}_{name}.jpg"), img)
        shots.append(label(img, lines, ok))

    try:
        # 1 open
        print("1. OPEN the claw ...")
        robot.claw("drop")
        ang, ok = robot.wait_servo()
        print(f"   claw at {ang} deg" + ("" if ok else " (not there yet!)"))
        shot("open", cam.grab(), [f"1 OPEN: claw {ang} deg", cam_name], ok)

        # 2 scan QR
        print(f"2. SCAN the QR code (up to {a.qr_secs:.0f} s) ...")
        text, img, pts, t0, n = None, None, None, time.time(), 0
        while time.time() - t0 < a.qr_secs and not text:
            img = cam.grab(flash=(n % 2 == 1))           # (every 2nd picture with the flash LED)
            n += 1
            if img is not None:
                text, pts = ad.decode_qr(img)
        results["qr"] = text
        if text:
            print(f"   QR code: {text!r}  ({n} pictures)")
            mark = img.copy()
            if pts is not None:
                cv2.polylines(mark, [np.int32(pts)], True, (0, 255, 0), max(2, img.shape[1] // 300))
            shot("qr", mark, [f"2 QR READ: {text}", f"after {n} pictures"])
        else:
            print(f"   QR code NOT read ({n} pictures).")
            shot("qr", img, ["2 QR NOT READ", f"{n} pictures"], False)
            if not a.skip_qr:
                print("   Stopped before picking (use --skip-qr to pick anyway). Check the camera sees the code, sharp.")
                return

        # 3 pick
        print("3. PICK: close onto the cube ...")
        robot.claw("pick")
        ang, ok = robot.wait_servo()
        st = robot.status()
        print(f"   claw at {ang} deg (target {st['target'] if st else '?'})" + ("" if ok else " - NOT reached"))
        img = cam.grab()
        still = seen_qr(cam, img)
        results["picked_angle"] = ang
        shot("picked", img, [f"3 PICKED: claw {ang} deg (touch {st['touch'] if st else '?'} + squeeze "
                             f"{st['squeeze'] if st else '?'})", "QR still seen: " + ("yes" if still else "no")], ok)

        # 4 hold
        print(f"4. HOLD for {a.hold:.0f} s ...")
        time.sleep(max(0.0, a.hold - 1.5))
        st = robot.status()
        shot("holding", cam.grab(), [f"4 HOLDING {a.hold:.0f} s: claw {st['servo'] if st else '?'} deg"])

        # 5 drop
        print("5. DROP: open the claw ...")
        robot.claw("drop")
        ang, ok = robot.wait_servo()
        img = cam.grab()
        after = seen_qr(cam, img)
        print(f"   claw at {ang} deg" + ("" if ok else " (not there yet!)"))
        results["dropped_angle"] = ang
        shot("dropped", img, [f"5 DROPPED: claw {ang} deg", "QR seen after the drop: " + ("yes" if after else "no")], ok)
    except KeyboardInterrupt:
        robot.ask("stop")                                # (freezes every servo move, also PICK / DROP)
        print("\nStopped (the claw stays where it is).")
    finally:
        cam.close()
        if shots:
            w = max(s.shape[1] for s in shots)
            shots_p = [cv2.copyMakeBorder(s, 0, 0, 0, w - s.shape[1], cv2.BORDER_CONSTANT) for s in shots]
            if len(shots_p) % 2:
                shots_p.append(np.zeros_like(shots_p[0]))
            rows = []
            for i in range(0, len(shots_p), 2):
                h = max(shots_p[i].shape[0], shots_p[i + 1].shape[0])
                rows.append(np.hstack([cv2.copyMakeBorder(s, 0, h - s.shape[0], 0, 0, cv2.BORDER_CONSTANT)
                                       for s in shots_p[i:i + 2]]))
            w2 = max(r.shape[1] for r in rows)
            over = np.vstack([cv2.copyMakeBorder(r, 0, 0, 0, w2 - r.shape[1], cv2.BORDER_CONSTANT) for r in rows])
            path = os.path.join(run, "overview.jpg")
            cv2.imwrite(path, over)
            print(f"Pictures: {run}  (overview: {path})")
            if results.get("qr"):
                print(f"CUBE QR CODE: {results['qr']}")
            if a.show_secs > 0:
                k = min(1.0, 1500 / over.shape[1], 900 / over.shape[0])
                view = cv2.resize(over, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
                cv2.imshow("pick test - any key to close", view)
                cv2.waitKey(int(a.show_secs * 1000))
                cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
