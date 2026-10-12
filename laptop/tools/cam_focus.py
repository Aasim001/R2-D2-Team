"""Focus a robot's ESP32-CAM: live picture + a sharpness number. Turn the lens until the number is highest.
(The OV2640 has a fixed lens - no autofocus motor - so focusing means turning the lens by hand, once.)

  python tools/cam_focus.py --robot 2              # the camera IP comes from arena_calib_car2.json (cam_host)
  python tools/cam_focus.py --robot 2 --speak      # the PC SAYS "better" / "worse" / "best" while you turn
  python tools/cam_focus.py --host 192.168.43.35

Best done at the real working distance: robot parked on a docking square, cube in the alcove, camera looking at
it (or hold a cube where it sits when the robot is docked, ~15-20 cm from the lens). Keep the scene still while
you turn - the number also changes when the picture changes. A hand over the lens (dark picture) is ignored.
It also tries to read the QR code in the live picture all the time and says "QR code read" when it can.
Keys: S = full-resolution picture (1600x1200) and read the QR code, F = the same with the flash LED, Q = quit.
Needs the camera firmware firmware/esp32cam (live view on port 81).
"""
import argparse, json, os, subprocess, sys, threading, time, urllib.request

LAPTOP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, LAPTOP)
import arena_drive as ad                                  # (before cv2 - see CLAUDE.md)
import cv2
import numpy as np


class Stream(threading.Thread):
    """Reads the camera's MJPEG stream and keeps only the newest picture."""
    def __init__(self, url):
        super().__init__(daemon=True)
        self.url, self.frame, self.error = url, None, None

    def run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=5) as r:
                    self.error = None
                    while True:                               # each part: headers with Content-Length, then the JPEG
                        line = r.readline()
                        if not line:
                            break
                        if line.lower().startswith(b"content-length:"):
                            n = int(line.split(b":")[1])
                            r.readline()                      # (the empty line after the headers)
                            img = cv2.imdecode(np.frombuffer(r.read(n), np.uint8), cv2.IMREAD_COLOR)
                            if img is not None:
                                self.frame = img
            except Exception as e:                        # camera restarted / Wi-Fi: try again
                self.error = str(e)
                time.sleep(1)


def sharpness(img):
    """How sharp the middle half of the picture is (higher = sharper), or None if the lens is covered / dark or
    the middle shows nothing with detail (a plain surface).
    Edge strength (Sobel) after a light blur, on the same 320x240 scale at any picture size, divided by the
    brightness: sensor noise in a dark picture (a finger on the lens) no longer looks 'sharp'."""
    h, w = img.shape[:2]
    g = cv2.cvtColor(img[h // 4:3 * h // 4, w // 4:3 * w // 4], cv2.COLOR_BGR2GRAY)
    g = cv2.resize(g, (320, 240), interpolation=cv2.INTER_AREA).astype(np.float32)
    if g.mean() < 50 or g.std() < 4:
        return None
    g = cv2.GaussianBlur(g, (0, 0), 1.0)
    gx, gy = cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1)
    return float(100 * np.mean(gx * gx + gy * gy) / g.mean() ** 2)


class Voice:
    """Windows speech (one PowerShell process reading lines); silent if that isn't available."""
    def __init__(self):
        try:
            self.p = subprocess.Popen(
                ["powershell", "-NoProfile", "-Command",
                 "Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer;"
                 " $s.Rate = 2; while (($l = [Console]::In.ReadLine()) -ne $null) { $s.Speak($l) }"],
                stdin=subprocess.PIPE, text=True)
        except OSError:
            self.p = None

    def say(self, words):
        print("  (says)", words)
        if self.p and self.p.poll() is None:
            try:
                self.p.stdin.write(words + "\n")
                self.p.stdin.flush()
            except OSError:
                pass


class LiveQR(threading.Thread):
    """Keeps trying to read the QR code in the newest live picture."""
    def __init__(self, stream):
        super().__init__(daemon=True)
        self.stream, self.text = stream, None

    def run(self):
        while True:
            img = self.stream.frame
            if img is not None:
                text, _ = ad.decode_qr(img)
                if text:
                    self.text = text
            time.sleep(0.3)


def snapshot(host, flash):
    url = f"http://{host}/capture?size=uxga" + ("&flash=1" if flash else "")
    img = cv2.imdecode(np.frombuffer(urllib.request.urlopen(url, timeout=15).read(), np.uint8), cv2.IMREAD_COLOR)
    text, pts = ad.decode_qr(img)
    out = img.copy()
    if pts is not None:
        cv2.polylines(out, [np.int32(pts)], True, (0, 255, 0), 4)
    cv2.putText(out, f"QR: {text}" if text else "no QR code read", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.8,
                (0, 255, 0) if text else (0, 0, 255), 4)
    s = sharpness(img)
    cv2.putText(out, "covered, dark or no detail" if s is None else f"sharpness {s:.0f}", (20, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0), 3)
    name = os.path.join(LAPTOP, f"cam_focus_{host.replace('.', '_')}_{time.strftime('%H%M%S')}.jpg")
    cv2.imwrite(name, img)
    print(f"1600x1200 picture: QR {text!r}, sharpness {s} -> {name}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, choices=(1, 2, 3))
    ap.add_argument("--host", help="the camera's IP")
    ap.add_argument("--speak", action="store_true", help='say "better" / "worse" / "best" every few seconds')
    a = ap.parse_args()
    host = a.host
    if not host and a.robot:
        try:
            host = json.load(open(os.path.join(LAPTOP, f"arena_calib_car{a.robot}.json")))["cam_host"]
        except (OSError, KeyError, ValueError):
            sys.exit(f"No cam_host for robot {a.robot} - use --host")
    if not host:
        sys.exit("Which camera? --robot N or --host IP")
    stream = Stream(f"http://{host}:81/stream")
    stream.start()
    live = LiveQR(stream)
    live.start()
    voice = Voice() if a.speak else None
    best, hist, snap, now = 0.0, [], None, None
    said_t, said_val, said_qr = time.time(), None, None
    print(f"Camera {host}: turn the lens slowly; S = full-res picture + QR, F = with flash, Q = quit.")
    if voice:
        voice.say("Focus helper on. Turn the lens slowly.")
    while True:
        img = stream.frame
        if img is not None and img.shape[1] != 640:        # (a frame left over from a full-res picture)
            img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)
        if img is None:
            view = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(view, "waiting for " + host + (f": {stream.error}" if stream.error else ""), (10, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            s = sharpness(img)
            if s is None:                                  # hand over the lens: don't count it
                hist, now = [], None
            else:
                hist = (hist + [s])[-5:]                   # (a little smoothing: JPEG noise)
                now = sum(hist) / len(hist)
            view = img.copy()
            h, w = view.shape[:2]
            cv2.rectangle(view, (w // 4, h // 4), (3 * w // 4, 3 * h // 4), (255, 255, 0), 1)
            if now is None:
                cv2.putText(view, "covered, dark or no detail", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                shown_best = max(best, now)
                bar = int((w - 20) * min(1.0, now / max(shown_best, 1e-6)))
                cv2.rectangle(view, (10, h - 30), (10 + bar, h - 10),
                              (0, 255, 0) if now > 0.9 * shown_best else (0, 165, 255), -1)
                cv2.putText(view, f"sharpness {now:.0f}   best {shown_best:.0f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if live.text:
                cv2.putText(view, f"QR: {live.text}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            if voice and time.time() - said_t > 2.5:       # spoken guide, every 2.5 s
                said_t = time.time()
                if live.text and live.text != said_qr:
                    voice.say("QR code read: " + live.text)
                    said_qr = live.text
                elif now is None:
                    if said_val != "covered":
                        voice.say("covered")
                    said_val = "covered"
                else:
                    last = said_val if isinstance(said_val, float) else None
                    if now > best * 1.03:
                        voice.say("best")
                    elif last and now > last * 1.06:
                        voice.say("better")
                    elif last and now < last * 0.94:
                        voice.say("worse")
                    else:
                        voice.say("same")
                    said_val = now
            if now is not None and len(hist) == 5:
                best = max(best, now)
        try:
            cv2.imshow(f"ESP32-CAM focus {host}", view)
            if snap is not None:
                cv2.imshow("ESP32-CAM 1600x1200", cv2.resize(snap, (800, 600)))
        except cv2.error as e:                             # (e.g. the PC short of memory for a moment)
            print("display:", str(e).splitlines()[-1])
            time.sleep(0.5)
        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            break
        if k in (ord("s"), ord("f")):
            try:
                snap = snapshot(host, k == ord("f"))
            except Exception as e:
                print("picture failed:", e)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
