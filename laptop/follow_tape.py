#!/usr/bin/env python3
"""
Red-tape follower: iPhone camera -> MacBook (OpenCV) -> ESP32 over UDP

The camera can be EITHER
  - a camera number  (Continuity Camera / webcam):    --camera 1
  - a Wi-Fi stream URL from an iPhone camera app:     --camera http://192.168.1.23:8081/video
                                                      --camera rtsp://192.168.1.23:8554/live

  python3 follow_tape.py --find                 # show each camera number for 2 s
  python3 follow_tape.py --camera URL --preview # just show the camera (no robot needed)
  python3 follow_tape.py --camera http://PHONE_IP:8081 --user admin --password admin --preview
      (give just the phone's address: the script finds the video inside the app's web page)
  python3 follow_tape.py --camera URL           # run.  SPACE = start/stop, +/- = speed, Q = quit
  add --record to save the run (camera frames + every decision + robot wheel data) in laptop/runs/
"""
import os, re, cv2, numpy as np, socket, struct, threading, time, argparse
import urllib.request, urllib.parse, urllib.error

# ---------------- settings ----------------
CAMERA       = 1              # camera number, or a stream URL like "http://192.168.1.23:8081/video"
CAR_HOST     = "car1.local"   # or the IP the ESP32 prints
CAR_PORT     = 4210

PROC_W, PROC_H = 320, 180     # frames are shrunk to this - fast
ROI_TOP  = 0.35               # ignore the top 35% of the image (too far away)
BANDS    = 6                  # horizontal slices, near (bottom) to far (top)
MIN_PIX  = 30                 # red pixels a slice needs to count as "tape seen"

BASE_SPEED   = 170            # PWM on straights (0-255); +/- keys change it live
MIN_PWM      = 95             # N20s stall below this
KP           = 1.3            # steering strength
KD           = 0.02           # steering damping
SEARCH_SPEED = 150            # spin speed when the tape is lost
CAM_OFFSET   = 0.0            # tape x (-1..1) when the robot is centred on it

RED1 = ((0, 100, 60), (10, 255, 255))     # red wraps around hue 0,
RED2 = ((165, 100, 60), (180, 255, 255))  # so two ranges


# low-latency options for network streams opened by FFmpeg (RTSP): no buffering, TCP
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                      "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0")
COMMON_PATHS = ["/video", "/videofeed", "/mjpeg", "/live", "/stream", "/mjpg/video.mjpg",
                "/video.mjpg", "/cam.mjpg", "/live.mjpg", "/stream.mjpg"]
DEFAULT_LOGINS = [("admin", "admin")]          # tried automatically if the phone asks for a login


class LoginNeeded(Exception):
    pass


def split_login(url, user, password):
    """Take user:password out of a URL like http://admin:admin@192.168.1.5:8081/video"""
    p = urllib.parse.urlsplit(url)
    netloc = p.netloc
    if p.username:
        user = urllib.parse.unquote(p.username)
        password = urllib.parse.unquote(p.password or "")
        netloc = p.hostname + (f":{p.port}" if p.port else "")
    path = p.path or "/"                       # "http://ip:port" -> "http://ip:port/" (needed for logins)
    return urllib.parse.urlunsplit((p.scheme, netloc, path, p.query, p.fragment)), user, password


def make_opener(url, user, password):
    """urllib opener that answers both Basic and Digest login prompts (like a browser does)."""
    mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
    if user is not None:
        p = urllib.parse.urlsplit(url)
        mgr.add_password(None, f"{p.scheme}://{p.netloc}/", user, password)
    return urllib.request.build_opener(urllib.request.HTTPBasicAuthHandler(mgr),
                                       urllib.request.HTTPDigestAuthHandler(mgr))


class HttpMjpeg:
    """Reads an MJPEG stream (multipart/x-mixed-replace) frame by frame."""
    def __init__(self, resp):
        self.resp, self.buf = resp, b""
        m = re.search(r'boundary="?([^";]+)"?', resp.headers.get("Content-Type", ""), re.I)
        b = m.group(1).strip() if m else None
        self.boundary = (b if b and b.startswith("--") else f"--{b}").encode() if b else None

    def _fill(self):
        chunk = self.resp.read1(65536) if hasattr(self.resp, "read1") else self.resp.read(4096)
        if not chunk:
            raise EOFError
        self.buf += chunk
        if len(self.buf) > 8_000_000:                   # never let the buffer grow forever
            self.buf = self.buf[-2_000_000:]

    def _next_jpeg(self, block=True):
        """Next complete JPEG from the buffer. With block=False, return None instead of waiting."""
        while True:
            start = self.buf.find(b"\xff\xd8")          # JPEG start
            if start != -1:
                head = self.buf[:start].decode("latin-1").lower()
                m = re.findall(r"content-length:\s*(\d+)", head)
                if m:                                   # best case: the part says how long it is
                    n = int(m[-1])
                    if len(self.buf) >= start + n:
                        jpg, self.buf = self.buf[start:start + n], self.buf[start + n:]
                        return jpg
                elif self.boundary:                     # otherwise: up to the next boundary
                    end = self.buf.find(self.boundary, start)
                    if end != -1:
                        jpg, self.buf = self.buf[start:end], self.buf[end:]
                        return jpg.rstrip(b"\r\n")
                else:                                   # last resort: JPEG end marker
                    end = self.buf.find(b"\xff\xd9", start + 2)
                    if end != -1:
                        jpg, self.buf = self.buf[start:end + 2], self.buf[end + 2:]
                        return jpg
            if not block:
                return None
            self._fill()

    def read(self):
        try:
            while True:
                jpg = self._next_jpeg()
                newer = self._next_jpeg(block=False)
                while newer is not None:                # frames piled up? jump to the newest
                    jpg, newer = newer, self._next_jpeg(block=False)
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    return True, img
        except (EOFError, OSError, ValueError):
            return False, None

    def release(self):
        try:
            self.resp.close()
        except Exception:
            pass


class HttpSnapshot:
    """For apps that only offer a single JPEG picture URL: fetch it again and again."""
    def __init__(self, opener, url):
        self.opener, self.url = opener, url

    def read(self):
        try:
            data = self.opener.open(self.url, timeout=3).read()
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            return img is not None, img
        except Exception:
            return False, None

    def release(self):
        pass


def _try_http(url, opener, depth=0, seen=None):
    """Open url; if it is a web page, look inside it for the video like a browser would."""
    seen = seen if seen is not None else set()
    if url in seen:
        return None, None
    seen.add(url)
    try:
        resp = opener.open(url, timeout=4)
    except urllib.error.HTTPError as e:
        if e.code == 401:
            raise LoginNeeded()
        return None, None
    except Exception:
        return None, None
    ctype = resp.headers.get("Content-Type", "").lower()
    if "multipart" in ctype:
        return HttpMjpeg(resp), url
    if ctype.startswith("image/"):
        resp.close()
        return HttpSnapshot(opener, url), url
    if depth > 0:
        resp.close()
        return None, None
    page = resp.read(300_000).decode("utf-8", "ignore") if "html" in ctype or not ctype else ""
    resp.close()
    links = re.findall(r'(?:src|href|data|url)\s*[=:]\s*["\']([^"\']+)["\']', page, re.I)
    links += re.findall(r'["\']((?:rtsp|http)://[^"\']+)["\']', page)
    keep = [l for l in links if re.search(r"video|mjpe?g|stream|live|cam|shot|\.jpg|rtsp", l, re.I)
            and not re.search(r"\.(js|css|png|ico|svg|gif)(\?|$)", l, re.I)]
    base = url if url.endswith("/") else url + "/"
    for cand in keep + COMMON_PATHS:
        full = urllib.parse.urljoin(base, cand)
        if full.startswith("rtsp"):
            return full, full                          # handled by FFmpeg below
        cap, used = _try_http(full, opener, 1, seen)
        if cap is not None:
            return cap, used
    return None, None


def open_capture(src, user=None, password=None):
    """Open a camera number (AVFoundation on macOS) or a network stream (http MJPEG or rtsp)."""
    if isinstance(src, int):
        cap = cv2.VideoCapture(src, cv2.CAP_AVFOUNDATION)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        return cap if cap.isOpened() else None

    url, user, password = split_login(src, user, password)
    if url.startswith("http"):
        logins = [(user, password)] if user is not None else [(None, None)] + DEFAULT_LOGINS
        for u, pw in logins:
            try:
                cap, used = _try_http(url, make_opener(url, u, pw))
            except LoginNeeded:
                continue
            if isinstance(cap, str):                   # the web page pointed at an rtsp:// stream
                url = cap
                break
            if cap is not None:
                if used != url or (u and user is None):
                    print(f"Stream found at {used}" + (f"  (login {u})" if u else ""))
                return cap
        else:
            raise LoginNeeded()

    if user is not None and "@" not in url:            # rtsp with a login
        p = urllib.parse.urlsplit(url)
        url = f"{p.scheme}://{urllib.parse.quote(user)}:{urllib.parse.quote(password or '')}@{p.netloc}{p.path}"
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if cap.isOpened() and cap.read()[0]:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap
    cap.release()
    return None


class Camera:                               # always holds the newest frame (no lag build-up)
    def __init__(self, src, user=None, password=None):
        self.src, self.user, self.password = src, user, password
        self.frame, self.t, self.running, self.status = None, 0.0, True, "connecting"
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last_msg = ""
        while self.running:
            try:
                cap = open_capture(self.src, self.user, self.password)
                if cap is None:
                    self.status = f"cannot open {self.src} - retrying"
            except LoginNeeded:
                cap = None
                self.status = "the phone wants a login: add --user NAME --password PASS"
            if cap is None:
                if self.status != last_msg:
                    print(self.status)
                    last_msg = self.status
                time.sleep(1.0)
                continue
            self.status, fails, last_msg = "ok", 0, ""
            while self.running and fails < 30:              # ~1 s of failed reads -> reconnect
                ok, f = cap.read()
                if ok:
                    fails = 0
                    with self.lock:
                        self.frame, self.t = f, time.time()
                else:
                    fails += 1
                    time.sleep(0.03)
            cap.release()
            self.status = "camera dropped - reconnecting"

    def latest(self):
        with self.lock:
            return self.frame, self.t


def waiting_screen(text):
    img = np.zeros((360, 640, 3), np.uint8)
    for i, line in enumerate(["Waiting for camera...", text, "", "Q = quit"]):
        cv2.putText(img, line[:70], (15, 60 + 40 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
    return img


class Car:                                  # 8-byte UDP drive packet: 'DR', left, right, sequence
    def __init__(self, host, port):
        self.addr = (socket.gethostbyname(host), port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def send(self, left, right):
        self.seq = (self.seq + 1) & 0xFFFF
        self.sock.sendto(b"DR" + struct.pack("<hhH", int(left), int(right), self.seq), self.addr)


def detect(frame):
    small = cv2.resize(frame, (PROC_W, PROC_H), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, *RED1) | cv2.inRange(hsv, *RED2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    y0 = int(PROC_H * ROI_TOP)
    bh = (PROC_H - y0) // BANDS
    pts = []                                 # pts[0] = nearest slice
    for i in range(BANDS):
        y2 = PROC_H - i * bh
        y1 = y2 - bh
        xs = np.nonzero(mask[y1:y2])[1]
        if len(xs) >= MIN_PIX:
            pts.append(((xs.mean() - PROC_W / 2) / (PROC_W / 2), (y1 + y2) // 2))
        else:
            pts.append(None)
    return small, mask, pts


class Follower:
    def __init__(self):
        self.prev_err, self.prev_t, self.last_side, self.err = 0.0, time.time(), 1, None

    def update(self, pts, base):
        now = time.time()
        dt = max(now - self.prev_t, 1e-3)
        self.prev_t = now
        seen = [p for p in pts if p is not None]
        if not seen:                         # lost: spin toward where the tape was
            s = self.last_side
            self.err = None
            return s * SEARCH_SPEED, -s * SEARCH_SPEED, "SEARCH"
        err = 0.5 * seen[0][0] + 0.5 * seen[-1][0] - CAM_OFFSET   # near + look-ahead
        self.err = err
        derr = (err - self.prev_err) / dt
        self.prev_err = err
        if abs(err) > 0.05:
            self.last_side = 1 if err > 0 else -1
        turn = (KP * err + KD * derr) * base
        speed = base * (1 - 0.6 * min(1.0, abs(err)))             # slow down in bends
        return speed + turn, speed - turn, "FOLLOW"


def to_pwm(v):
    if abs(v) < 20:
        return 0
    return int(np.sign(v) * min(255, max(MIN_PWM, abs(v))))


def draw(small, mask, pts, state, base, cmd, fps, running):
    view = np.hstack([small, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
    cx = PROC_W // 2
    cv2.line(view, (cx, 0), (cx, PROC_H), (255, 255, 0), 1)
    for p in pts:
        if p:
            cv2.circle(view, (int(cx + p[0] * cx), p[1]), 4, (0, 255, 0), -1)
    col = (0, 255, 0) if running else (0, 0, 255)
    cv2.putText(view, f"{state}  base {base}  L {cmd[0]} R {cmd[1]}  {fps:.0f} fps",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    return cv2.resize(view, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)


class Recorder:
    """--record: saves what the robot saw and did, so a run can be reviewed afterwards.
    laptop/runs/<date-time>/  log.csv (every frame), robot.jsonl (wheel data from the ESP32),
                              frames/*.jpg (3 per second + every state change), summary.txt"""
    def __init__(self):
        here = os.path.dirname(os.path.abspath(__file__))
        self.dir = os.path.join(here, "runs", time.strftime("%Y%m%d-%H%M%S"))
        os.makedirs(os.path.join(self.dir, "frames"))
        self.csv = open(os.path.join(self.dir, "log.csv"), "w")
        self.csv.write("t,fps,state,base,err,seen,L,R," + ",".join(f"band{i}" for i in range(BANDS)) + "\n")
        self.robot = open(os.path.join(self.dir, "robot.jsonl"), "w")
        self.t0, self.last_img, self.last_state = time.time(), -1.0, None
        self.rows, self.errs, self.fps, self.lost_frames = 0, [], [], 0
        self.search_start, self.searches = None, []
        print("Recording to", self.dir)

    def frame(self, view, state, base, cmd, fps, pts, err, running):
        t = time.time() - self.t0
        seen = sum(p is not None for p in pts)
        bands = ",".join("" if p is None else f"{p[0]:.3f}" for p in pts)
        self.csv.write(f"{t:.3f},{fps:.1f},{state},{base},{'' if err is None else f'{err:.3f}'},"
                       f"{seen},{cmd[0]},{cmd[1]},{bands}\n")
        if running:
            self.rows += 1
            if self.rows > 15:                           # skip the warm-up frames
                self.fps.append(fps)
            if err is not None:
                self.errs.append(abs(err))
            if seen == 0:
                self.lost_frames += 1
            if state == "SEARCH" and self.search_start is None:
                self.search_start = t
            if state != "SEARCH" and self.search_start is not None:
                self.searches.append(t - self.search_start)
                self.search_start = None
        if state != self.last_state or t - self.last_img >= 0.33:
            cv2.imwrite(os.path.join(self.dir, "frames", f"{t:07.2f}_{state}.jpg"), view,
                        [cv2.IMWRITE_JPEG_QUALITY, 70])
            self.last_img, self.last_state = t, state

    def robot_data(self, text):
        if text.startswith("{"):
            self.robot.write(f'{{"t":{time.time() - self.t0:.3f},"robot":{text}}}\n')

    def close(self):
        if self.search_start is not None:
            self.searches.append(time.time() - self.t0 - self.search_start)
        avg = lambda v: sum(v) / len(v) if v else 0
        lines = [
            f"run folder:            {self.dir}",
            f"duration:              {time.time() - self.t0:.1f} s   (frames while running: {self.rows})",
            f"camera fps:            avg {avg(self.fps):.1f}, min {min(self.fps, default=0):.1f}",
            f"tape lost:             {100 * self.lost_frames / max(self.rows, 1):.1f} % of frames",
            f"searches (tape lost):  {len(self.searches)}, longest {max(self.searches, default=0):.2f} s",
            f"steering error |err|:  avg {avg(self.errs):.3f}, max {max(self.errs, default=0):.3f}  (0 = centred, 1 = edge)",
            f"settings:              BASE_SPEED {BASE_SPEED} KP {KP} KD {KD} ROI_TOP {ROI_TOP} SEARCH_SPEED {SEARCH_SPEED}",
        ]
        for f in (self.csv, self.robot):
            f.close()
        with open(os.path.join(self.dir, "summary.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print("\n".join(lines))


def find_cameras():
    print("Trying camera numbers 0-5. Each one that works is shown for 2 seconds.")
    for i in range(6):
        cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
        ok, f = cap.read() if cap.isOpened() else (False, None)
        for _ in range(10):                                  # some cameras need a few frames
            if ok:
                break
            ok, f = cap.read()
        if ok:
            print(f"  camera {i}: {f.shape[1]}x{f.shape[0]}")
            cv2.putText(f, f"camera {i}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
            cv2.imshow("find cameras", f)
            cv2.waitKey(2000)
        else:
            print(f"  camera {i}: nothing")
        cap.release()
    cv2.destroyAllWindows()


def preview(cam):
    """Show the camera with fps and frame age - no robot needed."""
    last_t, fps = 0.0, 0.0
    while True:
        frame, t = cam.latest()
        if frame is None:
            cv2.imshow("preview", waiting_screen(cam.status))
        elif t != last_t:
            if last_t:
                fps = 0.9 * fps + 0.1 / max(t - last_t, 1e-3)
            last_t = t
            view = cv2.resize(frame, (960, int(960 * frame.shape[0] / frame.shape[1])))
            cv2.putText(view, f"{frame.shape[1]}x{frame.shape[0]}  {fps:.0f} fps", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("preview", view)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cam.running = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--find", action="store_true", help="show each camera number")
    ap.add_argument("--camera", default=None, help="camera number or stream URL")
    ap.add_argument("--preview", action="store_true", help="only show the camera")
    ap.add_argument("--car", default=CAR_HOST, help="robot address (car1.local or its IP)")
    ap.add_argument("--user", default=None, help="camera app login name")
    ap.add_argument("--password", default=None, help="camera app password")
    ap.add_argument("--record", action="store_true", help="save frames + decisions + wheel data to laptop/runs/")
    a = ap.parse_args()
    if a.find:
        find_cameras()
        return
    src = a.camera if a.camera is not None else CAMERA
    if isinstance(src, str) and src.isdigit():
        src = int(src)

    cam = Camera(src, a.user, a.password)
    if a.preview:
        preview(cam)
        return
    try:
        car = Car(a.car, CAR_PORT)
    except socket.gaierror:
        print(f"Can't find the robot at '{a.car}'. Use its IP:  --car 10.67.197.57")
        cam.running = False
        return
    print("Sending to", car.addr, " SPACE = start/stop, +/- = speed, Q = quit")
    rec = Recorder() if a.record else None
    if rec:                                              # ask the robot to stream its wheel data back
        car.sock.setblocking(False)
        car.sock.sendto(b"tele on", car.addr)
    fol, running, base = Follower(), False, BASE_SPEED
    cmd, state, last_t, last_send, fps = (0, 0), "STOPPED", 0.0, 0.0, 0.0

    while True:
        frame, t = cam.latest()
        if frame is None:
            cv2.imshow("follow", waiting_screen(cam.status))
        new = frame is not None and t != last_t
        if new:
            if last_t:
                fps = 0.9 * fps + 0.1 / max(t - last_t, 1e-3)
            last_t = t
            small, mask, pts = detect(frame)
            if running:
                l, r, state = fol.update(pts, base)
                cmd = (to_pwm(l), to_pwm(r))
            else:
                cmd, state = (0, 0), "STOPPED"
            view = draw(small, mask, pts, state, base, cmd, fps, running)
            cv2.imshow("follow", view)
            if rec:
                rec.frame(view, state, base, cmd, fps, pts, fol.err if running else None, running)
        if frame is not None and time.time() - t > 0.3:      # camera froze -> stop
            cmd, state = (0, 0), "NO CAMERA"
        if new or time.time() - last_send > 0.05:            # keep-alive at 20 Hz minimum
            car.send(*cmd)
            last_send = time.time()
        while rec:                                           # wheel data coming back from the robot
            try:
                rec.robot_data(car.sock.recvfrom(4096)[0].decode("utf-8", "ignore"))
            except (BlockingIOError, OSError):
                break

        k = cv2.waitKey(1) & 0xFF
        if k == ord(' '):
            running = not running
            fol.prev_t = time.time()
        elif k in (ord('+'), ord('=')):
            base = min(255, base + 10)
        elif k == ord('-'):
            base = max(MIN_PWM, base - 10)
        elif k == ord('q'):
            break

    for _ in range(5):
        car.send(0, 0)
        time.sleep(0.02)
    if rec:
        car.sock.sendto(b"tele off", car.addr)
        rec.close()
    cam.running = False


if __name__ == "__main__":
    main()
