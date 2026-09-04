"""MaixCAM2 rotary-ball recognizer for the continuously rotating BALL task.

This program is intentionally independent from ``licang_BLUE_RED_BALL.py``.
The normal RZ/STAIR program keeps its 1/2 protocol and ROI behavior.  This
program accepts 3 for a red rotary target and 4 for a blue rotary target,
searches the complete camera frame, estimates the blob centre velocity, and
reports ``1\n`` when the predicted centre reaches the fixed grab area.
"""

import math

try:
    from maix import app, camera, display, gpio, image, pinmap, time
    from maix.peripheral import uart
    MAIXPY = True
except Exception:
    app = camera = display = gpio = image = pinmap = time = uart = None
    MAIXPY = False


# ========================== User Configuration ==========================

MODE_ROTARY_RED = 3
MODE_ROTARY_BLUE = 4
COLOR_NAMES = {
    MODE_ROTARY_RED: "RED",
    MODE_ROTARY_BLUE: "BLUE",
}

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

# MaixCAM2 UART2: TX=B0, RX=B1, device=/dev/ttyS2.
UART_DEVICE = "/dev/ttyS2"
UART_BAUDRATE = 115200

DEFAULT_GRAB_ROI = [220, 190, 260, 155]
GRAB_ROI = DEFAULT_GRAB_ROI[:]

# Rotary speed-compensation parameters.  Tune ARM_DELAY_MS on the real rig.
ARM_DELAY_MS = 50
SPEED_FILTER = 0.30
TRACK_LENGTH = 5
MIN_TRACK_FRAME = 3
MAX_ASSOCIATION_DISTANCE_PX = 100
MIN_TRACK_SPEED_PX_S = 20.0
TRACK_LOST_TIMEOUT_MS = 250
MAX_TRACKS = 4

# Keep the same simple colour/geometry gates used by the existing program.
RED_THRESHOLD = (0, 80, 40, 80, 10, 80)
BLUE_THRESHOLD = (10, 80, -20, 50, -100, -20)
COLOR_THRESHOLDS = {
    MODE_ROTARY_RED: RED_THRESHOLD,
    MODE_ROTARY_BLUE: BLUE_THRESHOLD,
}
PIXELS_THRESHOLD = 700
AREA_THRESHOLD = 900
MIN_AREA = 900
MIN_WIDTH = 20
MIN_HEIGHT = 20
MAX_WIDTH = 260
MAX_HEIGHT = 260
RATIO_MIN = 0.65
RATIO_MAX = 1.35

MAIN_LOOP_SLEEP_MS = 1
PERFORMANCE_REPORT_INTERVAL_MS = 1000

STATE_WAIT_CMD = 0
STATE_SEARCH_BALL = 1
STATE_TRACK_BALL = 2
STATE_TRIGGERED = 3


# ============================== Runtime State ==============================

current_mode = MODE_ROTARY_RED
recognition_armed = False
detected_latched = False
recognition_state = STATE_WAIT_CMD
illumination_gpio = None
last_status_message = ""
last_candidates = []
last_selected_blob = None
last_prediction = None
rotary_tracks = []
next_track_id = 1


# ============================== Time/Hardware ==============================

def ticks_ms():
    if MAIXPY and time is not None:
        return time.ticks_ms()
    import time as pytime
    return int(pytime.time() * 1000)


def ticks_diff(new_tick, old_tick):
    if MAIXPY and time is not None and hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_tick, old_tick)
    return int(new_tick - old_tick)


def sleep_ms(milliseconds):
    if MAIXPY and time is not None:
        time.sleep_ms(milliseconds)
    else:
        import time as pytime
        pytime.sleep(milliseconds / 1000.0)


class PerformanceStats:
    """Small in-process profiler for the rotary main loop.

    Stage values are accumulated for one-second reporting windows.  Track
    updates are printed immediately because they are useful for verifying
    whether velocity estimation is actually receiving consecutive blobs.
    """

    STAGES = ("camera", "detect", "track", "draw", "display")

    def __init__(self):
        self.stage_totals = {stage: 0.0 for stage in self.STAGES}
        self.frame_count = 0
        self.frame_time_total = 0.0
        self.raw_blob_count = 0
        self.valid_blob_count = 0
        self.track_count = 0
        self.track_update_count = 0
        self.ball_presence_frames = 0
        self.ball_presence_run = 0
        self.ball_presence_runs = []
        self.last_fps = 0.0
        self.min_fps = None
        self._window_start_ms = None
        self._frame_start_ms = None

    def start_frame(self, now_ms=None):
        self._frame_start_ms = ticks_ms() if now_ms is None else now_ms

    def record_stage(self, stage, duration_ms):
        if stage in self.stage_totals:
            self.stage_totals[stage] += max(0.0, float(duration_ms))

    def record_detection(self, raw_count, valid_count):
        self.raw_blob_count = int(raw_count)
        self.valid_blob_count = int(valid_count)
        if self.valid_blob_count > 0:
            self.ball_presence_frames += 1
            self.ball_presence_run += 1
        elif self.ball_presence_run > 0:
            self.ball_presence_runs.append(self.ball_presence_run)
            self.ball_presence_run = 0

    def record_track_update(self, track):
        self.track_update_count += 1
        print("ID:{} FRAME:{} X:{} Y:{} VX:{:.1f} VY:{:.1f}".format(
            track.id,
            track.frame_count,
            int(round(track.last_x)),
            int(round(track.last_y)),
            track.vx,
            track.vy,
        ))

    def finish_frame(self, now_ms=None, track_count=0):
        if self._frame_start_ms is None:
            self.start_frame(now_ms)
        end_ms = ticks_ms() if now_ms is None else now_ms
        frame_duration = ticks_diff(end_ms, self._frame_start_ms)
        if frame_duration < 0:
            frame_duration = 0
        self.frame_count += 1
        self.frame_time_total += frame_duration
        self.track_count = int(track_count)
        if self._window_start_ms is None:
            self._window_start_ms = end_ms
        elapsed_ms = ticks_diff(end_ms, self._window_start_ms)
        if elapsed_ms < PERFORMANCE_REPORT_INTERVAL_MS:
            return None

        fps = self.frame_count * 1000.0 / max(1, elapsed_ms)
        self.last_fps = fps
        if self.min_fps is None or fps < self.min_fps:
            self.min_fps = fps
        frame_ms = self.frame_time_total / max(1, self.frame_count)
        averages = {
            stage: self.stage_totals[stage] / max(1, self.frame_count)
            for stage in self.STAGES
        }
        print(("FPS:{:.1f} MIN_FPS:{:.1f} FRAME:{:.1f}ms "
               "DETECT:{:.1f}ms TRACK:{:.1f}ms DRAW:{:.1f}ms "
               "DISPLAY:{:.1f}ms RAW:{} VALID:{} TRACKS:{} "
               "BALL_FRAMES:{}").format(
                   fps,
                   self.min_fps,
                   frame_ms,
                   averages["detect"],
                   averages["track"],
                   averages["draw"],
                   averages["display"],
                   self.raw_blob_count,
                   self.valid_blob_count,
                   self.track_count,
                   self.ball_presence_frames,
               ))
        self.frame_count = 0
        self.frame_time_total = 0.0
        for stage in self.STAGES:
            self.stage_totals[stage] = 0.0
        self._window_start_ms = end_ms
        return fps


performance_stats = PerformanceStats()


def light_init():
    global illumination_gpio
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    pinmap.set_pin_function("B25", "B25")
    illumination_gpio = gpio.GPIO("B25", gpio.Mode.OUT)
    light_off()
    return illumination_gpio


def light_on():
    if illumination_gpio is None:
        raise RuntimeError("light_init() must be called first")
    illumination_gpio.value(1)


def light_off():
    if illumination_gpio is not None:
        illumination_gpio.value(0)


def init_uart():
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    pinmap.set_pin_function("B0", "UART2_TX")
    pinmap.set_pin_function("B1", "UART2_RX")
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def init_camera():
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    return camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT)


def init_display():
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    return display.Display()


# ================================ Geometry =================================

def _valid_rect(rect):
    if not isinstance(rect, (list, tuple)) or len(rect) != 4:
        return False
    try:
        values = [int(value) for value in rect]
    except (TypeError, ValueError):
        return False
    if any(isinstance(value, bool) for value in rect):
        return False
    if any(int(value) != value for value in rect):
        return False
    x, y, width, height = values
    return (0 <= x < CAMERA_WIDTH and 0 <= y < CAMERA_HEIGHT and
            0 < width <= CAMERA_WIDTH and 0 < height <= CAMERA_HEIGHT and
            x + width <= CAMERA_WIDTH and y + height <= CAMERA_HEIGHT)


def _clamp_rect(x, y, width, height):
    width = max(1, min(int(width), CAMERA_WIDTH))
    height = max(1, min(int(height), CAMERA_HEIGHT))
    x = max(0, min(int(x), CAMERA_WIDTH - width))
    y = max(0, min(int(y), CAMERA_HEIGHT - height))
    return [x, y, width, height]


def point_in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


def _blob_value(blob, name, fallback=0):
    value = getattr(blob, name, None)
    if value is None:
        return fallback
    try:
        value = value() if callable(value) else value
        return int(value)
    except (TypeError, ValueError):
        return fallback


def blob_center(blob):
    width = _blob_value(blob, "w")
    height = _blob_value(blob, "h")
    return (_blob_value(blob, "x") + width / 2.0,
            _blob_value(blob, "y") + height / 2.0)


# ============================== UART/Command ===============================

def reset_rotary_tracks():
    global rotary_tracks, next_track_id, last_candidates
    global last_prediction, last_selected_blob
    rotary_tracks = []
    next_track_id = 1
    last_candidates = []
    last_prediction = None
    last_selected_blob = None


def set_rotary_command(mode):
    global current_mode, recognition_armed, detected_latched, recognition_state
    global last_status_message
    if mode not in (MODE_ROTARY_RED, MODE_ROTARY_BLUE):
        return False
    current_mode = mode
    recognition_armed = True
    detected_latched = False
    recognition_state = STATE_SEARCH_BALL
    reset_rotary_tracks()
    last_status_message = "SEARCH BALL"
    return True


def process_command_bytes(data):
    """Accept rotary commands 3/4; normal 1/2 belongs to the old program."""
    if data is None:
        return
    if isinstance(data, str):
        data = data.encode("ascii", "ignore")
    try:
        values = bytes(data)
    except (TypeError, ValueError):
        return
    for value in values:
        if value == ord("3"):
            set_rotary_command(MODE_ROTARY_RED)
        elif value == ord("4"):
            set_rotary_command(MODE_ROTARY_BLUE)


def uart_process(serial):
    if serial is None:
        return
    try:
        data = serial.read()
    except Exception:
        return
    if data:
        process_command_bytes(data)


# ================================ Detection ================================

def filter_blob(blob):
    if blob is None:
        return None
    width = _blob_value(blob, "w")
    height = _blob_value(blob, "h")
    if width < MIN_WIDTH or height < MIN_HEIGHT:
        return None
    if width > MAX_WIDTH or height > MAX_HEIGHT:
        return None
    pixels = _blob_value(blob, "pixels", width * height)
    area = _blob_value(blob, "area", width * height)
    if pixels < PIXELS_THRESHOLD or area < MIN_AREA:
        return None
    ratio = width / float(height)
    if ratio < RATIO_MIN or ratio > RATIO_MAX:
        return None
    return blob


def select_largest_blob(blobs):
    candidates = []
    for blob in blobs or []:
        valid = filter_blob(blob)
        if valid is not None:
            candidates.append((_blob_value(valid, "area", 0),
                               _blob_value(valid, "pixels", 0), valid))
    if not candidates:
        return None
    return max(candidates, key=lambda item: (item[0], item[1]))[2]


def detect_rotary_candidates(img, mode=None, metrics=None):
    """Detect valid target blobs over the whole frame; no ROI is passed."""
    if metrics is None:
        metrics = performance_stats
    if img is None:
        metrics.record_detection(0, 0)
        return []
    selected_mode = current_mode if mode is None else mode
    threshold = COLOR_THRESHOLDS.get(selected_mode, RED_THRESHOLD)
    try:
        blobs = img.find_blobs(
            [threshold],
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
        )
    except Exception:
        metrics.record_detection(0, 0)
        return []
    raw_blobs = blobs or []
    candidates = [blob for blob in raw_blobs if filter_blob(blob) is not None]
    metrics.record_detection(len(raw_blobs), len(candidates))
    return candidates


# ================================= Tracking =================================

class RotaryTrack:
    def __init__(self, track_id, center_x, center_y, now_ms, metrics=None):
        self.id = track_id
        self.metrics = performance_stats if metrics is None else metrics
        self.history = []
        self.last_x = float(center_x)
        self.last_y = float(center_y)
        self.last_seen_ms = now_ms
        self.frame_count = 0
        self.vx = 0.0
        self.vy = 0.0
        self.triggered = False
        self.update(center_x, center_y, now_ms)

    def update(self, center_x, center_y, now_ms):
        self.last_x = float(center_x)
        self.last_y = float(center_y)
        self.last_seen_ms = now_ms
        self.frame_count += 1
        self.history.append((now_ms, self.last_x, self.last_y))
        if len(self.history) > TRACK_LENGTH:
            self.history.pop(0)
        if len(self.history) >= 2:
            first = self.history[0]
            last = self.history[-1]
            elapsed_ms = ticks_diff(last[0], first[0])
            if elapsed_ms > 0:
                elapsed_seconds = elapsed_ms / 1000.0
                measured_vx = (last[1] - first[1]) / elapsed_seconds
                measured_vy = (last[2] - first[2]) / elapsed_seconds
                self.vx = ((1.0 - SPEED_FILTER) * self.vx +
                           SPEED_FILTER * measured_vx)
                self.vy = ((1.0 - SPEED_FILTER) * self.vy +
                           SPEED_FILTER * measured_vy)
        if self.metrics is not None:
            self.metrics.record_track_update(self)

    def speed(self):
        return math.sqrt(self.vx * self.vx + self.vy * self.vy)


def _distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def update_rotary_tracks(candidates, now_ms=None, metrics=None):
    global rotary_tracks, next_track_id
    if now_ms is None:
        now_ms = ticks_ms()
    if metrics is None:
        metrics = performance_stats

    unmatched = list(candidates or [])
    matched_track_ids = set()
    # Greedy nearest-neighbour matching, with each blob used at most once.
    for track in rotary_tracks:
        if not unmatched:
            break
        best_index = None
        best_distance = MAX_ASSOCIATION_DISTANCE_PX + 1
        for index, blob in enumerate(unmatched):
            center_x, center_y = blob_center(blob)
            distance = _distance(track.last_x, track.last_y,
                                 center_x, center_y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        if best_index is not None and best_distance <= MAX_ASSOCIATION_DISTANCE_PX:
            blob = unmatched.pop(best_index)
            track.update(*blob_center(blob), now_ms)
            matched_track_ids.add(track.id)

    rotary_tracks = [
        track for track in rotary_tracks
        if ticks_diff(now_ms, track.last_seen_ms) <= TRACK_LOST_TIMEOUT_MS
    ]
    while unmatched and len(rotary_tracks) < MAX_TRACKS:
        blob = unmatched.pop(0)
        center_x, center_y = blob_center(blob)
        rotary_tracks.append(RotaryTrack(next_track_id, center_x, center_y,
                                         now_ms, metrics))
        next_track_id += 1
    return rotary_tracks


def point_distance_to_roi(x, y, roi):
    rx, ry, rw, rh = roi
    dx = max(rx - x, 0.0, x - (rx + rw))
    dy = max(ry - y, 0.0, y - (ry + rh))
    return math.sqrt(dx * dx + dy * dy)


def predict_track(track):
    """Predict the ball centre when the arm would finish its motion."""
    if track.frame_count < MIN_TRACK_FRAME or track.speed() < MIN_TRACK_SPEED_PX_S:
        return None
    future_x = track.last_x + track.vx * ARM_DELAY_MS / 1000.0
    future_y = track.last_y + track.vy * ARM_DELAY_MS / 1000.0
    current_distance = point_distance_to_roi(track.last_x, track.last_y,
                                             GRAB_ROI)
    future_distance = point_distance_to_roi(future_x, future_y, GRAB_ROI)
    approaching = future_distance < current_distance
    if not approaching or not point_in_rect(future_x, future_y, GRAB_ROI):
        return None
    return (future_x, future_y, future_distance)


def select_trigger_track():
    choices = []
    for track in rotary_tracks:
        prediction = predict_track(track)
        if prediction is not None and not track.triggered:
            choices.append((track, prediction))
    if not choices:
        return None, None
    # In this one-frame scheduling model all candidates share ARM_DELAY; use
    # the one nearest to its predicted entry point as the deterministic tie-break.
    return min(choices, key=lambda item: item[1][2])


def rotary_recognition_process(img, serial, now_ms=None, metrics=None):
    global recognition_armed, detected_latched, recognition_state
    global last_status_message, last_candidates, last_selected_blob
    global last_prediction
    if not recognition_armed or detected_latched:
        return last_selected_blob

    if metrics is None:
        metrics = performance_stats
    if now_ms is None:
        now_ms = ticks_ms()
    detect_start_ms = ticks_ms()
    last_candidates = detect_rotary_candidates(img, metrics=metrics)
    last_selected_blob = select_largest_blob(last_candidates)
    metrics.record_stage("detect", ticks_diff(ticks_ms(), detect_start_ms))

    track_start_ms = ticks_ms()
    update_rotary_tracks(last_candidates, now_ms, metrics)

    recognition_state = STATE_TRACK_BALL if rotary_tracks else STATE_SEARCH_BALL
    track, prediction = select_trigger_track()
    metrics.record_stage("track", ticks_diff(ticks_ms(), track_start_ms))
    last_prediction = prediction
    if track is None:
        last_status_message = "TRACK BALL" if rotary_tracks else "SEARCH BALL"
        return last_selected_blob
    if serial is None:
        return last_selected_blob
    try:
        serial.write(b"1\n")
    except Exception:
        last_status_message = "UART WRITE FAILED"
        return last_selected_blob

    track.triggered = True
    detected_latched = True
    recognition_armed = False
    recognition_state = STATE_TRIGGERED
    last_status_message = "TRIGGER"
    reset_rotary_tracks()
    return last_selected_blob


# ================================== UI ======================================

def _color(name, fallback):
    if image is None:
        return fallback
    return getattr(image, name, fallback)


def _draw_text(img, x, y, text, color):
    try:
        img.draw_string(x, y, text, color)
    except Exception:
        pass


def _draw_cross(img, x, y, color, size=6):
    try:
        img.draw_line(int(x - size), int(y), int(x + size), int(y), color)
        img.draw_line(int(x), int(y - size), int(x), int(y + size), color)
    except Exception:
        pass


def draw_ui(img):
    """Draw rotary diagnostics; failures here never stop recognition."""
    if img is None:
        return
    white = _color("COLOR_WHITE", 0xFFFFFF)
    yellow = _color("COLOR_YELLOW", 0xFFFF00)
    green = _color("COLOR_GREEN", 0x00FF00)
    red = _color("COLOR_RED", 0xFF0000)
    blue = _color("COLOR_BLUE", 0x0000FF)
    blob_color = red if current_mode == MODE_ROTARY_RED else blue
    try:
        img.draw_rect(GRAB_ROI[0], GRAB_ROI[1], GRAB_ROI[2], GRAB_ROI[3], yellow)
        _draw_cross(img, GRAB_ROI[0] + GRAB_ROI[2] / 2,
                    GRAB_ROI[1] + GRAB_ROI[3] / 2, yellow)

        for blob in last_candidates:
            x = _blob_value(blob, "x")
            y = _blob_value(blob, "y")
            width = _blob_value(blob, "w")
            height = _blob_value(blob, "h")
            img.draw_rect(x, y, width, height, blob_color)
            center_x, center_y = blob_center(blob)
            _draw_cross(img, center_x, center_y, green, 4)

        for track in rotary_tracks:
            for point in track.history:
                _draw_cross(img, point[1], point[2], green, 2)
            _draw_cross(img, track.last_x, track.last_y, green, 6)

        if last_prediction is not None:
            _draw_cross(img, last_prediction[0], last_prediction[1], yellow, 8)
            try:
                img.draw_line(int(last_prediction[0]), int(last_prediction[1]),
                              int(GRAB_ROI[0] + GRAB_ROI[2] / 2),
                              int(GRAB_ROI[1] + GRAB_ROI[3] / 2), yellow)
            except Exception:
                pass

        track = rotary_tracks[0] if rotary_tracks else None
        speed = track.speed() if track is not None else 0.0
        vx = track.vx if track is not None else 0.0
        vy = track.vy if track is not None else 0.0
        color_name = COLOR_NAMES.get(current_mode, "WAIT")
        if recognition_state == STATE_WAIT_CMD:
            status = "WAIT CMD"
        elif recognition_state == STATE_SEARCH_BALL:
            status = last_status_message or "SEARCH BALL"
        elif recognition_state == STATE_TRACK_BALL:
            status = last_status_message or "TRACK BALL"
        else:
            status = last_status_message or "TRIGGER"
        _draw_text(img, 10, 10, "MODE: ROTARY", white)
        _draw_text(img, 10, 35, "COLOR: {}".format(color_name), white)
        _draw_text(img, 10, 60, "STATUS: {}".format(status), white)
        _draw_text(img, 10, 85, "TRACKS: {}".format(len(rotary_tracks)), white)
        _draw_text(img, 10, 110, "VX: {:.1f}  VY: {:.1f}".format(vx, vy), white)
        _draw_text(img, 10, 135, "SPEED: {:.1f}px/s".format(speed), white)
        _draw_text(img, 10, 160, "ARM_DELAY: {}ms".format(ARM_DELAY_MS), white)
        _draw_text(img, 10, 185, "PREDICTED: {}".format(
            "YES" if last_prediction is not None else "NO"), white)
        _draw_text(img, 10, 210, "GRAB ROI: FIXED DEFAULT", yellow)
    except Exception:
        pass


# ================================ Main Loop =================================

def main():
    if not MAIXPY:
        raise RuntimeError("This script must run on MaixCAM2 with MaixPy")
    light_init()
    light_on()
    try:
        serial = init_uart()
        cam = init_camera()
        disp = init_display()
        print("MaixCAM2 rotary speed compensation started")
        print("UART2 {} TX=B0 RX=B1".format(UART_DEVICE))
        while not app.need_exit():
            performance_stats.start_frame()
            uart_process(serial)
            camera_start_ms = ticks_ms()
            frame = cam.read()
            performance_stats.record_stage(
                "camera", ticks_diff(ticks_ms(), camera_start_ms))
            if recognition_armed:
                rotary_recognition_process(frame, serial)
            draw_start_ms = ticks_ms()
            draw_ui(frame)
            performance_stats.record_stage(
                "draw", ticks_diff(ticks_ms(), draw_start_ms))
            display_start_ms = ticks_ms()
            disp.show(frame)
            performance_stats.record_stage(
                "display", ticks_diff(ticks_ms(), display_start_ms))
            sleep_ms(MAIN_LOOP_SLEEP_MS)
            performance_stats.finish_frame(ticks_ms(), len(rotary_tracks))
    finally:
        light_off()
        print("program exit")


# ================================= Self-test =================================

class _FakeBlob:
    def __init__(self, x, y, width=50, height=50, pixels=1800):
        self._x = x
        self._y = y
        self._width = width
        self._height = height
        self._pixels = pixels

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._width

    def h(self):
        return self._height

    def pixels(self):
        return self._pixels

    def area(self):
        return self._width * self._height


class _FakeImage:
    def __init__(self, blobs):
        self.blobs = blobs
        self.kwargs = None

    def find_blobs(self, thresholds, **kwargs):
        self.kwargs = kwargs
        return self.blobs


class _FakeSerial:
    def __init__(self):
        self.sent = []

    def write(self, data):
        self.sent.append(bytes(data))


def _selftest():
    global GRAB_ROI
    global current_mode, recognition_armed, detected_latched, recognition_state

    metrics = PerformanceStats()
    metrics.record_stage("camera", 3)
    metrics.record_stage("detect", 4)
    metrics.record_stage("track", 1)
    metrics.record_stage("draw", 2)
    metrics.record_stage("display", 5)
    metrics.record_detection(3, 1)
    assert metrics.stage_totals["camera"] == 3
    assert metrics.stage_totals["detect"] == 4
    assert metrics.raw_blob_count == 3
    assert metrics.valid_blob_count == 1

    assert DEFAULT_GRAB_ROI == [220, 190, 260, 155]
    assert GRAB_ROI == DEFAULT_GRAB_ROI
    assert _valid_rect(GRAB_ROI) is True
    assert filter_blob(_FakeBlob(0, 0, 100, 20)) is None

    process_command_bytes(b"3\r\n")
    assert current_mode == MODE_ROTARY_RED
    assert recognition_armed is True
    process_command_bytes(b"4")
    assert current_mode == MODE_ROTARY_BLUE
    assert recognition_armed is True

    original_roi = GRAB_ROI[:]
    original_mode = current_mode
    original_armed = recognition_armed
    original_latched = detected_latched
    original_state = recognition_state

    serial = _FakeSerial()
    process_command_bytes(b"3")
    # Full-frame detection must not receive an ROI keyword.
    for now_ms, x in ((0, 100), (100, 130), (200, 160)):
        rotary_recognition_process(_FakeImage([_FakeBlob(x, 200)]),
                                   serial, now_ms, metrics)
        assert serial.sent == []
    rotary_recognition_process(_FakeImage([_FakeBlob(190, 200)]),
                               serial, 300, metrics)
    assert serial.sent == [b"1\n"]
    assert recognition_armed is False
    assert detected_latched is True
    assert metrics.raw_blob_count == 1
    assert metrics.valid_blob_count == 1
    assert metrics.track_update_count >= 4

    # A new command starts a fresh transaction.  A ball moving away from the
    # fixed ROI must not satisfy the trigger.
    process_command_bytes(b"3")
    for now_ms, x in ((0, 300), (100, 270), (200, 240), (300, 210)):
        rotary_recognition_process(_FakeImage([_FakeBlob(x, 200)]),
                                   serial, now_ms)
    assert serial.sent == [b"1\n"]

    GRAB_ROI = original_roi
    current_mode = original_mode
    recognition_armed = original_armed
    detected_latched = original_latched
    recognition_state = original_state
    last_status_message = ""
    reset_rotary_tracks()
    print("rotary speed compensation selftest passed")


if __name__ == "__main__":
    import sys

    if "--selftest" in sys.argv:
        _selftest()
    else:
        main()
