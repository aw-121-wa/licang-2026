"""MaixCAM2 red/blue ball ROI recognizer.

The script intentionally uses a small ASCII protocol:

* STM32 -> MaixCAM2: ``1`` selects red, ``2`` selects blue.
* MaixCAM2 -> STM32: ``1\\n`` reports one valid target.

Thresholds and geometry values near the top of this file are field-calibration
parameters and should be tuned with the B25 illumination LED enabled.
"""

import json

try:
    from maix import app, camera, display, gpio, image, pinmap, time, touchscreen
    from maix.peripheral import uart
    MAIXPY = True
except Exception:
    app = camera = display = gpio = image = pinmap = time = touchscreen = uart = None
    MAIXPY = False


# ========================== User Config ==========================

MODE_RED = 1
MODE_BLUE = 2
COLOR_NAMES = {
    MODE_RED: "RED",
    MODE_BLUE: "BLUE",
}

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

# MaixCAM2 official UART4: TX=A21, RX=A22, device=/dev/ttyS4.
UART_DEVICE = "/dev/ttyS2"
UART_BAUDRATE = 115200
UART_READ_CHUNK = 64

# Fixed grab window defaults. AUTO ROI may replace these at runtime and save
# the result to ROI_CONFIG_PATH.
ROI_CONFIG_PATH = "/root/maixcam_ball_roi.json"
DEFAULT_ROI = [240, 150, 160, 140]
DEFAULT_TRIGGER_ZONE = [295, 180, 50, 90]
ROI = DEFAULT_ROI[:]
TRIGGER_ZONE = DEFAULT_TRIGGER_ZONE[:]

CALIB_SEARCH_ROI = [0, 0, CAMERA_WIDTH, 380]
# Calibration search center only; the final ROI uses the measured ball center.
GRAB_CENTER_X = CAMERA_WIDTH // 2
GRAB_CENTER_Y = 220
CALIBRATED_CENTER_X = GRAB_CENTER_X
BALL_WIDTH = 50
BALL_HEIGHT = 50
CALIB_X_TOLERANCE = 40
CALIB_DURATION_MS = 5000
CALIB_MIN_SAMPLES = 5
ROI_WIDTH_SCALE = 3.0
ROI_HEIGHT_SCALE = 3.0
TRIGGER_WIDTH_SCALE = 1.2
TRIGGER_HEIGHT_SCALE = 1.8
ROI_MIN_WIDTH = 120
ROI_MIN_HEIGHT = 110
TRIGGER_MIN_WIDTH = 50
TRIGGER_MIN_HEIGHT = 70

# TODO: Re-calibrate with the B25 illumination LED enabled in the real venue.
RED_THRESHOLD = (0, 80, 40, 80, 10, 80)
# TODO: Re-calibrate with the B25 illumination LED enabled in the real venue.
BLUE_THRESHOLD = (10, 80, -20, 50, -100, -20)
COLOR_THRESHOLDS = {
    MODE_RED: RED_THRESHOLD,
    MODE_BLUE: BLUE_THRESHOLD,
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

BUTTON_RED = [10, 405, 200, 60]
BUTTON_BLUE = [220, 405, 200, 60]
BUTTON_AUTO_ROI = [430, 405, 200, 60]
TOUCH_DEBOUNCE_MS = 250
MAIN_LOOP_SLEEP_MS = 1


# ========================== Runtime State ==========================

current_mode = MODE_RED
display_mode = MODE_RED
detected_latched = False
recognition_armed = False
detected_streak = 0
last_touch_ms = 0
illumination_gpio = None
calibrating = False
calibration_started_ms = 0
calibration_samples = []
calibration_old_roi = None
calibration_old_trigger_zone = None
last_status_message = ""


# ========================== Time and Hardware ==========================

def ticks_ms():
    if MAIXPY and time is not None:
        return time.ticks_ms()
    import time as pytime
    return int(pytime.time() * 1000)


def sleep_ms(milliseconds):
    if MAIXPY and time is not None:
        time.sleep_ms(milliseconds)
    else:
        import time as pytime
        pytime.sleep(milliseconds / 1000.0)


def light_init():
    """Map MaixCAM2 B25 to GPIO output and leave it initially off."""
    global illumination_gpio
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    pinmap.set_pin_function("B25", "B25")
    illumination_gpio = gpio.GPIO("B25", gpio.Mode.OUT)
    light_off()
    return illumination_gpio


def light_on():
    """Turn on the MaixCAM2 onboard illumination LED."""
    if illumination_gpio is None:
        raise RuntimeError("light_init() must be called first")
    illumination_gpio.value(1)


def light_off():
    """Turn off the MaixCAM2 onboard illumination LED."""
    if illumination_gpio is None:
        return
    illumination_gpio.value(0)

def init_uart():
    """Initialize MaixCAM2 UART2 on B0/B1."""
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


def init_touchscreen():
    if not MAIXPY or touchscreen is None:
        return None
    return touchscreen.TouchScreen()


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


def median_int(values):
    if not values:
        raise ValueError("median requires at least one value")
    ordered = sorted(int(value) for value in values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return int((ordered[middle - 1] + ordered[middle]) / 2)


def build_calibrated_regions(center_x, center_y, ball_width, ball_height):
    """Build regions centered on the measured ball position."""
    roi_width = max(ROI_MIN_WIDTH, int(round(ball_width * ROI_WIDTH_SCALE)))
    roi_height = max(ROI_MIN_HEIGHT, int(round(ball_height * ROI_HEIGHT_SCALE)))
    trigger_width = max(TRIGGER_MIN_WIDTH,
                        int(round(ball_width * TRIGGER_WIDTH_SCALE)))
    trigger_height = max(TRIGGER_MIN_HEIGHT,
                         int(round(ball_height * TRIGGER_HEIGHT_SCALE)))
    roi = _clamp_rect(center_x - roi_width // 2,
                      center_y - roi_height // 2,
                      roi_width, roi_height)
    trigger_zone = _clamp_rect(center_x - trigger_width // 2,
                               center_y - trigger_height // 2,
                               trigger_width, trigger_height)
    return roi, trigger_zone


def load_roi_config(path=None):
    """Load persisted regions without replacing valid runtime values on error."""
    global ROI, TRIGGER_ZONE, GRAB_CENTER_Y, CALIBRATED_CENTER_X
    global BALL_WIDTH, BALL_HEIGHT
    if path is None:
        path = ROI_CONFIG_PATH
    try:
        with open(path, "r") as config_file:
            payload = json.load(config_file)
        loaded_roi = payload.get("roi")
        loaded_trigger = payload.get("trigger_zone")
        loaded_center_x = int(payload.get(
            "calibrated_center_x", payload.get("grab_center_x", CALIBRATED_CENTER_X)))
        loaded_center_y = int(payload.get("grab_center_y", GRAB_CENTER_Y))
        loaded_width = int(payload.get("ball_width", BALL_WIDTH))
        loaded_height = int(payload.get("ball_height", BALL_HEIGHT))
        if not _valid_rect(loaded_roi) or not _valid_rect(loaded_trigger):
            return False
        if not 0 <= loaded_center_x < CAMERA_WIDTH:
            return False
        if not 0 <= loaded_center_y < CAMERA_HEIGHT:
            return False
        if loaded_width <= 0 or loaded_height <= 0:
            return False
    except (OSError, TypeError, ValueError, KeyError, AttributeError,
            json.JSONDecodeError):
        return False

    ROI = [int(value) for value in loaded_roi]
    TRIGGER_ZONE = [int(value) for value in loaded_trigger]
    CALIBRATED_CENTER_X = loaded_center_x
    GRAB_CENTER_Y = loaded_center_y
    BALL_WIDTH = loaded_width
    BALL_HEIGHT = loaded_height
    return True


def save_roi_config(path=None):
    """Persist the current calibrated regions and their source measurements."""
    if path is None:
        path = ROI_CONFIG_PATH
    payload = {
        "version": 1,
        "roi": ROI[:],
        "trigger_zone": TRIGGER_ZONE[:],
        "grab_center_x": CALIBRATED_CENTER_X,
        "calibrated_center_x": CALIBRATED_CENTER_X,
        "grab_center_y": GRAB_CENTER_Y,
        "ball_width": BALL_WIDTH,
        "ball_height": BALL_HEIGHT,
    }
    try:
        with open(path, "w") as config_file:
            json.dump(payload, config_file)
    except (OSError, TypeError, ValueError):
        return False
    return True


# ========================== Mode and UART Protocol ==========================

def set_mode(mode, source):
    """Set task mode from UART or debug display mode from touch."""
    global current_mode, display_mode, detected_latched
    global recognition_armed, detected_streak, calibrating
    global calibration_samples, last_status_message
    if mode not in (MODE_RED, MODE_BLUE):
        return False
    if source == "uart":
        current_mode = mode
        display_mode = mode
        detected_latched = False
        recognition_armed = True
        detected_streak = 0
        calibrating = False
        calibration_samples = []
        last_status_message = ""
        return True
    if source == "touch":
        display_mode = mode
        return True
    current_mode = mode
    display_mode = mode
    detected_streak = 0
    return True


def process_command_bytes(data):
    """Process any ASCII 1/2 bytes without waiting for a complete line."""
    if data is None:
        return
    if isinstance(data, str):
        data = data.encode("ascii", "ignore")
    try:
        values = bytes(data)
    except (TypeError, ValueError):
        return
    for value in values:
        if value == ord("1"):
            set_mode(MODE_RED, "uart")
        elif value == ord("2"):
            set_mode(MODE_BLUE, "uart")


def uart_process(serial):
    """Poll UART once; MaixPy read() without arguments returns immediately."""
    if serial is None:
        return
    try:
        data = serial.read()
    except Exception:
        return
    if data:
        process_command_bytes(data)


# ========================== Blob Detection ==========================

def _blob_value(blob, name, fallback=0):
    value = getattr(blob, name, None)
    if value is None:
        return fallback
    try:
        value = value() if callable(value) else value
        return int(value)
    except (TypeError, ValueError):
        return fallback


def filter_blob(blob):
    """Return blob if it has ball-like geometry, otherwise return None."""
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


def is_blob_in_trigger_zone(blob):
    if blob is None:
        return False
    x = _blob_value(blob, "x")
    y = _blob_value(blob, "y")
    width = _blob_value(blob, "w")
    height = _blob_value(blob, "h")
    center_x = x + width // 2
    center_y = y + height // 2
    return point_in_rect(center_x, center_y, TRIGGER_ZONE)


def select_best_blob(blobs):
    candidates = []
    for blob in blobs or []:
        valid = filter_blob(blob)
        if valid is not None:
            area = _blob_value(valid, "area", 0)
            pixels = _blob_value(valid, "pixels", 0)
            candidates.append((area, pixels, valid))
    if not candidates:
        return None
    return max(candidates, key=lambda item: (item[0], item[1]))[2]


def detect_ball(img, mode=None):
    """Find the largest ball-like blob only inside ROI."""
    if img is None:
        return None
    selected_mode = current_mode if mode is None else mode
    threshold = COLOR_THRESHOLDS[selected_mode]
    try:
        blobs = img.find_blobs(
            [threshold],
            roi=ROI,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
        )
    except Exception:
        return None
    return select_best_blob(blobs)


def update_detection(blob, serial):
    """Report one valid ROI target for the currently armed UART task."""
    global detected_latched, recognition_armed, detected_streak
    global last_status_message, calibrating
    if calibrating or not recognition_armed or detected_latched:
        return
    valid_blob = filter_blob(blob)
    if valid_blob is None:
        detected_streak = 0
        return

    if serial is None:
        return
    try:
        serial.write(b"1\n")
    except Exception:
        return
    detected_latched = True
    recognition_armed = False
    detected_streak = 0
    last_status_message = "DONE"


def start_auto_roi():
    """Start a five-second trajectory calibration without reporting UART."""
    global calibrating, calibration_started_ms, calibration_samples
    global calibration_old_roi
    global calibration_old_trigger_zone, last_status_message
    calibration_old_roi = ROI[:]
    calibration_old_trigger_zone = TRIGGER_ZONE[:]
    calibration_started_ms = ticks_ms()
    calibration_samples = []
    calibrating = True
    last_status_message = "CALIBRATING"


def detect_calibration_blob(img):
    """Find either color in the large search area used for calibration."""
    if img is None:
        return None
    try:
        blobs = img.find_blobs(
            [RED_THRESHOLD, BLUE_THRESHOLD],
            roi=CALIB_SEARCH_ROI,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
        )
    except Exception:
        return None
    return select_best_blob(blobs)


def finish_auto_roi(success):
    """Commit a successful calibration or restore its pre-calibration state."""
    global ROI, TRIGGER_ZONE, GRAB_CENTER_Y, CALIBRATED_CENTER_X
    global BALL_WIDTH, BALL_HEIGHT
    global calibrating, calibration_samples
    global calibration_old_roi, calibration_old_trigger_zone
    global last_status_message
    if success and len(calibration_samples) >= CALIB_MIN_SAMPLES:
        center_x = median_int([sample[0] for sample in calibration_samples])
        center_y = median_int([sample[1] for sample in calibration_samples])
        ball_width = median_int([sample[2] for sample in calibration_samples])
        ball_height = median_int([sample[3] for sample in calibration_samples])
        new_roi, new_trigger_zone = build_calibrated_regions(
            center_x, center_y, ball_width, ball_height)
        ROI = new_roi
        TRIGGER_ZONE = new_trigger_zone
        CALIBRATED_CENTER_X = center_x
        GRAB_CENTER_Y = center_y
        BALL_WIDTH = ball_width
        BALL_HEIGHT = ball_height
        if save_roi_config():
            last_status_message = "ROI SAVED"
        else:
            last_status_message = "ROI SAVE FAILED"
    else:
        if calibration_old_roi is not None:
            ROI = calibration_old_roi[:]
        if calibration_old_trigger_zone is not None:
            TRIGGER_ZONE = calibration_old_trigger_zone[:]
        last_status_message = "CALIBRATION FAILED"
    calibrating = False
    calibration_samples = []
    calibration_old_roi = None
    calibration_old_trigger_zone = None


def calibration_process(img, now_ms=None):
    """Collect one sample per consecutive valid frame and finalize calibration."""
    global calibration_samples
    if not calibrating:
        return None
    if now_ms is None:
        now_ms = ticks_ms()
    if now_ms - calibration_started_ms >= CALIB_DURATION_MS:
        finish_auto_roi(len(calibration_samples) >= CALIB_MIN_SAMPLES)
        return None

    blob = detect_calibration_blob(img)
    if blob is None:
        calibration_samples = []
        return None
    x = _blob_value(blob, "x")
    y = _blob_value(blob, "y")
    width = _blob_value(blob, "w")
    height = _blob_value(blob, "h")
    center_x = x + width // 2
    center_y = y + height // 2
    inside_target = abs(center_x - GRAB_CENTER_X) <= CALIB_X_TOLERANCE
    if not inside_target:
        calibration_samples = []
        return blob
    calibration_samples.append((center_x, center_y, width, height))
    if len(calibration_samples) >= CALIB_MIN_SAMPLES:
        finish_auto_roi(True)
    return blob


# ========================== Touch and Display ==========================

def point_in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


def screen_to_image_point(x, y, image_width, image_height):
    return (
        int(x * image_width / max(1, SCREEN_WIDTH)),
        int(y * image_height / max(1, SCREEN_HEIGHT)),
    )


def touch_process(touch, image_width=CAMERA_WIDTH, image_height=CAMERA_HEIGHT):
    """Poll touch input for debug display selection or AUTO ROI."""
    global last_touch_ms
    if touch is None:
        return
    now = ticks_ms()
    if now - last_touch_ms < TOUCH_DEBOUNCE_MS:
        return
    try:
        point = touch.read()
    except Exception:
        return
    if not point or len(point) < 3 or not point[2]:
        return
    x, y = screen_to_image_point(point[0], point[1], image_width, image_height)
    if point_in_rect(x, y, BUTTON_RED):
        set_mode(MODE_RED, "touch")
        last_touch_ms = now
    elif point_in_rect(x, y, BUTTON_BLUE):
        set_mode(MODE_BLUE, "touch")
        last_touch_ms = now
    elif point_in_rect(x, y, BUTTON_AUTO_ROI):
        start_auto_roi()
        last_touch_ms = now


def _color(name, fallback):
    if image is None:
        return fallback
    return getattr(image, name, fallback)


def _draw_text(img, x, y, text, color):
    try:
        img.draw_string(x, y, text, color)
    except Exception:
        pass


def draw_ui(img, blob):
    """Draw ROI, target geometry, state text, and both touch buttons."""
    if img is None:
        return
    white = _color("COLOR_WHITE", 0xFFFFFF)
    yellow = _color("COLOR_YELLOW", 0xFFFF00)
    red = _color("COLOR_RED", 0xFF0000)
    blue = _color("COLOR_BLUE", 0x0000FF)
    green = _color("COLOR_GREEN", 0x00FF00)
    try:
        img.draw_rect(ROI[0], ROI[1], ROI[2], ROI[3], yellow)
        img.draw_rect(TRIGGER_ZONE[0], TRIGGER_ZONE[1],
                      TRIGGER_ZONE[2], TRIGGER_ZONE[3], green)
        if calibrating:
            img.draw_rect(CALIB_SEARCH_ROI[0], CALIB_SEARCH_ROI[1],
                          CALIB_SEARCH_ROI[2], CALIB_SEARCH_ROI[3], blue)
        valid_blob = filter_blob(blob)
        if valid_blob is not None:
            x = _blob_value(valid_blob, "x")
            y = _blob_value(valid_blob, "y")
            width = _blob_value(valid_blob, "w")
            height = _blob_value(valid_blob, "h")
            center_x = x + width // 2
            center_y = y + height // 2
            img.draw_rect(x, y, width, height, green)
            if hasattr(img, "draw_circle"):
                img.draw_circle(center_x, center_y, 5, red)
        mode_name = COLOR_NAMES[display_mode]
        if calibrating:
            status = "CALIBRATING {}/{}".format(
                len(calibration_samples), CALIB_MIN_SAMPLES)
        elif recognition_armed:
            status = "SEARCHING"
        else:
            status = "WAIT CMD"
        _draw_text(img, 10, 10, "MODE: {}".format(mode_name), white)
        _draw_text(img, 10, 35, "STATUS: {}".format(status), white)
        if last_status_message and not calibrating:
            _draw_text(img, 10, 60, last_status_message, white)

        for mode, rect, label, color in (
            (MODE_RED, BUTTON_RED, "RED", red),
            (MODE_BLUE, BUTTON_BLUE, "BLUE", blue),
        ):
            outline = color if display_mode == mode else white
            img.draw_rect(rect[0], rect[1], rect[2], rect[3], outline)
            marker = "[X]" if display_mode == mode else "[ ]"
            _draw_text(img, rect[0] + 20, rect[1] + 12,
                       "{} {}".format(marker, label), outline)
        auto_outline = yellow if calibrating else white
        img.draw_rect(BUTTON_AUTO_ROI[0], BUTTON_AUTO_ROI[1],
                      BUTTON_AUTO_ROI[2], BUTTON_AUTO_ROI[3], auto_outline)
        auto_marker = "[X]" if calibrating else "[ ]"
        _draw_text(img, BUTTON_AUTO_ROI[0] + 12, BUTTON_AUTO_ROI[1] + 12,
                   "{} AUTO ROI".format(auto_marker), auto_outline)
    except Exception:
        pass


# ========================== Main Loop ==========================

def main():
    """Initialize MaixCAM2 and keep vision running until the process exits."""
    if not MAIXPY:
        raise RuntimeError("This script must run on MaixCAM2 with MaixPy")

    # The light is initialized and turned on before the camera starts.
    light_init()
    light_on()
    try:
        serial = init_uart()
        cam = init_camera()
        disp = init_display()
        touch = init_touchscreen()
        load_roi_config()

        print("MaixCAM2 red/blue ball ROI recognizer started")
        print("UART4 {} TX=A21 RX=A22".format(UART_DEVICE))
        while not app.need_exit():
            uart_process(serial)
            image_frame = cam.read()
            image_width = image_frame.width() if hasattr(image_frame, "width") else CAMERA_WIDTH
            image_height = image_frame.height() if hasattr(image_frame, "height") else CAMERA_HEIGHT
            touch_process(touch, image_width, image_height)
            if calibrating:
                blob = calibration_process(image_frame)
            elif recognition_armed:
                blob = detect_ball(image_frame)
                update_detection(blob, serial)
            else:
                blob = None
            draw_ui(image_frame, blob)
            disp.show(image_frame)
            sleep_ms(MAIN_LOOP_SLEEP_MS)
    finally:
        light_off()
        print("program exit")


class _FakeBlob:
    def __init__(self, x, y, width, height, pixels):
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


class _FakeSerial:
    def __init__(self):
        self.sent = []

    def write(self, data):
        self.sent.append(bytes(data))


class _FakeImage:
    def __init__(self, blobs):
        self.blobs = blobs
        self.thresholds = None
        self.kwargs = None

    def find_blobs(self, thresholds, **kwargs):
        self.thresholds = thresholds
        self.kwargs = kwargs
        return self.blobs


class _ColorFakeImage(_FakeImage):
    def __init__(self, blob, visible_threshold):
        super().__init__([blob])
        self.visible_threshold = visible_threshold

    def find_blobs(self, thresholds, **kwargs):
        self.thresholds = thresholds
        self.kwargs = kwargs
        if thresholds == [self.visible_threshold]:
            return self.blobs
        return []


class _FakeTouch:
    def __init__(self, point):
        self.point = point

    def read(self):
        return self.point


def _selftest():
    global ROI, TRIGGER_ZONE, ROI_CONFIG_PATH, GRAB_CENTER_X
    global CALIBRATED_CENTER_X, GRAB_CENTER_Y, BALL_WIDTH, BALL_HEIGHT
    global current_mode, display_mode, detected_latched, recognition_armed
    global calibrating, last_touch_ms
    valid = _FakeBlob(295, 195, 50, 50, 1800)
    outside_trigger = _FakeBlob(245, 195, 50, 50, 1800)
    smaller = _FakeBlob(305, 205, 30, 30, 900)
    non_square = _FakeBlob(150, 90, 100, 20, 1800)
    assert filter_blob(non_square) is None
    assert select_best_blob([smaller, valid]) is valid
    assert is_blob_in_trigger_zone(outside_trigger) is False
    assert is_blob_in_trigger_zone(valid) is True
    assert median_int([190, 220, 203]) == 203
    assert median_int([190, 203, 220, 214]) == 208
    calibrated_roi, calibrated_trigger = build_calibrated_regions(350, 222, 50, 50)
    assert point_in_rect(350, 222, calibrated_roi)
    assert point_in_rect(350, 222, calibrated_trigger)
    assert not point_in_rect(315, 222, calibrated_trigger)
    fake_image = _FakeImage([valid])
    assert detect_ball(fake_image, MODE_RED) is valid
    assert fake_image.kwargs["roi"] == ROI
    assert fake_image.kwargs["merge"] is True
    calibration_image = _FakeImage([valid])
    assert detect_calibration_blob(calibration_image) is valid
    assert len(calibration_image.thresholds) == 2
    assert calibration_image.kwargs["roi"] == CALIB_SEARCH_ROI

    serial = _FakeSerial()
    assert recognition_armed is False
    idle_blue_image = _ColorFakeImage(valid, BLUE_THRESHOLD)
    idle_blob = detect_ball(idle_blue_image, MODE_BLUE)
    assert idle_blob is valid
    update_detection(idle_blob, serial)
    assert serial.sent == []

    process_command_bytes(b"2\r\n")
    assert current_mode == MODE_BLUE
    assert recognition_armed is True
    blue_image = _ColorFakeImage(valid, BLUE_THRESHOLD)
    assert detect_ball(blue_image) is valid
    assert blue_image.thresholds == [BLUE_THRESHOLD]
    update_detection(valid, serial)
    assert serial.sent == [b"1\n"]

    assert recognition_armed is False
    assert detected_latched is True
    for _ in range(100):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n"]

    process_command_bytes(b"2")
    assert recognition_armed is True
    next_blue_image = _ColorFakeImage(valid, BLUE_THRESHOLD)
    next_blob = detect_ball(next_blue_image)
    assert next_blob is valid
    update_detection(next_blob, serial)
    assert serial.sent == [b"1\n", b"1\n"]

    process_command_bytes(b"1")
    assert current_mode == MODE_RED
    assert recognition_armed is True
    wrong_blue_image = _ColorFakeImage(valid, BLUE_THRESHOLD)
    assert detect_ball(wrong_blue_image) is None
    assert wrong_blue_image.thresholds == [RED_THRESHOLD]
    red_image = _ColorFakeImage(valid, RED_THRESHOLD)
    assert detect_ball(red_image) is valid
    assert red_image.thresholds == [RED_THRESHOLD]
    update_detection(valid, serial)
    assert serial.sent == [b"1\n", b"1\n", b"1\n"]

    task_mode_before_touch = current_mode
    recognition_armed = False
    last_touch_ms = 0
    touch_process(_FakeTouch((230, 430, 1)))
    assert display_mode == MODE_BLUE
    assert current_mode == task_mode_before_touch
    assert recognition_armed is False

    import os
    import tempfile
    saved_state = (ROI[:], TRIGGER_ZONE[:], ROI_CONFIG_PATH,
                   GRAB_CENTER_X, CALIBRATED_CENTER_X, GRAB_CENTER_Y,
                   BALL_WIDTH, BALL_HEIGHT)
    with tempfile.TemporaryDirectory() as temp_dir:
        config_path = os.path.join(temp_dir, "roi.json")
        ROI_CONFIG_PATH = config_path
        calibrated_roi, calibrated_trigger = build_calibrated_regions(350, 222, 50, 50)
        ROI = calibrated_roi[:]
        TRIGGER_ZONE = calibrated_trigger[:]
        GRAB_CENTER_Y = 222
        BALL_WIDTH = 50
        BALL_HEIGHT = 50
        assert save_roi_config() is True
        ROI = DEFAULT_ROI[:]
        TRIGGER_ZONE = DEFAULT_TRIGGER_ZONE[:]
        assert load_roi_config() is True
        assert ROI == calibrated_roi
        assert TRIGGER_ZONE == calibrated_trigger

        detected_latched = True
        last_touch_ms = 0
        touch_process(_FakeTouch((500, 430, 1)))
        assert calibrating is True
        assert detected_latched is True
        assert recognition_armed is False
        finish_auto_roi(False)

        inside = _FakeBlob(325, 235, 50, 50, 1800)
        uart_count_before_calibration = len(serial.sent)
        start_auto_roi()
        for _ in range(CALIB_MIN_SAMPLES):
            calibration_process(_FakeImage([inside]), calibration_started_ms + 100)
        assert calibrating is False
        assert len(serial.sent) == uart_count_before_calibration
        assert os.path.exists(config_path)
        assert CALIBRATED_CENTER_X == 350
        assert last_status_message == "ROI SAVED"
        saved_roi = ROI[:]
        saved_trigger = TRIGGER_ZONE[:]
        assert point_in_rect(350, 260, saved_roi)
        assert point_in_rect(350, 260, saved_trigger)
        assert saved_roi != DEFAULT_ROI
        assert saved_trigger != DEFAULT_TRIGGER_ZONE

        start_auto_roi()
        calibration_process(_FakeImage([]), calibration_started_ms + CALIB_DURATION_MS)
        assert calibrating is False
        assert ROI == saved_roi
        assert TRIGGER_ZONE == saved_trigger

    ROI = saved_state[0]
    TRIGGER_ZONE = saved_state[1]
    ROI_CONFIG_PATH = saved_state[2]
    GRAB_CENTER_X = saved_state[3]
    CALIBRATED_CENTER_X = saved_state[4]
    GRAB_CENTER_Y = saved_state[5]
    BALL_WIDTH = saved_state[6]
    BALL_HEIGHT = saved_state[7]
    current_mode = MODE_RED
    display_mode = MODE_RED
    detected_latched = False
    recognition_armed = False
    calibrating = False
    with open(__file__, "r") as source_file:
        source = source_file.read()
    assert "while not app.need_exit()" in source
    assert "elif recognition_armed:" in source
    assert 'status = "WAIT CMD"' in source
    assert "finally:" in source
    assert "light_off()" in source
    print("red-blue ball selftest passed")


if __name__ == "__main__":
    import sys

    if "--selftest" in sys.argv:
        _selftest()
    else:
        main()
