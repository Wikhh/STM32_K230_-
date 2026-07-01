import time, os, gc

from media.sensor import *
from media.display import *
from media.media import *

from machine import UART, FPIOA


# ==================================================
# 图像参数
# ==================================================
DETECT_WIDTH = 320
DETECT_HEIGHT = 240

SHOW_TO_IDE = True


# ==================================================
# 矩形检测参数
# ==================================================
RECT_THRESHOLD = 12000
RECT_X_GRADIENT = 25
RECT_Y_GRADIENT = 25


# ==================================================
# 矩形稳定锁定参数
# ==================================================
STABLE_THRESHOLD = 3       # 连续识别到同一个框 5 次后锁定
CENTER_THRESHOLD = 15      # 判断同一个框：中心允许偏差
SIZE_THRESHOLD = 20        # 判断同一个框：宽高允许偏差

locked_rect = None         # 最终锁定的矩形
stable_rect = None         # 当前正在累计稳定次数的矩形
stable_count = 0           # 连续稳定次数
rect_locked = False        # 是否已经锁定矩形


# ==================================================
# 激光点阈值
# LAB 阈值格式：
# (L_min, L_max, A_min, A_max, B_min, B_max)
# ==================================================
red_laser_threshold = (87, 100, -17, -68, -25, 86)
green_laser_threshold = (90, 63, 33, 63, 1, 15)

# ==================================================
# UART 配置
# ==================================================
UART1_TX_PIN = 9
UART1_RX_PIN = 10
UART_BAUDRATE = 115200

PACKET_HEADER = 0xFF
PACKET_FOOTER = 0xFE
PACKET_CMD_ERROR = 0x01
PACKET_CMD_LOST = 0x02


# ==================================================
# 激光滤波参数
# ==================================================
FILTER_ALPHA = 0.25

red_filtered_cx = DETECT_WIDTH // 2
red_filtered_cy = DETECT_HEIGHT // 2
red_filter_inited = False

green_filtered_cx = DETECT_WIDTH // 2
green_filtered_cy = DETECT_HEIGHT // 2
green_filter_inited = False

lost_count = 0
LOST_LIMIT = 5

debug_count = 0

display_skip = 0
draw_skip = 0
# ==================================================
# 全局对象
# ==================================================
sensor = None
uart = None


# ==================================================
# 目标矩形
# ==================================================
target_rect_corners = [[0, 0] for _ in range(4)]


# ==================================================
# 工具函数
# ==================================================

def lowpass_point(raw_x, raw_y, old_x, old_y, inited, alpha):
    if not inited:
        return raw_x, raw_y, True

    new_x = int(alpha * raw_x + (1 - alpha) * old_x)
    new_y = int(alpha * raw_y + (1 - alpha) * old_y)

    return new_x, new_y, True

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def int16_to_bytes(value):
    value = int(clamp(value, -32768, 32767))

    if value < 0:
        value += 65536

    return (value >> 8) & 0xFF, value & 0xFF


def send_error_packet(error_x, error_y):
    global uart

    if uart is None:
        return

    ex_h, ex_l = int16_to_bytes(error_x)
    ey_h, ey_l = int16_to_bytes(error_y)

    checksum = (PACKET_CMD_ERROR + ex_h + ex_l + ey_h + ey_l) & 0xFF

    uart.write(bytearray([
        PACKET_HEADER,
        PACKET_CMD_ERROR,
        ex_h,
        ex_l,
        ey_h,
        ey_l,
        checksum,
        PACKET_FOOTER
    ]))


def send_lost_packet():
    global uart

    if uart is None:
        return

    uart.write(bytearray([
        PACKET_HEADER,
        PACKET_CMD_LOST,
        0,
        0,
        0,
        0,
        PACKET_CMD_LOST & 0xFF,
        PACKET_FOOTER
    ]))


def copy_rect(corners):
    """
    深拷贝矩形角点，避免引用被后续修改
    """
    return [
        [int(corners[0][0]), int(corners[0][1])],
        [int(corners[1][0]), int(corners[1][1])],
        [int(corners[2][0]), int(corners[2][1])],
        [int(corners[3][0]), int(corners[3][1])]
    ]


def rect_center_and_size(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]

    cx = sum(xs) / 4
    cy = sum(ys) / 4

    w = max(xs) - min(xs)
    h = max(ys) - min(ys)

    return cx, cy, w, h


def is_same_rect(r1, r2):
    """
    判断两个矩形是不是同一个目标
    """
    if r1 is None or r2 is None:
        return False

    c1x, c1y, w1, h1 = rect_center_and_size(r1)
    c2x, c2y, w2, h2 = rect_center_and_size(r2)

    center_dist = abs(c1x - c2x) + abs(c1y - c2y)
    size_diff = abs(w1 - w2) + abs(h1 - h2)

    if center_dist < CENTER_THRESHOLD and size_diff < SIZE_THRESHOLD:
        return True

    return False


def smooth_rect(old_rect, new_rect):
    """
    对正在稳定计数的矩形做轻微平均，减少角点抖动
    只在锁定前使用
    """
    if old_rect is None:
        return copy_rect(new_rect)

    result = []

    for i in range(4):
        x = int(old_rect[i][0] * 0.6 + new_rect[i][0] * 0.4)
        y = int(old_rect[i][1] * 0.6 + new_rect[i][1] * 0.4)
        result.append([x, y])

    return result


def get_rect_center(corners):
    cx = sum(p[0] for p in corners) // 4
    cy = sum(p[1] for p in corners) // 4

    return cx, cy


def get_rect_roi(corners, margin=8):
    """
    根据锁定矩形生成激光搜索 ROI
    """
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]

    x_min = int(clamp(min(xs) - margin, 0, DETECT_WIDTH - 1))
    y_min = int(clamp(min(ys) - margin, 0, DETECT_HEIGHT - 1))
    x_max = int(clamp(max(xs) + margin, 0, DETECT_WIDTH - 1))
    y_max = int(clamp(max(ys) + margin, 0, DETECT_HEIGHT - 1))

    w = x_max - x_min
    h = y_max - y_min

    if w < 1:
        w = 1

    if h < 1:
        h = 1

    return (x_min, y_min, w, h)


def draw_rect_by_corners(img, corners, color):
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]

        img.draw_line(
            p1[0],
            p1[1],
            p2[0],
            p2[1],
            color=color
        )

    for p in corners:
        img.draw_circle(p[0], p[1], 3, color=color)


def find_laser_blob(blobs):
    """
    从所有红色光斑中找最像激光点的一个
    """
    best_blob = None
    best_score = 0

    for b in blobs:
        pixels = b.pixels()

        if pixels < 1:
            continue

        if pixels > 300:
            continue

        if b.w() > 40 or b.h() > 40:
            continue

        score = pixels

        if score > best_score:
            best_score = score
            best_blob = b

    return best_blob


# ==================================================
# 矩形识别：找当前帧最可靠矩形
# ==================================================
def find_best_rect(img):
    rects = img.find_rects(
        threshold=RECT_THRESHOLD,
        x_gradient=RECT_X_GRADIENT,
        y_gradient=RECT_Y_GRADIENT
    )

    best = None
    best_score = 0

    for r in rects:
        corners = r.corners()

        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]

        w = max(xs) - min(xs)
        h = max(ys) - min(ys)

        if w < 40 or h < 40:
            continue

        if w > 250 or h > 250:
            continue

        ratio = w / h

        if ratio < 0.6 or ratio > 1.6:
            continue

        area = r.magnitude()

        # 过滤右侧干扰框
        cx = (min(xs) + max(xs)) // 2
        if cx > 180:
            continue

        # 面积越大，优先级越高
        score = area

        if score > best_score:
            best_score = score
            best = corners

    if best is None:
        return None

    return copy_rect(best)


# ==================================================
# 矩形锁定逻辑
# ==================================================
def update_locked_rect(current_rect):
    """
    连续识别到同一个矩形 STABLE_THRESHOLD 次后锁定
    锁定后不再更新
    """
    global stable_rect
    global stable_count
    global locked_rect
    global rect_locked

    # 已经锁定，不再更新矩形
    if rect_locked:
        return locked_rect

    # 当前帧没识别到矩形
    if current_rect is None:
        stable_count = 0
        stable_rect = None
        return None

    # 第一次识别到矩形
    if stable_rect is None:
        stable_rect = copy_rect(current_rect)
        stable_count = 1
        return stable_rect

    # 和上一帧是同一个矩形
    if is_same_rect(stable_rect, current_rect):
        stable_count += 1
        stable_rect = smooth_rect(stable_rect, current_rect)

        if stable_count >= STABLE_THRESHOLD:
            locked_rect = copy_rect(stable_rect)
            rect_locked = True
            print("rect locked:", locked_rect)

        return stable_rect

    # 识别到的是另一个矩形，重新计数
    stable_rect = copy_rect(current_rect)
    stable_count = 1
    return stable_rect


# ==================================================
# 主程序
# ==================================================
try:
    print("start")

    # ==================================================
    # UART 初始化
    # ==================================================
    fpioa = FPIOA()
    fpioa.set_function(UART1_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(UART1_RX_PIN, FPIOA.UART1_RXD)

    uart = UART(UART.UART1, baudrate=UART_BAUDRATE)

    # ==================================================
    # 摄像头初始化
    # ==================================================
    sensor = Sensor()
    sensor.reset()
    time.sleep_ms(200)

    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(
        Display.VIRT,
        width=DETECT_WIDTH,
        height=DETECT_HEIGHT,
        to_ide=True
    )

    MediaManager.init()

    sensor.run()
    time.sleep_ms(200)

    clock = time.clock()

    while True:
        clock.tick()
        os.exitpoint()

        img = sensor.snapshot()

        # ==================================================
        # 1. 识别当前帧矩形
        # ==================================================
        if rect_locked:
            current_rect = None
            target_rect_corners = locked_rect
        else:
            current_rect = find_best_rect(img)
            target_rect_corners = update_locked_rect(current_rect)

        # ==================================================
        # 3. 显示当前候选矩形和锁定状态
        # ==================================================
        if SHOW_TO_IDE:
            if current_rect is not None and not rect_locked:
                # 黄色：当前识别候选框
                draw_rect_by_corners(img, current_rect, color=(255, 255, 0))

            if target_rect_corners is not None:
                if rect_locked:
                    # 绿色：已经锁定的矩形
                    draw_rect_by_corners(img, target_rect_corners, color=(0, 255, 0))
                else:
                    # 蓝色：正在累计稳定次数的矩形
                    draw_rect_by_corners(img, target_rect_corners, color=(0, 0, 255))

            # ==================================================
            # 4. 有目标矩形后，识别红色激光点和绿色激光点
            # ==================================================
            if target_rect_corners is not None:
                rect_center_x, rect_center_y = get_rect_center(target_rect_corners)

                laser_roi = get_rect_roi(target_rect_corners, margin=12)

                # ------------------------------
                # 找红色激光点
                # ------------------------------
                red_blobs = img.find_blobs(
                    [red_laser_threshold],
                    roi=laser_roi,
                    pixels_threshold=1,
                    area_threshold=1,
                    merge=False
                )

                red_laser = find_laser_blob(red_blobs)

                # ------------------------------
                # 找绿色激光点
                # ------------------------------
                green_blobs = img.find_blobs(
                    [green_laser_threshold],
                    roi=laser_roi,
                    pixels_threshold=1,
                    area_threshold=1,
                    merge=False
                )

                green_laser = find_laser_blob(green_blobs)

                if red_laser and green_laser:
                    lost_count = 0

                    red_raw_cx = red_laser.cx()
                    red_raw_cy = red_laser.cy()

                    green_raw_cx = green_laser.cx()
                    green_raw_cy = green_laser.cy()

                    red_filtered_cx, red_filtered_cy, red_filter_inited = lowpass_point(
                        red_raw_cx,
                        red_raw_cy,
                        red_filtered_cx,
                        red_filtered_cy,
                        red_filter_inited,
                        FILTER_ALPHA
                    )

                    green_filtered_cx, green_filtered_cy, green_filter_inited = lowpass_point(
                        green_raw_cx,
                        green_raw_cy,
                        green_filtered_cx,
                        green_filtered_cy,
                        green_filter_inited,
                        FILTER_ALPHA
                    )

                    # ==================================================
                    # 关键：红绿激光误差
                    # 红点在绿点右边，error_x 为正
                    # 红点在绿点下边，error_y 为正
                    # ==================================================
                    error_x = red_filtered_cx - green_filtered_cx
                    error_y = red_filtered_cy - green_filtered_cy

                    send_error_packet(error_x, error_y)

                    if SHOW_TO_IDE:
                        # 红色激光点
                        img.draw_rectangle(red_laser.rect(), color=(255, 0, 0))
                        img.draw_cross(red_raw_cx, red_raw_cy, color=(255, 0, 0))
                        img.draw_cross(red_filtered_cx, red_filtered_cy, color=(255, 128, 128))

                        # 绿色激光点
                        img.draw_rectangle(green_laser.rect(), color=(0, 255, 0))
                        img.draw_cross(green_raw_cx, green_raw_cy, color=(0, 255, 0))
                        img.draw_cross(green_filtered_cx, green_filtered_cy, color=(128, 255, 128))

                        # 红绿误差连线
                        img.draw_line(
                            green_filtered_cx,
                            green_filtered_cy,
                            red_filtered_cx,
                            red_filtered_cy,
                            color=(255, 255, 255)
                        )

                        # 矩形中心仅作为参考显示
                        img.draw_cross(rect_center_x, rect_center_y, color=(0, 0, 255))

                    debug_count += 1

                    if debug_count >= 10:
                        debug_count = 0
                        print(
                            "track",
                            "locked=", rect_locked,
                            "red_x=", red_filtered_cx,
                            "red_y=", red_filtered_cy,
                            "green_x=", green_filtered_cx,
                            "green_y=", green_filtered_cy,
                            "err_x=", error_x,
                            "err_y=", error_y,
                            "red_pixels=", red_laser.pixels(),
                            "green_pixels=", green_laser.pixels(),
                            "fps=", clock.fps()
                        )

                else:
                    lost_count += 1

                    if lost_count > LOST_LIMIT:
                        red_filter_inited = False
                        green_filter_inited = False

                    send_lost_packet()

                    debug_count += 1

                    if debug_count >= 10:
                        debug_count = 0
                        print(
                            "laser lost",
                            "red=", red_laser is not None,
                            "green=", green_laser is not None,
                            "locked=", rect_locked,
                            "fps=", clock.fps()
                        )

            else:
                send_lost_packet()

                debug_count += 1

                if debug_count >= 10:
                    debug_count = 0
                    print(
                        "no rect",
                        "locked=", rect_locked,
                        "stable_count=", stable_count,
                        "fps=", clock.fps()
                    )

        # ==================================================
        # 5. 显示图像
        # ==================================================
        if SHOW_TO_IDE:
            display_skip += 1

            if display_skip % 2 == 0:
                Display.show_image(img)

        if debug_count == 0:
            gc.collect()

        time.sleep_ms(10)


except Exception as e:
    print("error:", e)


finally:
    try:
        if sensor:
            sensor.stop()
    except Exception as e:
        print("sensor stop error:", e)

    try:
        Display.deinit()
    except Exception as e:
        print("Display deinit error:", e)

    try:
        MediaManager.deinit()
    except Exception as e:
        print("MediaManager deinit error:", e)

    try:
        if uart:
            uart.deinit()
    except Exception as e:
        print("UART deinit error:", e)

    print("exit")
