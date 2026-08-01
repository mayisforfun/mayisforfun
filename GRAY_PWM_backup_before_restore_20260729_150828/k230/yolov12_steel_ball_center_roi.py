from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os, sys, gc, time, random, utime
import ujson
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo
from time import ticks_ms
from machine import FPIOA
from machine import Pin
from machine import PWM
from machine import Timer
from machine import UART
import time


class YOLOv12App(AIBase):
    def __init__(self, kmodel_path, model_input_size, anchors, confidence_threshold=0.5, nms_threshold=0.2, rgb888p_size=[224,224], display_size=[1920,1080], guide_roi=None, debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.class_id = ["gangzhu"]
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.anchors = anchors
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        # Guide box in original camera coordinates: (x, y, width, height).
        # Only a steel-ball center inside this box is considered valid.
        self.guide_roi = guide_roi
        self.debug_mode = debug_mode
        self.last_ball_center = None
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right = self.get_padding_param()
            print("padding: {} {} {} {}".format(top, bottom, left, right))
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [104, 117, 123])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self, results):
        det_res = []
        with ScopedTiming("postprocess", self.debug_mode > 0):
            for i in range(2100):
                result = results[0][0][:, i]
                max_score = max(result[4:])
                if max_score > self.confidence_threshold:
                    scale = max(self.rgb888p_size) / max(self.model_input_size)
                    x = result[0] * scale
                    y = result[1] * scale
                    w = result[2] * scale
                    h = result[3] * scale
                    if not self.center_in_guide(int(x), int(y)):
                        continue
                    det_res.append([x, y, w, h, list(result[4:]).index(max_score), max_score])

            # Keep all candidates. draw_result() selects the highest-confidence
            # candidate inside the guide box. Keeping only one candidate per
            # class here would allow an out-of-box ball to hide an in-box ball.
            det_res.sort(key=lambda x: x[-1], reverse=True)
        return det_res

    def center_in_guide(self, center_x, center_y):
        if self.guide_roi is None:
            return True
        roi_x, roi_y, roi_w, roi_h = self.guide_roi
        return (center_x >= roi_x and center_x < roi_x + roi_w and
                center_y >= roi_y and center_y < roi_y + roi_h)

    def draw_guide(self, pl):
        if self.guide_roi is None:
            return
        roi_x, roi_y, roi_w, roi_h = self.guide_roi
        dx = roi_x * self.display_size[0] // self.rgb888p_size[0]
        dy = roi_y * self.display_size[1] // self.rgb888p_size[1]
        dw = roi_w * self.display_size[0] // self.rgb888p_size[0]
        dh = roi_h * self.display_size[1] // self.rgb888p_size[1]
        pl.osd_img.draw_rectangle(dx, dy, dw, dh,
                                  color=(255, 255, 255, 0), thickness=3)
        pl.osd_img.draw_string_advanced(dx + 4, dy + 4, 24,
                                        "BALL ROI",
                                        color=(255, 255, 255, 0))

    def detect_circle_center(self, img, det):
        # Return the steel ball center in original camera coordinates.
        x, y, w, h = map(lambda v: int(round(v, 0)), det[:4])
        fallback_r = max(2, min(w, h) // 2)
        pad = max(6, fallback_r // 3)

        x0 = max(0, x - w // 2 - pad)
        y0 = max(0, y - h // 2 - pad)
        x1 = min(self.rgb888p_size[0], x + w // 2 + pad)
        y1 = min(self.rgb888p_size[1], y + h // 2 + pad)
        roi_w = x1 - x0
        roi_h = y1 - y0
        if roi_w <= 4 or roi_h <= 4:
            return x, y, fallback_r

        try:
            roi = img.crop(x0, y0, roi_w, roi_h)    # 对DOI进行处理
            gray = roi.to_grayscale()    # 灰度
            edge = gray.find_edges(image.EDGE_CANNY, threshold=(45, 90))    #边缘检测
            if edge is None:
                edge = gray
        except Exception as e:
            print("circle roi error:", e)
            return x, y, fallback_r

        min_x = roi_w
        min_y = roi_h
        max_x = 0
        max_y = 0
        count = 0

        # Sampling every 2 pixels is fast enough for real-time use on K230.
        for yy in range(0, roi_h, 2):
            for xx in range(0, roi_w, 2):
                p = edge.get_pixel(xx, yy)
                if p is not None and p > 0:
                    if xx < min_x:
                        min_x = xx
                    if xx > max_x:
                        max_x = xx
                    if yy < min_y:
                        min_y = yy
                    if yy > max_y:
                        max_y = yy
                    count += 1

        if count < 8:
            return x, y, fallback_r

        cx = x0 + (min_x + max_x) // 2
        cy = y0 + (min_y + max_y) // 2
        radius = max(2, max(max_x - min_x, max_y - min_y) // 2)
        return cx, cy, radius

    def draw_result(self, pl, img, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            self.last_ball_center = None
            self.last_display_center = None
            pl.osd_img.clear()
            self.draw_guide(pl)
            if dets:
                for det in dets:
                    cam_x, cam_y, cam_w, cam_h = map(lambda v: int(round(v, 0)), det[:4])

                    # Reject obvious out-of-box candidates before the more
                    # expensive edge-based circle-center refinement.
                    if not self.center_in_guide(cam_x, cam_y):
                        continue

                    ball_x, ball_y, ball_r = self.detect_circle_center(img, det)
                    if not self.center_in_guide(ball_x, ball_y):
                        continue

                    sx = cam_x * self.display_size[0] // self.rgb888p_size[0]
                    sy = cam_y * self.display_size[1] // self.rgb888p_size[1]
                    sw = cam_w * self.display_size[0] // self.rgb888p_size[0]
                    sh = cam_h * self.display_size[1] // self.rgb888p_size[1]
                    pl.osd_img.draw_rectangle(sx - sw // 2, sy - sh // 2, sw, sh, color=(255, 0, 255, 0), thickness=2)
                    pl.osd_img.draw_string_advanced(sx - sw // 2, sy - sh // 2, 30, "{} {}".format(self.class_id[det[-2]], round(det[-1], 2)), color=(255, 0, 255, 0))

                    self.last_ball_center = (ball_x, ball_y)
                    dsx = ball_x * self.display_size[0] // self.rgb888p_size[0]
                    dsy = ball_y * self.display_size[1] // self.rgb888p_size[1]
                    dsr = max(2, ball_r * self.display_size[0] // self.rgb888p_size[0])
                    self.last_display_center = (dsx, dsy)
                    pl.osd_img.draw_circle(dsx, dsy, dsr, color=(255, 255, 0, 0), thickness=2)
                    pl.osd_img.draw_cross(dsx, dsy, color=(255, 0, 255, 0), size=12, thickness=2)
                    pl.osd_img.draw_string_advanced(dsx + 8, dsy + 8, 24, "({},{})".format(ball_x, ball_y), color=(255, 255, 0, 0))

                    # dets is sorted by confidence, so the first valid one is
                    # the desired steel ball inside the guide box.
                    break

    def get_padding_param(self):
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        ratio_w = dst_w / self.rgb888p_size[0]
        ratio_h = dst_h / self.rgb888p_size[1]
        ratio = min(ratio_w, ratio_h)
        new_w = int(ratio * self.rgb888p_size[0])
        new_h = int(ratio * self.rgb888p_size[1])
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(0))
        bottom = int(round(dh * 2 + 0.1))
        left = int(round(0))
        right = int(round(dw * 2 - 0.1))
        return top, bottom, left, right


if __name__ == "__main__":
    display_mode = "lcd"
    rgb888p_size = [1920, 1080]

    if display_mode == "hdmi":
        display_size = [1920, 1080]
    else:
        display_size = [800, 480]

    kmodel_path = "/sdcard/best.kmodel"
    confidence_threshold = 0.4
    nms_threshold = 0.2
    anchors = None

    # Detection guide box, using the original 1920x1080 camera coordinates.
    # Adjust these four values after observing the LCD overlay.
    guide_roi = (160, 300, 1600, 480)  # x, y, width, height

    # Put camera/display/model creation inside try as well. If the IDE stops
    # the script while pl.create() is opening the sensor, finally can still
    # release any partially-created media resources before the next run.
    pl = None
    yolo_det = None
    try:
        pl = PipeLine(rgb888p_size=rgb888p_size,
                      display_size=display_size,
                      display_mode=display_mode)
        pl.create()
        yolo_det = YOLOv12App(kmodel_path,
                              model_input_size=[320, 320],
                              anchors=anchors,
                              confidence_threshold=confidence_threshold,
                              nms_threshold=nms_threshold,
                              rgb888p_size=rgb888p_size,
                              display_size=display_size,
                              guide_roi=guide_roi,
                              debug_mode=0)
        yolo_det.config_preprocess()

        # Enable this block when coordinates need to be sent to the lower controller.

        fpioa = FPIOA()
        fpioa.help()
        fpioa.set_function(32, FPIOA.UART3_TXD)
        fpioa.set_function(33, FPIOA.UART3_RXD)
#        key = Pin(53, Pin.IN, Pin.PULL_DOWN)
        uart3 = UART(UART.UART3, 115200)

        tx_position = [0xAA, 0xAA, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF]
        def send_position(center_x, center_y):
            tx_position[2] = (center_x // 256) % 256
            tx_position[3] = center_x % 256
            tx_position[4] = (center_y // 256) % 256
            tx_position[5] = center_y % 256
            uart3.write(bytearray(tx_position))
            print(bytearray(tx_position))

        while True:
            os.exitpoint()
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = yolo_det.run(img)
                yolo_det.draw_result(pl, img, res)

#                uart3.write(bytearray([0xAA, 0xAA, 0x01, 0x02, 0x03, 0x04, 0xFF, 0xFF]))
#                print(bytearray([0xAA, 0xAA, 0x01, 0x02, 0x03, 0x04, 0xFF, 0xFF]))

                # The value is in original camera coordinates: (x, y).
#                if yolo_det.last_ball_center:
#                if hasattr(yolo_det, 'last_display_center') and yolo_det.last_display_center:
                if yolo_det.last_ball_center:

                    send_position(yolo_det.last_ball_center[0], yolo_det.last_ball_center[1])

                pl.show_image()
                gc.collect()
                time.sleep_ms(50)

    except Exception as e:
        print(e)
    finally:
        if yolo_det is not None:
            try:
                yolo_det.deinit()
            except Exception as cleanup_error:
                print("model cleanup error:", cleanup_error)
        if pl is not None:
            try:
                pl.destroy()
            except Exception as cleanup_error:
                print("pipeline cleanup error:", cleanup_error)
