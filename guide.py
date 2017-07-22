from ev3dev.auto import *
from ev3dev.helper import Tank
import time
from math import *

class LineTracer:
    u"""PID制御によるライントレース
    キャリブレーション後のカラーセンサー値を目標値として旋回量を制御
    現時点では前進速度は固定値を想定
    →走行速度によって係数を調整する必要があるため
    """

    def __init__(self):
        # target_v 目標値
        # delta_t 周回時間
        self.refrection_target = 0.0
        self.delta_t = 0.025 # 10msec
        # TODO: P係数(要調整)
        self.k_p = 0.26
        # TODO: I係数(要調整)
        self.k_i = 0.1
        # TODO: D係数(要調整)
        self.k_d = 0.05
        # 前回偏差
        self.e_b = 0.0
        # 前回までの偏差値
        self.p_b = 0.0
        # 前回までの積分値
        self.i_b = 0.0
        # 前回までの微分値
        self.d_b = 0.0
        # 前回値
        #self.previous = [0 for _ in range(5) ]
        self.previous = 0

        self.color = ColorSensor()
        self.color.mode = self.color.MODE_REF_RAW #raw値
        self.color_reflection_fd = open(self.color._path + "/value0", "rb")

    def calibrate_color_sensor(self):
        target = 0
        gyro_rate_calibrate_count = 100
        color_val = 0
        for _ in range(gyro_rate_calibrate_count):
            color_val = self._read_fd(self.color_reflection_fd)
            target = target + color_val
            #pushPrevious(color_val)
            time.sleep(0.01)
        target = target / gyro_rate_calibrate_count
        #目標値を保管
        self.refrection_target = target
        self.previous = target

        u"""
        def pushPrevious(self, val)
        self.previous[0] = self.previous[1]
        self.previous[1] = self.previous[2]
        self.previous[2] = self.previous[3]
        self.previous[3] = self.previous[3]
        self.previous[4] = val
        """

    def line_tracing(self):
        u""" 速度、旋回値をpwm値として返却 """
        #センサー値を取得
        refrection_raw = self._read_fd(self.color_reflection_fd)
        ref_avarage = int(round((self.previous + refrection_raw) / 2))
        self.previous = refrection_raw

        #指示値を取得
        direction = self._calc_direction(ref_avarage)
        speed = 50 #固定値
        

        # NOTE: ライン左端を基準に走行させるために、旋回方向を - で反転している
        return speed, -direction, refrection_raw

    def shutdown(self):
        self.color_reflection_fd.close()

    def _calc_direction(self, brightness):
        u"""旋回量取得"""
        # error 偏差
        # p_n P制御
        # i_n I制御
        # d_n D制御

        # pid演算
        error = self.refrection_target - brightness
        p_n = self.k_p * error
        i_n = self.k_i * (self.i_b + ((error + self.e_b) / 2.0 * self.delta_t))
        d_n = self.k_d * (error - self.e_b) / self.delta_t
        direction = (p_n + i_n + d_n)

        # -100 ～100に収める
        if direction > 100:
            direction = 100
        elif direction < -100:
            direction = -100

        # 値の保存
        self.e_b = error
        self.p_b = p_n
        self.i_b = i_n
        self.d_b = d_n

        return direction

    # Function for fast reading from sensor files
    @staticmethod
    def _read_fd(fd):
        fd.seek(0)
        return int(fd.read().decode().strip())
