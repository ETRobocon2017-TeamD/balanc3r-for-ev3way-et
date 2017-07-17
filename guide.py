from ev3dev.auto import *
from ev3dev.helper import Tank
import time

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
        self.k_p = 0.3
        # TODO: I係数(要調整) 
        self.k_i = 0.1
        # TODO: D係数(要調整)
        self.k_d = 0.1
        # 前回偏差
        self.e_b = 0.0
        # 前回までの積分値
        self.i_b = 0.0

        self.color = ColorSensor()
        self.color.mode = self.color.MODE_REF_RAW #raw値
        self.color_reflection_fd = open(self.color._path + "/value0", "rb")

    def calibrate_color_sensor(self):
        target = 0
        gyro_rate_calibrate_count = 100
        for _ in range(gyro_rate_calibrate_count):
            target = target + self._read_fd(self.color_reflection_fd)
            time.sleep(0.01)
        target = target / gyro_rate_calibrate_count
        #目標値を保管
        self.refrection_target = target

    def line_tracing(self):
        u""" 速度、旋回値をpwm値として返却 """
        #センサー値を取得
        refrection_raw = self._read_fd(self.color_reflection_fd)

        #指示値を取得
        direction = self._calc_direction(refrection_raw)
        speed = 50 #固定値

        return speed, direction

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
        self.i_b = i_n

        return direction

    # Function for fast reading from sensor files
    @staticmethod
    def _read_fd(infile):
        infile.seek(0)
        return int(infile.read().decode().strip())
