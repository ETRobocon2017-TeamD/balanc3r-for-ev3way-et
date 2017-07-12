from ev3dev.auto import *
from ev3dev.helper import Tank


class LineTracer:
    u"""PID制御によるライントレース
    キャリブレーション後のカラーセンサー値を目標値として旋回量を制御
    
    現時点では前進速度は固定値を想定
    →走行速度によって係数を調整する必要があるため
    """

    def __init__(self):
        # target_v 目標値
        # delta_t 周回時間
        self.target = 0.0
        self.delta_t = 0.015 # 10msec
        self.k_p = 0.3  # P係数(要調整)
        self.k_i = 0.1  # I係数(要調整)
        self.k_d = 0.1  # D係数(要調整)
        self.e_b = 0.0  # 前回偏差
        self.i_b = 0.0  # 前回までの積分値
    
        self.color = ColorSensor()
        self.color.mode = self.color.MODE_REF_RAW #raw値
        self.colorReflectionRaw = open(self.color._path + "/value0", "rb")

    
    def calibrate_color_sensor
        colorTarget = 0
        gyroRateCalibrateCount = 100
        for i in range(gyroRateCalibrateCount):
            colorTarget = colorTarget + self.FastRead(self.colorReflectionRaw)
            time.sleep(0.01)
        colorTarget = colorTarget/gyroRateCalibrateCount
        self.set_target(colorTarget) #目標値を保管

    def set_target(self, target_v):
        self.target = target_v


    def line_tracing(self)
        u""" 速度、旋回値をpwm値として返却 """
        #センサー値を取得
        colorRaw = self.FastRead(self.colorReflectionRaw)

        #指示値を取得
        direction = self.calc_direction(self, colorRaw)
        speed = 10 #固定値
        
        return speed, direction

    def calc_direction(self, brightness):
        u"""旋回量取得"""
        # e_n 偏差
        # p_n P制御
        # i_n I制御
        # d_n D制御

        # pid演算
        e_n = self.target - brightness 
        p_n = self.k_p * e_n
        i_n = self.k_i * (self.i_b + ((e_n + self.e_b) / 2.0 * self.delta_t))
        d_n = self.k_d * (e_n - self.e_b) / self.delta_t
        direction = (p_n + i_n + d_n)

        # -100 ～100に収める
        if direction > 100:
            direction = 100
        elif direction < -100:
            direction = -100

        # 値の保存
        self.e_b = e_n
        self.i_b = i_n
        
        return direction

    def shutdown
        self.colorReflectionRaw.close()

    # Function for fast reading from sensor files
    def FastRead(infile):
        infile.seek(0)
        return int(infile.read().decode().strip())
