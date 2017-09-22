# coding:utf-8
import time
from math import sin, cos, sqrt, pow, pi, atan2, radians

class Odometry:

    def __init__(self):
        self.__distance_periodic_L = 0.0  # 左タイヤの4ms間の距離
        self.__distance_periodic_R = 0.0  # 右タイヤの4ms間の距離
        self.__pre_angle_L = 0.0
        self.__pre_angle_R = 0.0  # 左右モータ回転角度の過去値
        self.pre_pos_x = 0
        self.pre_pos_y = 0

        self.__total_direction = 0.0 #現在の角度
        self.__total_distance = 0.0 #現在の距離

        self.__TREAD = 132.6 # 車体トレッド幅(132.6mm)
        self.__TIRE_DIAMETER = 81.0  #タイヤ直径（81mm）

        #調整用パラメータ
        self.__TARGET_AREA_X = 100 #目標地点の到達判定領域 x軸(mm)
        self.__TARGET_AREA_Y = 100 #目標地点の到達判定領域 y軸(mm)
        self.__RUN_SPEED = 30 #PWM値ロボットの進行速度


        # 目標地点の設定 (mm)
        self.__target_pos = [[0 for i in range(2)] for j in range(10)]
        self.__cur_target_index = 0
        self.__set_target(500, 0)
        self.__set_target(500, -500)
        self.__set_target(0, -500)
        self.__set_target(0, 0)
        self.__set_target(500, 0)
        self.__set_target(500, -500)
        self.__set_target(0, -500)
        self.__set_target(0, 0)

        self.__cur_target_index = 0

    # 目標地点へ進行させる
    # left_motor  左モータ回転角度の現在値
    # right_motor 右モータ回転角度の現在値
    # TODO: direction positionの正負について検討すること
    def target_trace(self, left_angle, right_angle):
        direction = 0.0
        speed = 0.0

        # 前回計測時点との差分を取得
        cur_dis = self.__get_distance(left_angle, right_angle)        
        self.__total_distance += cur_dis

        cur_dir = self.__get_direction()
        self.__total_direction += cur_dir

        # 現在の位置を計算
        pos_x = self.pre_pos_x + (cur_dis * cos(radians(self.__total_direction))) #進行距離 * cos x
        pos_y = self.pre_pos_y + (cur_dis * sin(radians(self.__total_direction))) #進行距離 * sin x

        self.pre_pos_x = pos_x
        self.pre_pos_y = pos_y

        # 目標座標までの方位，距離を格納
        target_pos_x = self.__target_pos[self.__cur_target_index][0]
        target_pos_y = self.__target_pos[self.__cur_target_index][1]
        target_dis = self.__get_target_distance(pos_x, pos_y, target_pos_x, target_pos_y)
        target_dir = self.__get_target_direction(pos_x, pos_y, target_pos_x, target_pos_y)

        #targetとの差分から速度と角度を調整
        diff_dir = target_dir - self.__total_direction
        direction = -diff_dir
        if direction < -100:
            direction = -100
        elif direction > 100:
            direction = 100

        #TODO:距離に比例してスピードを出すべきか検討
        #計測してから
        speed = self.__RUN_SPEED

        #目標に到達していたらindexを進める
        #TODO:目標近傍の閾値について検討
        if abs(target_pos_x - pos_x) < self.__TARGET_AREA_X and abs(target_pos_y - pos_y) < self.__TARGET_AREA_Y:
            self.__cur_target_index += 1

        return speed, direction

    #自己の座標
    #ターゲットの座標
    def __set_target(self, x, y):
        self.__target_pos[self.__cur_target_index][0] = x
        self.__target_pos[self.__cur_target_index][1] = y

        self.__cur_target_index += 1

    # 距離 計測
    # left_motor  左モータ回転角度の現在値
    # right_motor 右モータ回転角度の現在値
    def __get_distance(self, cur_angle_L, cur_angle_R):
        distance = 0.0 # 前回との距離

        # 計測間の走行距離 = ((円周率 * タイヤの直径) / 360) * (モータ角度過去値　- モータ角度現在値)
        self.__distance_periodic_L = ((pi * self.__TIRE_DIAMETER) / 360.0) * (cur_angle_L - self.__pre_angle_L) # 計測間の左モータ距離
        self.__distance_periodic_R = ((pi * self.__TIRE_DIAMETER) / 360.0) * (cur_angle_R - self.__pre_angle_R) # 計測間の右モータ距離
        distance = (self.__distance_periodic_L + self.__distance_periodic_R) / 2.0 # 左右タイヤの走行距離を足して割る

        # モータの回転角度の過去値を更新
        self.__pre_angle_L = cur_angle_L
        self.__pre_angle_R = cur_angle_R

        return distance


    #/ *方位を取得(右旋回が正転) * /
    def __get_direction(self):
        # (360 / (2 * 円周率 * 車体トレッド幅)) * (右進行距離 - 左進行距離)
        return (360.0 / (2.0 * pi * self.__TREAD)) * (self.__distance_periodic_R - self.__distance_periodic_L)


    #/* 座標aから座標bまでの移動距離を取得する関数 */
    # aX, aY 現在地
    # bX, bY 目標値
    @staticmethod
    def __get_target_distance(aX, aY, bX, bY):
        return sqrt(pow((bX - aX), 2) + pow((bY - aY), 2))

    #/* 目標座標の方位を取得する関数 */
    # aX, aY 現在地
    # bX, bY 目標値
    @staticmethod
    def __get_target_direction(aX, aY, bX, bY) :
        target_dir = 0.0 # 目標方位

        #//　座標aから座標bへの方位（ラジアン）を取得
        target_dir = atan2( (bY - aY), (bX - aX) )
        #//ラジアンから度に変換
        target_dir = target_dir * 180.0 / pi
        return target_dir

