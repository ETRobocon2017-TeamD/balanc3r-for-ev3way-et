# coding:utf-8
import time
from math import *

class Odometry:

    def __init__(self):
        self.distance = 0.0  # 走行距離
        self.distance_periodic_L = 0.0  # 左タイヤの4ms間の距離
        self.distance_periodic_R = 0.0  # 右タイヤの4ms間の距離
        self.pre_angleL = 0.0
        self.pre_angleR = 0.0  # 左右モータ回転角度の過去値
        self.pre_pos_x = 0
        self.pre_pos_y = 0

        self.total_direction = 0.0 #現在の角度
        self.total_distance = 0.0 #現在の距離

        self.pre_direction_pwm = 0.0 # 前回の旋回値
        self.pre_speed_pwm = 0.0 #前回のspeed

        # self.grid_distance = 0.0 # 現在座標から目標座標までの距離
        # self.grid_direction = 0.0 # 現在座標から目標座標の方位

        self.TREAD = 132.6 # 車体トレッド幅(132.6mm)
        self.PI = 3.14159265358
        self.TIRE_DIAMETER = 81.0  #タイヤ直径（81mm）

        #調整用パラメータ
        self.turning_angle = 1 #旋回時のPWM上昇値 1ループごとに加算(減算)する
        self.target_area_x = 200 #目標地点の到達判定領域 x軸(mm)
        self.target_area_y = 200 #目標地点の到達判定領域 y軸(mm)
        self.run_speed = 40 #PWM値ロボットの進行速度


        # 目標地点の設定 (mm)
        self.target_pos = [[0 for i in range(2)] for j in range(10)]
        self.cur_target_index =0
        self.set_target(1000,0)
        self.set_target(1000,-1000)
        self.set_target(0,-1000)
        self.set_target(0,0)
        self.set_target(1000,0)
        self.set_target(1000,-1000)
        self.set_target(0,-1000)
        self.set_target(0,0)

        self.cur_target_index =0

        self.odmetry_logs = ["" for _ in range(10000)]
        self.odmetry_log_pointer = 0


        # # 目標座標までの方位，距離を格納
        # self.Grid_setDistance(0, 0, self.target_pos[0][self.cur_target_index], self.target_pos[1][self.cur_target_index])
        # self.Grid_setDirection(0, 0, self.target_pos[0][self.cur_target_index], self.target_pos[1][self.cur_target_index])
        #
        # self.target_dis = self.Grid_getDistance()
        # self.target_dir = self.Grid_getDirection()


    #自己の座標
    #ターゲットの座標

    def set_target (self, x, y):
        self.target_pos[self.cur_target_index][0] = x
        self.target_pos[self.cur_target_index][1] = y

        self.cur_target_index += 1

    # 目標地点へ進行させる
    # left_motor  左モータ回転角度の現在値
    # right_motor 右モータ回転角度の現在値
    # TODO: direction positionの正負について検討すること
    def target_trace(self, left_angle, right_angle):
        direction = 0.0
        speed =0.0


        # 前回計測時点との差分を取得
        cur_dis = self.get_distance(left_angle, right_angle)
        cur_dir = self.get_direction()


        # 現在の位置を計算
        pos_x = self.pre_pos_x + (cur_dis * cos(radians(cur_dir))) #進行距離 * cos x
        pos_y = self.pre_pos_y + (cur_dis * sin(radians(cur_dir))) #進行距離 * sin x

        self.pre_pos_x = pos_x
        self.pre_pos_y = pos_y


        # 目標座標までの方位，距離を格納
        target_dis = self.get_target_distance(pos_x, pos_y, self.target_pos[0][self.cur_target_index], self.target_pos[1][self.cur_target_index])
        target_dir = self.get_target_direction(pos_x, pos_y, self.target_pos[0][self.cur_target_index], self.target_pos[1][self.cur_target_index])


        #targetとの差分から速度と角度を調整
        #角度の差に比例すべき？
        if target_dir < 0 :
            direction = self.pre_direction_pwm + self.turning_angle
        elif target_dir == 0:
            direction = 0
        else:
            direction = self.pre_direction_pwm - self.turning_angle

        #TODO:距離に比例してスピードを出すべきか検討
        #計測してから
        speed = self.run_speed

        #前回値として保管
        self.pre_direction_pwm = direction
        self.pre_speed_pwm = speed

        #目標に到達していたらindexを進める
        #TODO:目標近傍の閾値について検討
        if abs(self.target_pos[0][self.cur_target_index] - pos_x) < self.target_area_x and abs(self.target_pos[1][self.cur_target_index] - pos_y) < self.target_area_y:
            cur_target_index +=1


        #log
        self.odmetry_logs[self.odmetry_log_pointer] = "{},{},{},{},{},{},{},{},{},{}".format(
            left_angle,
            right_angle,
            cur_dis,
            cur_dir,
            pos_x,
            pos_y,
            target_dis,
            target_dir,
            self.total_distance,
            self.total_direction)
        self.odmetry_log_pointer += 1

        return speed, direction

    # 距離 計測
    # left_motor  左モータ回転角度の現在値
    # right_motor 右モータ回転角度の現在値
    def get_distance(self, left_motor, right_motor):
        cur_angleL = left_motor
        cur_angleR = right_motor
        distance = 0.0 # 前回との距離

        # 計測間の走行距離 = ((円周率 * タイヤの直径) / 360) * (モータ角度過去値　- モータ角度現在値)
        self.distance_periodic_L = ((self.PI * self.TIRE_DIAMETER) / 360.0) * (cur_angleL - self.pre_angleL) # 計測間の左モータ距離
        self.distance_periodic_R = ((self.PI * self.TIRE_DIAMETER) / 360.0) * (cur_angleR - self.pre_angleR) # 計測間の右モータ距離
        distance = (self.distance_periodic_L + self.distance_periodic_R) / 2.0 # 左右タイヤの走行距離を足して割る
        self.total_distance += distance

        # モータの回転角度の過去値を更新
        self.pre_angleL = cur_angleL
        self.pre_angleR = cur_angleR

        return distance


    #/ *方位を取得(右旋回が正転) * /
    def get_direction(self):
        # (360 / (2 * 円周率 * 車体トレッド幅)) * (右進行距離 - 左進行距離)
        direction = (360.0 / (2.0 * self.PI * self.TREAD)) * ( self.distance_periodic_R - self.distance_periodic_L)
        self.total_direction += direction
        return direction


    #/* 座標aから座標bまでの移動距離を取得する関数 */
    # aX, aY 現在地
    # bX, bY 目標値
    def get_target_distance(self, aX, aY, bX, bY) :
        target_distance = sqrt( pow((bX-aX),2) + pow((bY-aY),2) )
        return target_distance


    #/* 目標座標の方位を取得する関数 */
    # aX, aY 現在地
    # bX, bY 目標値
    def get_target_direction(self,  aX, aY, bX, bY) :
        target_dir = 0.0 # 目標方位

        #//　座標aから座標bへの方位（ラジアン）を取得
        target_dir = atan2((bY-aY), (bX-aX))
        #//ラジアンから度に変換
        target_dir = target_dir * 180.0 / self.PI
        return target_dir

    def shutdown(self, log_datetime):
        log_file = open("./log/log_odometry_{}.csv".format(log_datetime), 'w')
        for log in self.odmetry_logs:
            if log != "":
                log_file.write("{}\n".format(log))
        log_file.close()
