import os
import sys
import mmap
import time
import signal
import logging
import math
import subprocess
import re
from collections import deque
from ev3dev.auto import *
from ev3dev.helper import Tank
from guide import LineTracer
# import runner, guide

log = logging.getLogger(__name__)

########################################################################
##
## File I/O functions
##
########################################################################

# Function for fast reading from sensor files
def read_device(fd):
    fd.seek(0)
    return int(fd.read().decode().strip())

# Function for fast writing to motor files
def write_device(fd, value):
    fd.truncate(0)
    fd.write(str(int(value)))
    fd.flush()

########################################################################
##
## mmapオブジェクト読み取り・書き出し関数
##
########################################################################

MMAP_SIZE = 18 # マップするバイト数

# mmapオブジェクト読み取り用関数
def read_anon_mem(memfd):
    memfd.seek(0)
    bts = memfd.read()
    length = bts[0]
    idx_end = 1 + length
    return int.from_bytes(bts[1:idx_end], byteorder='little', signed=True)

# mmapオブジェクト書き込み用関数
def write_anon_mem(memfd, value):
    memfd.seek(0)
    val_length = value.bit_length()
    bts = bytearray(1 + val_length)
    bts[0] = val_length
    bts[1:] = value.to_bytes(val_length, byteorder='little', signed=True)
    memfd.write(bytes(bts))

# Function to set the duty cycle of the motors
def set_duty(motor_duty_devfd, duty, barance=1):
    # Clamp the value between -100 and 100
    duty = min(max(duty, -100), 100) * barance
    # Apply the signal to the motor
    write_device(motor_duty_devfd, duty)
    return duty

########################################################################
##
## プロセス間で各値を共有するための関数群
##
########################################################################

# 左右モーターの値をmmapで共有
motor_encoder_left_memfd = mmap.mmap(-1, MMAP_SIZE)
write_anon_mem(motor_encoder_left_memfd, 0)
motor_encoder_left_memfd = mmap.mmap(-1, MMAP_SIZE)
write_anon_mem(motor_encoder_left_memfd, 0)

def read_motor_encoder_left_mem():
    return read_anon_mem(motor_encoder_left_memfd)

def read_motor_encoder_right_mem():
    return read_anon_mem(motor_encoder_left_memfd)

def write_motor_encoder_left_mem(value):
    write_anon_mem(motor_encoder_left_memfd, value)

def write_motor_encoder_right_mem(value):
    write_anon_mem(motor_encoder_left_memfd, value)

# タッチセンサーの値をmmapで共有
touch_sensor_memfd = mmap.mmap(-1, MMAP_SIZE)
write_anon_mem(touch_sensor_memfd, 0)

def read_touch_sensor_mem():
    return read_anon_mem(touch_sensor_memfd)

def write_touch_sensor_mem(value):
    write_anon_mem(touch_sensor_memfd, value)

# 前進後退スピード、旋回スピードの値をmmapで共有
speed_memfd = mmap.mmap(-1, MMAP_SIZE)
write_anon_mem(speed_memfd, 0)

def read_speed_mem():
    return read_anon_mem(speed_memfd)

def write_speed_mem(value):
    write_anon_mem(speed_memfd, value)

steering_memfd = mmap.mmap(-1, MMAP_SIZE)
write_anon_mem(steering_memfd, 0)

def read_steering_mem():
    return read_anon_mem(steering_memfd)

def write_steering_mem(value):
    write_anon_mem(steering_memfd, value)

########################################################################
##
## Guide：ラインの状態を感知して、前進後退速度と旋回速度を算出する関数
##
########################################################################
def guide():
    print('Im Guide')

    # ここで変数定義などの事前準備を行う
    # Time of each loop, measured in miliseconds.
    loop_time_millisec = 25
    # Time of each loop, measured in seconds.
    loop_time_sec = loop_time_millisec / 1000.0

    print('Calibrate ColorSensor ...')
    line_tracer = LineTracer()
    line_tracer.calibrate_color_sensor()

    # タッチセンサー押し待ち
    print('Guide Waiting ...')
    while not read_touch_sensor_mem():
        time.sleep(0.1)

    speed_reference = 0
    direction = 0

    while True:
        ###############################################################
        ##  Loop info
        ###############################################################
        t_loop_start = time.clock()

        # ここでライントレースする
        speed_reference, direction = line_tracer.line_tracing()

        # 左右モーターの角度は下記のように取得
        # print(read_motor_encoder_left_mem())
        # print(read_motor_encoder_right_mem())

        # 前進後退・旋回スピードは下記のように入力
        write_speed_mem(speed_reference)
        write_steering_mem(int(round(direction)))

        ###############################################################
        ##  Busy wait for the loop to complete
        ###############################################################
        # while ((time.clock() - t_loop_start) < loop_time_sec):
        #     time.sleep(0.0001)
        time.sleep(loop_time_sec - (time.clock() - t_loop_start))

########################################################################
##
## Runner：倒立振子の制御とDuty比の入力を行う関数
##
########################################################################
def runner():
    print('Im Runner')

    def shutdown_child(signum=None, frame=None):
        left_motor.stop()
        right_motor.stop()

        time.sleep(0.2)

        gyro_sensor_devfd.close()
        battery_voltage_devfd.close()
        motor_encoder_left_devfd.close()
        motor_encoder_right_devfd.close()
        motor_duty_cycle_left_devfd.close()
        motor_duty_cycle_right_devfd.close()

        log_file = open('./log/log_runner_%s.txt' % time.time(),'w')
        for log_ in logs:
            if log_ != "":
                log_file.write("%s\n" % log_)
        log_file.close()

        sys.exit()

    signal.signal(signal.SIGTERM, shutdown_child)

    try:
        # Sensor setup
        gyro = GyroSensor()
        gyro.mode = gyro.MODE_GYRO_RATE

        # Battery setup
        battery = PowerSupply()

        # Motor setup
        left_motor = LargeMotor('outC')
        right_motor = LargeMotor('outB')
        left_motor.reset()
        right_motor.reset()
        left_motor.run_direct()
        right_motor.run_direct()
        #しっぽモーター
        tail_motor = Motor('outA')

        ########################################################################
        ## Definitions and Initialization variables
        ########################################################################

        # Timing settings for the program
        ## Time of each loop, measured in miliseconds.
        loop_time_millisec = 25
        ## Time of each loop, measured in seconds.
        loop_time_sec      = loop_time_millisec / 1000.0

        # Math constants
        ## The number of radians in a degree.
        radians_per_degree = math.pi / 180

        # Platform specific constants and conversions
        deg_per_sec_per_raw_gyro_unit    = 1                                                  # For the LEGO EV3 Gyro in Rate mode, 1 unit = 1 deg/s
        rad_per_second_per_raw_gyro_unit = deg_per_sec_per_raw_gyro_unit * radians_per_degree # Express the above as the rate in rad/s per gyro unit
        deg_per_raw_motor_unit           = 1                                                  # For the LEGO EV3 Large Motor 1 unit = 1 deg
        radians_per_raw_motor_unit       = deg_per_raw_motor_unit*radians_per_degree          # Express the above as the angle in rad per motor unit
        rpmper_per_percent_speed         = 1.7                                                # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled) EV3では、「speed」を1%とした場合、１分間に1.7回転させる速さであると定義する（※電圧によって変わる）
        deg_per_sec_per_percent_speed    = rpmper_per_percent_speed * 360 / 60                # Convert this number to the speed in deg/s per "percent speed" 「speed」を回転角速度(deg)に変換する係数
        rad_per_sec_per_percent_speed    = deg_per_sec_per_percent_speed * radians_per_degree # Convert this number to the speed in rad/s per "percent speed" 「speed」を回転角速度(rad)に変換する係数

        # The rate at which we'll update the gyro offset (precise definition given in docs)
        # ジャイロ値を補正するオフセット値の更新に使用する。調節する必要がある。
        gyro_drift_compensation_rate = 0.075 * loop_time_sec * rad_per_second_per_raw_gyro_unit

        # State feedback control gains (aka the magic numbers)
        gain_motor_angle                   = 0.1606 * 3 * 0.85    # K_F[0]
        gain_gyro_angle                    = 30.2153 * 2.5 * 0.85 # K_F[1]
        gain_motor_angular_speed           = 1.0796 * 1.7 * 0.85  # K_F[2]
        gain_gyro_rate                     = 3.3269 * 2 * 0.85    # K_F[3]
        gain_motor_angle_error_accumulated = 0.4472 * 0.85        # K_I

        battery_gain = 0.001089  # PWM出力算出用バッテリ電圧補正係数
        battery_offset = 0.625  # PWM出力算出用バッテリ電圧補正オフセット

        a_d = 1.0 - 0.55 #0.51 #0.47  # ローパスフィルタ係数(左右車輪の平均回転角度用)。左右モーターの平均回転角速度(rad/sec)の算出時にのみ使用する。小さいほど角速度の変化に過敏になる。〜0.4951
        a_r = 0.985 #0.98  # ローパスフィルタ係数(左右車輪の目標平均回転角度用)。左右モーターの目標平均回転角度(rad)の算出時に使用する。小さいほど前進・後退する反応が早くなる。
        a_b = 0.85 #ローパスフィルタ係数(最大モーター電圧b用）
        k_theta_dot = 7.5 # モータ目標回転角速度係数

        # Variables representing physical signals (more info on these in the docs)
        # The angle of "the motor", measured in raw units (degrees for the
        # EV3). We will take the average of both motor positions as "the motor"
        # angle, wich is essentially how far the middle of the robot has traveled.
        motor_angle_raw = 0

        # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
        motor_angle = 0

        # 1ループ前に算出した左右車輪回転角度
        motor_angle_last = 0

        # The reference angle of the motor. The robot will attempt to drive
        # forward or backward, such that its measured position equals this
        # reference (or close enough).
        motor_angle_reference = 0

        # The error: the deviation of the measured motor angle from the reference.
        # The robot attempts to make this zero, by driving toward the reference.
        motor_angle_error = 0

        # We add up all of the motor angle error in time. If this value gets out of
        # hand, we can use it to drive the robot back to the reference position a bit quicker.
        motor_angle_error_accumulated = 0

        # The motor speed, estimated by how far the motor has turned in a given amount of time
        motor_angular_speed = 0

        # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
        motor_angular_speed_reference = 0

        # The error: the deviation of the motor speed from the reference speed.
        motor_angular_speed_error = 0

        # The 'voltage' signal we send to the motor. We calulate a new value each
        # time, just right to keep the robot upright.
        motor_duty_cycle = 0

        # The raw value from the gyro sensor in rate mode.
        gyro_rate_raw = 0

        # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
        gyro_rate = 0

        # 現在のバッテリー電圧
        voltage_raw = 0

        # The gyro doesn't measure the angle of the robot, but we can estimate
        # this angle by keeping track of the gyro_rate value in time
        gyro_estimated_angle = 0

        # Over time, the gyro rate value can drift. This causes the sensor to think
        # it is moving even when it is perfectly still. We keep track of this offset.
        gyro_offset = 0

        # ログ記録用
        logs = ["" for _ in range(10000)]
        log_pointer = 0

        voltage_target = 0
        voltage_estimate_max = 0

        # filehandles for fast reads/writes
        # =================================
        gyro_sensor_devfd = open(gyro._path + "/value0", "rb")
        battery_voltage_devfd = open(battery._path + "/voltage_now", "rb")

        # Open motor files for (fast) reading
        motor_encoder_left_devfd = open(left_motor._path + "/position", "rb")
        motor_encoder_right_devfd = open(right_motor._path + "/position", "rb")

        # Open motor files for (fast) writing
        motor_duty_cycle_left_devfd = open(left_motor._path + "/duty_cycle_sp", "w")
        motor_duty_cycle_right_devfd = open(right_motor._path + "/duty_cycle_sp", "w")

        # バッテリー電圧
        voltage_raw = read_device(battery_voltage_devfd) # 単位(μV)
        # 現在のバッテリー電圧をもとに、PWMデューティ値を100%にしたとき、モータが受け取る推定電圧を求める。
        # 6.2 アクチュエータ　(6.1)式 バッテリ電圧とモータ回転速度(rpm)の関係式
        voltage_estimate_max = (battery_gain * voltage_raw / 1000) - battery_offset

        ########################################################################
        ## Calibrate Gyro
        ########################################################################
        print("-----------------------------------")
        print("Calibrating...")

        #As you hold the robot still, determine the average sensor value of 100 samples
        gyro_rate_calibrate_count = 200
        for _ in range(gyro_rate_calibrate_count):
            gyro_rate_raw = read_device(gyro_sensor_devfd)
            gyro_offset = gyro_offset + gyro_rate_raw
            time.sleep(0.01)
        gyro_offset = gyro_offset / gyro_rate_calibrate_count

        # Print the result
        print("GyroOffset: %s" % gyro_offset)

        speed_reference = read_speed_mem()
        steering = read_steering_mem()

        #speed_reference = 62.5 # 前進・後退速度。62.5〜-62.5の間で入力
        #steering = -1 * self.STEER_SPEED * 0.5 # 旋回速度。他の係数をいじったせいか、今の値でも少々不安定になっている。もう少し下げたほうがいいかも

        ########################################################################
        ## タッチセンサー押し待ち
        ########################################################################
        print('Runner Waiting ...')
        while not read_touch_sensor_mem():
            time.sleep(0.025)

        print("-----------------------------------")
        print("GO!")
        print("-----------------------------------")

        tail_motor.run_timed(time_sp=250, speed_sp=-300) # しっぽモーター上に上げる

        # 倒立振子スタート時の時間取得
        t_balancer_start = time.clock()

        while True:

            ###############################################################
            ##  Loop info
            ###############################################################
            t_loop_start = time.clock()

            ###############################################################
            ##  Reading the Gyro.
            ###############################################################
            gyro_rate_raw = read_device(gyro_sensor_devfd)
            gyro_rate = (gyro_rate_raw - gyro_offset) * rad_per_second_per_raw_gyro_unit # 躯体の角速度(rad/sec)。ジャイロから得た角速度をオフセット値で調整している

            ###############################################################
            ##  Reading the Motor Position
            ###############################################################
            motor_angle_last = motor_angle
            motor_angle_raw = (read_device(motor_encoder_left_devfd) + read_device(motor_encoder_right_devfd)) * 0.5
            motor_angle = (motor_angle_raw * radians_per_raw_motor_unit) + gyro_estimated_angle # 左右モーターの現在の平均回転角度(rad) + 躯体の(推定)回転角度

            speed_reference = read_speed_mem()

            #motor_angular_speed_reference = speed_reference * rad_per_sec_per_percent_speed # 左右モーターの目標平均回転角速度(rad/sec)。入力値speed_referenceを角速度(rad)に変換したもの。
            # K_THETA_DOT(7.5): 最大モーター角速度だと思われる値。 speed_reference(モータ最大角速度を100%とする、目標割合)にかけ合わせて
            motor_angular_speed_reference = ((1.0 - a_r) * ((speed_reference / 100.0) * k_theta_dot)) + (a_r * motor_angular_speed_reference)
            motor_angle_reference = motor_angle_reference + (motor_angular_speed_reference * loop_time_sec) # 左右モーターの目標平均回転角度(rad)。初期値は0になる。入力値speed_referenceがずっと0でも0になる

            motor_angle_error = motor_angle - motor_angle_reference # 左右モーターの現在の平均回転角度と目標平均回転角度との誤差(rad)

            ###############################################################
            ##  Computing Motor Speed
            ###############################################################
            motor_angular_speed = (((1.0 - a_d) * motor_angle) + (a_d * motor_angle_last) - motor_angle_last) / loop_time_sec # 左右モーターの平均回転角速度(rad/sec)。左右モーターの現在平均回転角度をローパスフィルターに通して、前回の平均回転角度との差分を周期で割っている。
            motor_angular_speed_error = motor_angular_speed - motor_angular_speed_reference # 左右モーターの現在の平均回転角速度と目標平均回転角速度との誤差(rad/sec)

            ###############################################################
            ##  Reading the Voltage.
            ###############################################################
            voltage_raw = read_device(battery_voltage_devfd) #バッテリー電圧(μV)

            ###############################################################
            ##  Computing the motor duty cycle value
            ###############################################################
            voltage_target = ((gain_gyro_angle  * gyro_estimated_angle)
               + (gain_gyro_rate   * gyro_rate)
               + (gain_motor_angle * motor_angle_error)
               + (gain_motor_angular_speed * motor_angular_speed_error)
               + (gain_motor_angle_error_accumulated * motor_angle_error_accumulated))
            voltage_estimate_max = (battery_gain * voltage_raw / 1000) - battery_offset
            motor_duty_cycle = (voltage_target / voltage_estimate_max) * 100

            ###############################################################
            ##  Apply the signal to the motor, and add steering
            ###############################################################
            steering = read_steering_mem()
            set_duty(motor_duty_cycle_right_devfd, motor_duty_cycle + steering)
            duty = set_duty(motor_duty_cycle_left_devfd, motor_duty_cycle - steering, 0.975) # 右車輪のモーター出力が弱いので、左車輪のPWM値を3つ目の引数で調節(%)してる。まだ偏ってるので調節必要

            ###############################################################
            ##  Update angle estimate and Gyro Offset Estimate
            ###############################################################
            gyro_estimated_angle = gyro_estimated_angle + (gyro_rate * loop_time_sec) # 次回の躯体の（推定）回転角度(rad)
            gyro_offset = ((1 - gyro_drift_compensation_rate) * gyro_offset) + (gyro_drift_compensation_rate * gyro_rate_raw) # ジャイロの角速度を補正するオフセット値(rad/sec)の更新。 現状gyro_drift_compensation_rateが極小なので、ほぼほぼ前回算出したオフセット値寄りになる

            ###############################################################
            ##  Update Accumulated Motor Error
            ###############################################################
            motor_angle_error_accumulated = motor_angle_error_accumulated + (motor_angle_error * loop_time_sec) # モーター角度誤差累積(rad*t) もしかして積分？ 前回の累積に、今回の目標平均回転角度との誤差を周期でかけたものを足している

            # 実行時間、PWM値(duty cycle value)に関わる値をログに出力
            t_loop_end = time.clock()
            logs[log_pointer] = "{}   {}   {}   {}   {}   {}   {}   {}   {}   {}   {}".format(
                t_loop_end - t_balancer_start,
                t_loop_end - t_loop_start,
                gyro_rate_raw,
                motor_angle_raw,
                gyro_estimated_angle,
                gyro_rate,
                motor_angle_error,
                motor_angular_speed_error,
                motor_angle_error_accumulated,
                duty,
                voltage_raw)
            log_pointer += 1

            ###############################################################
            ##  Busy wait for the loop to complete
            ###############################################################
            # while ((time.clock() - t_loop_start) < (loop_time_sec - 0.011)): # clock()の値にはsleep中の経過時間が含まれないので、このwhileの条件文の算出時間をsleep代わりにしている(算出時間はバラバラ…)
                # time.sleep(0.0001)
            time.sleep(max(loop_time_sec - (gyro_rate*0.0005) - (time.clock() - t_loop_start), 0.002))

    except (KeyboardInterrupt, Exception) as e:
        log.exception(e)
        shutdown()

########################################################################
##
## Runner_stub：Runnerのスタブ
## ライントレーサー開発等で倒立振子ライブラリを使いたくない場合はrunner_stub()を使用すること
##
########################################################################
def runner_stub():
    print('Im Runner Stub')

    def shutdown_child(signum=None, frame=None):
        motor_encoder_left_devfd.close()
        motor_encoder_right_devfd.close()
        motor_duty_cycle_left_devfd.close()
        motor_duty_cycle_right_devfd.close()

        left_motor.stop()
        right_motor.stop()

        sys.exit()

    signal.signal(signal.SIGTERM, shutdown_child)

    try:
        # Motor setup
        left_motor = LargeMotor('outC')
        right_motor = LargeMotor('outB')
        left_motor.reset()
        right_motor.reset()
        left_motor.run_direct()
        right_motor.run_direct()
         #しっぽモーター
        tail_motor = Motor('outA')

        ########################################################################
        ## Definitions and Initialization variables
        ########################################################################

        # Timing settings for the program
        # Time of each loop, measured in miliseconds.
        loop_time_millisec = 100
        # Time of each loop, measured in seconds.
        loop_time_sec = loop_time_millisec / 1000.0

        motor_angle_raw_left = 0
        motor_angle_raw_right = 0

        # filehandles for fast reads/writes
        # =================================

        # Open motor files for (fast) reading
        motor_encoder_left_devfd = open(left_motor._path + "/position", "rb")
        motor_encoder_right_devfd = open(right_motor._path + "/position", "rb")

        # Open motor files for (fast) writing
        motor_duty_cycle_left_devfd = open(left_motor._path + "/duty_cycle_sp", "w")
        motor_duty_cycle_right_devfd = open(right_motor._path + "/duty_cycle_sp", "w")

        speed_reference = read_speed_mem()
        steering = read_steering_mem()

        ########################################################################
        ## タッチセンサー押し待ち
        ########################################################################
        print('Runner Waiting ...')
        while not read_touch_sensor_mem():
            time.sleep(0.025)

        print("-----------------------------------")
        print("GO!")
        print("-----------------------------------")

        tail_motor.run_timed(time_sp=250, speed_sp=-300) # しっぽモーター上に上げる

        # 倒立振子スタート時の時間取得
        t_balancer_start = time.clock()

        while True:

            ###############################################################
            ##  Loop info
            ###############################################################
            t_loop_start = time.clock()

            ###############################################################
            ##  Reading the Motor Position
            ###############################################################

            motor_angle_raw_left = read_device(motor_encoder_left_devfd)
            motor_angle_raw_right = read_device(motor_encoder_right_devfd)
            write_motor_encoder_left_mem(motor_angle_raw_left)
            write_motor_encoder_right_mem(motor_angle_raw_right)

            speed_reference = read_speed_mem()

            ###############################################################
            ##  Apply the signal to the motor, and add steering
            ###############################################################
            steering = read_steering_mem()
            set_duty(motor_duty_cycle_right_devfd, speed_reference + steering)
            duty = set_duty(motor_duty_cycle_left_devfd, speed_reference - steering, 0.975)

            ###############################################################
            ##  Busy wait for the loop to complete
            ###############################################################
            # clock()の値にはsleep中の経過時間が含まれないので、このwhileの条件文の算出時間をsleep代わりにしている(算出時間はバラバラ…)
            time.sleep(loop_time_sec - (time.clock() - t_loop_start))

    except (KeyboardInterrupt, Exception) as ex:
        log.exception(ex)
        shutdown()


########################################################################
##
## メイン：Guide関数とRunner関数用の子プロセスをフォークする
##
########################################################################
if __name__ == '__main__':

    def shutdown():
        if ('guide_pid' in globals()) or ('guide_pid' in locals()):
            os.kill(guide_pid, signal.SIGKILL)
        if ('runner_pid' in globals()) or ('runner_pid' in locals()):
            os.kill(runner_pid, signal.SIGTERM)
        touch_sensor_devfd.close()
        print('Done')

    try:
        # touchSensorの読み込み
        touch = TouchSensor()
        touch_sensor_devfd = open(touch._path + "/value0", "rb")
        touch_sensor_pressed = read_device(touch_sensor_devfd)

        # TODO: ガイドが提示する前進後退・旋回スピードもmmapで共有する

        runner_pid = os.fork()

        if runner_pid == 0:
            # NOTE: 倒立振子ライブラリを使う場合はrunner()を、ライントレーサー開発等で倒立振子ライブラリを使いたくない場合はrunner_stub()を使用すること
            runner()
            print('Runner Done')
            sys.exit()

        guide_pid = os.fork()

        if guide_pid == 0:  # In a child process
            guide()
            # write_speed_mem(50)
            # write_steering_mem(50)
            print('Guide Done')
            sys.exit()

        time.sleep(0.1)

        ###############################################################
        ## 直接デバイスのメモリ操作をしているkworkerのpidを特定
        ###############################################################
        read_device(touch_sensor_devfd)
        kworker_psdels_bytes = subprocess.check_output("ps aux | grep [k]worker/0:", shell=True)
        kworker_psdels_str = kworker_psdels_bytes.decode()
        kworker_psdels = {}
        kworker_psdels_str_splited = kworker_psdels_str.split("\n")
        kworker_psdels_str_splited.pop()
        for psdel in kworker_psdels_str_splited:
            psdel_arry = re.split("\s*", psdel)
            # print(psdel)
            # print("pid:%s, CPU:%s" % (psdel_arry[1], psdel_arry[2]))
            # CPU使用率をkeyに、valueをpidに。
            kworker_psdels[float(psdel_arry[2])] = psdel_arry[1]
            subprocess.check_output(("renice -13 -p %s" % psdel_arry[1]), shell=True)
        # kworker_psdels_sortedkeys = sorted(kworker_psdels.keys())
        # kworker_target_pid = kworker_psdels[kworker_psdels_sortedkeys[-1]]
        # subprocess.check_output(("renice -13 -p %s" % kworker_target_pid), shell=True)

        print('Im Pistol')

        while not touch_sensor_pressed:
            time.sleep(0.1)
            touch_sensor_pressed = read_device(touch_sensor_devfd)
            write_touch_sensor_mem(touch_sensor_pressed)

        time.sleep(1)
        touch_sensor_pressed = read_device(touch_sensor_devfd)

        while not touch_sensor_pressed:
            time.sleep(0.2)
            touch_sensor_pressed = read_device(touch_sensor_devfd)
            write_touch_sensor_mem(touch_sensor_pressed)

        shutdown()

    except (KeyboardInterrupt, Exception) as ex:
        log.exception(ex)
        shutdown()
