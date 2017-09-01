from time import clock, sleep
import sys
import signal
import math
import logging
from ev3dev.auto import Motor, LargeMotor, GyroSensor, PowerSupply
from device import read_device, set_duty

g_log = logging.getLogger(__name__)

########################################################################
##
## Runner_stub：Runnerのスタブ
## ライントレーサー開発等で倒立振子ライブラリを使いたくない場合はrunner_stub()を使用すること
##
########################################################################
def runner_stub(sh_mem):
    print('Im Runner Stub')

    def shutdown_child(signum=None, frame=None):
        motor_encoder_left_devfd.close()
        motor_encoder_right_devfd.close()
        motor_duty_cycle_left_devfd.close()
        motor_duty_cycle_right_devfd.close()

        tail_motor.stop_action = tail_motor.STOP_ACTION_COAST
        tail_motor.stop()

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

        # しっぽモーターを固定する
        tail_motor.stop_action = tail_motor.STOP_ACTION_HOLD
        tail_motor.stop()

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
        motor_angular_speed_reference = 0.0

        a_r = 0.985 #0.98  # ローパスフィルタ係数(左右車輪の目標平均回転角度用)。左右モーターの目標平均回転角度(rad)の算出時に使用する。小さいほど前進・後退する反応が早くなる。
        k_theta_dot = 3.5 # モータ目標回転角速度係数

        # filehandles for fast reads/writes
        # =================================

        # Open motor files for (fast) reading
        motor_encoder_left_devfd = open(left_motor._path + "/position", "rb")
        motor_encoder_right_devfd = open(right_motor._path + "/position", "rb")

        # Open motor files for (fast) writing
        motor_duty_cycle_left_devfd = open(left_motor._path + "/duty_cycle_sp", "w")
        motor_duty_cycle_right_devfd = open(right_motor._path + "/duty_cycle_sp", "w")

        speed_reference = sh_mem.read_speed_mem()
        steering = sh_mem.read_steering_mem()

        ########################################################################
        ## タッチセンサー押し待ち
        ########################################################################
        print('Runner Waiting ...')
        while not sh_mem.read_touch_sensor_mem():
            sleep(0.025)

        print("-----------------------------------")
        print("GO!")
        print("-----------------------------------")

        # 倒立振子スタート時の時間取得
        t_balancer_start = clock()

        while True:

            ###############################################################
            ##  Loop info
            ###############################################################
            t_loop_start = clock()

            ###############################################################
            ##  Reading the Motor Position
            ###############################################################

            motor_angle_raw_left = read_device(motor_encoder_left_devfd)
            motor_angle_raw_right = read_device(motor_encoder_right_devfd)
            sh_mem.write_motor_encoder_left_mem(motor_angle_raw_left)
            sh_mem.write_motor_encoder_right_mem(motor_angle_raw_right)

            speed_reference = sh_mem.read_speed_mem()

            motor_angular_speed_reference = ((1.0 - a_r) * ((speed_reference / 100.0) * k_theta_dot)) + (a_r * motor_angular_speed_reference)

            ###############################################################
            ##  Computing the motor duty cycle value
            ###############################################################
            motor_duty_cycle = motor_angular_speed_reference

            ###############################################################
            ##  Apply the signal to the motor, and add steering
            ###############################################################
            steering = sh_mem.read_steering_mem()
            set_duty(motor_duty_cycle_right_devfd, speed_reference + steering)
            duty = set_duty(motor_duty_cycle_left_devfd, speed_reference - steering)

            ###############################################################
            ##  Busy wait for the loop to complete
            ###############################################################
            # clock()の値にはsleep中の経過時間が含まれないので、このwhileの条件文の算出時間をsleep代わりにしている(算出時間はバラバラ…)
            sleep(max(loop_time_sec - (clock() - t_loop_start), 0.002))

    except Exception as ex:
        print("It's a Runner Exception")
        g_log.exception(ex)
        shutdown_child()
