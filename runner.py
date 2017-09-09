from time import strftime, sleep, clock
import sys
import signal
import math
import logging
from ev3dev.auto import Motor, LargeMotor, GyroSensor, PowerSupply
from device import read_device, set_duty

g_log = logging.getLogger(__name__)

########################################################################
##
## Runner：倒立振子の制御とDuty比の入力を行う関数
##
########################################################################
def runner(sh_mem, log_datetime):
    print('Im Runner')

    def shutdown_child(signum=None, frame=None):
        try:
            left_motor.stop()
            right_motor.stop()

            sleep(0.2)

            log_file = open('./log/log_%s_runner.csv' % log_datetime,'w')
            for log in logs:
                if log != "":
                    log_file.write("{}\n".format(log))
            log_file.close()

            gyro_sensor_devfd.close()
            battery_voltage_devfd.close()
            motor_encoder_left_devfd.close()
            motor_encoder_right_devfd.close()
            motor_duty_cycle_left_devfd.close()
            motor_duty_cycle_right_devfd.close()

            sys.exit()

        except Exception as ex:
            g_log.exception(e)
            raise ex

    def wait_for_input():
        print('Runner Waiting ...')
        sh_mem.write_runner_is_ready_mem(1)
        while not sh_mem.read_touch_sensor_mem():
            sleep(0.025)

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
        # 8.5V - 0.60
        # 8.2V - 0.68
        # 7.5V - 0.77
        gain_all                           = 0.68 #0.70 #0.81
        gain_motor_angle                   = 0.1606 * 3 * gain_all    # K_F[0]
        gain_gyro_angle                    = 30.2153 * 2.5 * gain_all # K_F[1]
        gain_motor_angular_speed           = 1.0796 * 1.7 * gain_all  # K_F[2]
        gain_gyro_rate                     = 3.3269 * 2 * gain_all    # K_F[3]
        gain_motor_angle_error_accumulated = 0.4472 * gain_all        # K_I

        battery_gain_left = 0.001276 * 1.3632  # PWM出力算出用バッテリ電圧補正係数(左モーター用)
        battery_offset_left = 1.959 * 2.94  # PWM出力算出用バッテリ電圧補正オフセット(左モーター用)
        battery_gain_right = 0.000688 * 0.95 * 1.8217 # PWM出力算出用バッテリ電圧補正係数(右モーター用)
        battery_offset_right = -2.712 * 0.95 * -0.7094 # PWM出力算出用バッテリ電圧補正オフセット(右モーター用)

        a_d = 1.0 - 0.55 #0.51 #0.47  # ローパスフィルタ係数(左右車輪の平均回転角度用)。左右モーターの平均回転角速度(rad/sec)の算出時にのみ使用する。小さいほど角速度の変化に過敏になる。〜0.4951
        a_r = 0.985 #0.98  # ローパスフィルタ係数(左右車輪の目標平均回転角度用)。左右モーターの目標平均回転角度(rad)の算出時に使用する。小さいほど前進・後退する反応が早くなる。
        a_b = 0.85 #ローパスフィルタ係数(最大モーター電圧b用）
        k_theta_dot = 7.5 # モータ目標回転角速度係数

        # Variables representing physical signals (more info on these in the docs)
        # The angle of "the motor", measured in raw units (degrees for the
        # EV3). We will take the average of both motor positions as "the motor"
        # angle, wich is essentially how far the middle of the robot has traveled.
        motor_angle_left_raw = 0
        motor_angle_right_raw = 0
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
        motor_duty_cycle_left = 0
        motor_duty_cycle_right = 0

        # ログ出力用のデューティー比用変数
        duty_left = 0
        duty_right = 0

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
        voltage_estimate_max_left = 0
        voltage_estimate_max_right = 0

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
        voltage_estimate_max_left = (battery_gain_left * voltage_raw / 1000) - battery_offset_left
        voltage_estimate_max_right = (battery_gain_right * voltage_raw / 1000) - battery_offset_right

        ########################################################################
        ## Calibrate Gyro
        ########################################################################

        wait_for_input() # 設置が終わるまで待つ

        print("-----------------------------------")
        print("Calibrating...")

        #As you hold the robot still, determine the average sensor value of 100 samples
        gyro_rate_calibrate_count = 100
        for _ in range(gyro_rate_calibrate_count):
            gyro_rate_raw = read_device(gyro_sensor_devfd)
            gyro_offset = gyro_offset + gyro_rate_raw
            sleep(0.01)
        gyro_offset = gyro_offset / gyro_rate_calibrate_count

        # Print the result
        print("GyroOffset: %s" % gyro_offset)

        speed_reference = sh_mem.read_speed_mem()
        steering = sh_mem.read_steering_mem()

        #speed_reference = 62.5 # 前進・後退速度。62.5〜-62.5の間で入力
        #steering = -1 * self.STEER_SPEED * 0.5 # 旋回速度。他の係数をいじったせいか、今の値でも少々不安定になっている。もう少し下げたほうがいいかも

        ########################################################################
        ## タッチセンサー押し待ち
        ########################################################################

        print('Runner is Ready')
        wait_for_input()

        print("-----------------------------------")
        print("GO!")
        print("-----------------------------------")

        tail_motor.run_timed(time_sp=100, speed_sp=120) # しっぽモーター下に少し下げる
        sleep(0.1)

        # 倒立振子スタート時の時間取得
        t_balancer_start = clock()

        while True:

            ###############################################################
            ##  Loop info
            ###############################################################
            t_loop_start = clock()

            ###############################################################
            ##  Reading the Gyro.
            ###############################################################
            gyro_rate_raw = read_device(gyro_sensor_devfd)
            gyro_rate = (gyro_rate_raw - gyro_offset) * rad_per_second_per_raw_gyro_unit # 躯体の角速度(rad/sec)。ジャイロから得た角速度をオフセット値で調整している

            # 倒れた時のログを確認したところ、倒れた時点の角速度が4以上だった。
            # TODO: 高速走行中にジャイロ値が4.0を超えるケースがあるようだ。グラフで確認すること。
            if gyro_rate > 5.0 or gyro_rate < -5.0:
                # TODO: 倒れた時用の例外クラスをつくること
                raise Exception('I fell down!')

            ###############################################################
            ##  Reading the Motor Position
            ###############################################################
            motor_angle_left_raw = read_device(motor_encoder_left_devfd)
            motor_angle_right_raw = read_device(motor_encoder_right_devfd)
            # sh_mem.write_motor_encoder_left_mem(motor_angle_left_raw)
            # sh_mem.write_motor_encoder_right_mem(motor_angle_right_raw)

            motor_angle_last = motor_angle
            motor_angle_raw = (motor_angle_left_raw + motor_angle_right_raw) * 0.5
            motor_angle = (motor_angle_raw * radians_per_raw_motor_unit) + gyro_estimated_angle # 左右モーターの現在の平均回転角度(rad) + 躯体の(推定)回転角度

            speed_reference = sh_mem.read_speed_mem()

            # K_THETA_DOT(7.5): 最大モーター角速度だと思われる値。 speed_reference(モータ最大角速度を100%とする、目標割合)にかけ合わせて
            motor_angular_speed_reference_next = (speed_reference / 100.0) * k_theta_dot
            motor_angular_speed_reference = ((1.0 - a_r) * motor_angular_speed_reference_next) + (a_r * motor_angular_speed_reference)
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
            voltage_estimate_max_left = (battery_gain_left * voltage_raw / 1000) - battery_offset_left
            voltage_estimate_max_right = (battery_gain_right * voltage_raw / 1000) - battery_offset_right
            motor_duty_cycle_left = (voltage_target / voltage_estimate_max_left) * 100
            motor_duty_cycle_right = (voltage_target / voltage_estimate_max_right) * 100

            ###############################################################
            ##  Apply the signal to the motor, and add steering
            ###############################################################
            steering = sh_mem.read_steering_mem()
            duty_right = set_duty(motor_duty_cycle_right_devfd, motor_duty_cycle_right + steering)
            duty_left = set_duty(motor_duty_cycle_left_devfd, motor_duty_cycle_left - steering) # 右車輪のモーター出力が弱いので、左車輪のPWM値を3つ目の引数で調節(%)してる。まだ偏ってるので調節必要

            ###############################################################
            ##  ここでしっぽモーターを上げる
            ###############################################################
            if gyro_estimated_angle == 0:
                tail_motor.run_timed(time_sp=250, speed_sp=-300, stop_action='coast') # しっぽモーター上に上げる

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
            t_loop_end = clock()
            logs[log_pointer] = "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(
                t_loop_end - t_balancer_start,
                t_loop_end - t_loop_start,
                gyro_rate_raw,
                motor_angle_raw,
                gyro_estimated_angle,
                gyro_rate,
                motor_angle_error,
                motor_angular_speed_error,
                motor_angle_error_accumulated,
                duty_left,
                duty_right,
                voltage_raw,
                voltage_estimate_max_left,
                voltage_estimate_max_right,
                motor_duty_cycle_left,
                motor_duty_cycle_right,
                )
            log_pointer += 1

            ###############################################################
            ##  Busy wait for the loop to complete
            ###############################################################
            # while ((clock() - t_loop_start) < (loop_time_sec - 0.011)): # clock()の値にはsleep中の経過時間が含まれないので、このwhileの条件文の算出時間をsleep代わりにしている(算出時間はバラバラ…)
                # sleep(0.0001)
            sleep(max(loop_time_sec - (gyro_rate*0.0005) - (clock() - t_loop_start), 0.002))

    except Exception as e:
        print("It's a Runner Exception")
        g_log.exception(e)
        shutdown_child()
