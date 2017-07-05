# -*- coding: utf_8 -*-
#!/usr/bin/env python3

"""
This is a class-based version of https://github.com/laurensvalk/segway
"""
import logging
import math
import time
from collections import deque
from ev3dev.auto import *
from ev3dev.helper import Tank


log = logging.getLogger(__name__)


########################################################################
##
## File I/O functions
##
########################################################################

# Function for fast reading from sensor files
def FastRead(infile):
    infile.seek(0)
    return int(infile.read().decode().strip())


# Function for fast writing to motor files
def FastWrite(outfile, value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()


# Function to set the duty cycle of the motors
def SetDuty(motorDutyFileHandle, duty, barance=1):
    # Clamp the value between -100 and 100
    duty = min(max(duty, -100), 100)*barance

    # Apply the signal to the motor
    FastWrite(motorDutyFileHandle, duty)

    return duty


class GyroBalancer(Tank):
    """
    Base class for a robot that stands on two wheels and uses a gyro sensor
    to keep its balance.
    """

    def __init__(self,
                 gainGyroAngle,                   # For every radian (57 degrees) we lean forward,            apply this amount of duty cycle
                 gainGyroRate,                    # For every radian/s we fall forward,                       apply this amount of duty cycle
                 gainMotorAngle,                  # For every radian we are ahead of the reference,           apply this amount of duty cycle
                 gainMotorAngularSpeed,           # For every radian/s drive faster than the reference value, apply this amount of duty cycle
                 gainMotorAngleErrorAccumulated,  # For every radian x s of accumulated motor angle,          apply this amount of duty cycle
                 left_motor=OUTPUT_C,
                 right_motor=OUTPUT_B):
        Tank.__init__(self, left_motor, right_motor)

        # magic numbers
        self.gainGyroAngle                  = gainGyroAngle
        self.gainGyroRate                   = gainGyroRate
        self.gainMotorAngle                 = gainMotorAngle
        self.gainMotorAngularSpeed          = gainMotorAngularSpeed
        self.gainMotorAngleErrorAccumulated = gainMotorAngleErrorAccumulated

        # Sensor setup
        self.gyro = GyroSensor()
        self.gyro.mode = self.gyro.MODE_GYRO_RATE
        self.touch = TouchSensor()
        self.battery = PowerSupply()
        self.color = ColorSensor()

        # TODO: デバッグ録画時、スタートした瞬間を映像に記録するため、カラーセンサーの色を変えている(後で消す)
        # self.color.ambient_light_intensity

        # Motor setup
        self.left_motor.reset()
        self.right_motor.reset()
        self.left_motor.run_direct()
        self.right_motor.run_direct()

        #しっぽモーター
        self.tail_motor = Motor('outA')

        self.speed     = 0
        self.steering  = 0
        self.red_up    = False
        self.red_down  = False
        self.blue_up   = False
        self.blue_down = False
        self.STEER_SPEED = 20

    def main(self):

        def shutdown():
            touchSensorValueRaw.close()
            gyroSensorValueRaw.close()
            batteryVoltageRaw.close()
            motorEncoderLeft.close()
            motorEncoderRight.close()
            motorDutyCycleLeft.close()
            motorDutyCycleRight.close()

            for motor in list_motors():
                motor.stop()

        try:

            ########################################################################
            ##
            ## Definitions and Initialization variables
            ##
            ########################################################################

            # Timing settings for the program
            loopTimeMilliSec     = 25                       # Time of each loop, measured in miliseconds.
            loopTimeSec          = loopTimeMilliSec/1000.0  # Time of each loop, measured in seconds.
            batteryHistoryLength = 5                        # Number of previous motor angles we keep track of.

            # Math constants
            radiansPerDegree               = math.pi/180       # The number of radians in a degree.

            # Platform specific constants and conversions
            degPerSecondPerRawGyroUnit     = 1                                             # For the LEGO EV3 Gyro in Rate mode, 1 unit = 1 deg/s
            radiansPerSecondPerRawGyroUnit = degPerSecondPerRawGyroUnit*radiansPerDegree   # Express the above as the rate in rad/s per gyro unit
            degPerRawMotorUnit             = 1                                             # For the LEGO EV3 Large Motor 1 unit = 1 deg
            radiansPerRawMotorUnit         = degPerRawMotorUnit*radiansPerDegree           # Express the above as the angle in rad per motor unit
            RPMperPerPercentSpeed          = 1.7                                           # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled) EV3では、「speed」を1%とした場合、１分間に1.7回転させる速さであると定義する（※電圧によって変わる）
            degPerSecPerPercentSpeed       = RPMperPerPercentSpeed*360/60                  # Convert this number to the speed in deg/s per "percent speed" 「speed」を回転角速度(deg)に変換する係数
            radPerSecPerPercentSpeed       = degPerSecPerPercentSpeed * radiansPerDegree   # Convert this number to the speed in rad/s per "percent speed" 「speed」を回転角速度(rad)に変換する係数

            # The rate at which we'll update the gyro offset (precise definition given in docs) ジャイロ値を補正するオフセット値の更新に使用する。調節する必要がある。
            gyroDriftCompensationRate      = 0.075 * loopTimeSec * radiansPerSecondPerRawGyroUnit

            # State feedback control gains (aka the magic numbers)
            gainGyroAngle                  = self.gainGyroAngle
            gainGyroRate                   = self.gainGyroRate
            gainMotorAngle                 = self.gainMotorAngle
            gainMotorAngularSpeed          = self.gainMotorAngularSpeed
            gainMotorAngleErrorAccumulated = self.gainMotorAngleErrorAccumulated

            battery_gain = 0.001089  # PWM出力算出用バッテリ電圧補正係数
            battery_offset = 0.625  # PWM出力算出用バッテリ電圧補正オフセット

            a_d = 1.0 - 0.55 #0.51 #0.47  # ローパスフィルタ係数(左右車輪の平均回転角度用)。左右モーターの平均回転角速度(rad/sec)の算出時にのみ使用する。小さいほど角速度の変化に過敏になる。〜0.4951
            a_r = 0.985 #0.98  # ローパスフィルタ係数(左右車輪の目標平均回転角度用)。左右モーターの目標平均回転角度(rad)の算出時に使用する。小さいほど前進・後退する反応が早くなる。
            a_b = 0.85 #ローパスフィルタ係数(最大モーター電圧b用）

            # Variables representing physical signals (more info on these in the docs)
            # The angle of "the motor", measured in raw units (degrees for the
            # EV3). We will take the average of both motor positions as "the motor"
            # angle, wich is essentially how far the middle of the robot has traveled.
            motorAngleRaw              = 0

            # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
            motorAngle                 = 0

            # 1ループ前に算出した左右車輪回転角度
            motorAngleLast             = 0

            # The reference angle of the motor. The robot will attempt to drive
            # forward or backward, such that its measured position equals this
            # reference (or close enough).
            motorAngleReference        = 0

            # The error: the deviation of the measured motor angle from the reference.
            # The robot attempts to make this zero, by driving toward the reference.
            motorAngleError            = 0

            # We add up all of the motor angle error in time. If this value gets out of
            # hand, we can use it to drive the robot back to the reference position a bit quicker.
            motorAngleErrorAccumulated = 0

            # The motor speed, estimated by how far the motor has turned in a given amount of time
            motorAngularSpeed          = 0

            # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
            motorAngularSpeedReference = 0

            # The error: the deviation of the motor speed from the reference speed.
            motorAngularSpeedError     = 0

            # The 'voltage' signal we send to the motor. We calulate a new value each
            # time, just right to keep the robot upright.
            motorDutyCycle             = 0

            # The raw value from the gyro sensor in rate mode.
            gyroRateRaw                = 0

            # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
            gyroRate                   = 0

            # 現在のバッテリー電圧
            voltageRaw = 0

            # The gyro doesn't measure the angle of the robot, but we can estimate
            # this angle by keeping track of the gyroRate value in time
            gyroEstimatedAngle         = 0

            # Over time, the gyro rate value can drift. This causes the sensor to think
            # it is moving even when it is perfectly still. We keep track of this offset.
            gyroOffset                 = 0

            # ログ記録用
            logs = ["" for _ in range(10000)]
            log_pointer = 0

            voltageTarget = 0
            voltageEstimateMax = 0
            gyroAsseletion = 0
            gyroRateEst = 0

            # filehandles for fast reads/writes
            # =================================
            touchSensorValueRaw = open(self.touch._path + "/value0", "rb")
            gyroSensorValueRaw  = open(self.gyro._path + "/value0", "rb")
            batteryVoltageRaw = open(self.battery._path + "/voltage_now", "rb")

            # Open motor files for (fast) reading
            motorEncoderLeft    = open(self.left_motor._path + "/position", "rb")
            motorEncoderRight   = open(self.right_motor._path + "/position", "rb")

            # Open motor files for (fast) writing
            motorDutyCycleLeft  = open(self.left_motor._path + "/duty_cycle_sp", "w")
            motorDutyCycleRight = open(self.right_motor._path + "/duty_cycle_sp", "w")

            # バッテリー電圧
            voltageRaw = FastRead(batteryVoltageRaw) # 単位(μV)
            # 現在のバッテリー電圧をもとに、PWMデューティ値を100%にしたとき、モータが受け取る推定電圧を求める。
            # 6.2 アクチュエータ　(6.1)式 バッテリ電圧とモータ回転速度(rpm)の関係式
            voltageEstimateMax = (battery_gain * voltageRaw / 1000) - battery_offset

            ########################################################################
            ##
            ## Calibrate Gyro
            ##
            ########################################################################
            print("-----------------------------------")
            print("Calibrating...")

            #As you hold the robot still, determine the average sensor value of 100 samples
            gyroRateCalibrateCount = 100
            gyroRateSquareds = 0
            for i in range(gyroRateCalibrateCount):
                gyroRateRaw = FastRead(gyroSensorValueRaw)
                gyroRateSquareds = gyroRateSquareds + pow(gyroRateRaw,2)
                gyroOffset = gyroOffset + gyroRateRaw
                time.sleep(0.01)
            gyroOffset = gyroOffset/gyroRateCalibrateCount
            p_bar = gyroRateSquareds/gyroRateCalibrateCount #ジャイロセンサーの事前予測の分散値 #初期値

            # Print the result
            print("GyroOffset: %s" % gyroOffset)
            print("-----------------------------------")
            print("GO!")
            self.tail_motor.run_timed(time_sp=250, speed_sp=-300) # しっぽモーター上に上げる
            print("-----------------------------------")

            #self.speed = 62.5 # 前進・後退速度。62.5〜-62.5の間で入力
            #self.steering = -1 * self.STEER_SPEED * 0.5 # 旋回速度。他の係数をいじったせいか、今の値でも少々不安定になっている。もう少し下げたほうがいいかも

            ########################################################################
            ##
            ## MAIN LOOP (Press Touch Sensor to stop the program)
            ##
            ########################################################################

            # Initial touch sensor value
            touchSensorPressed = FastRead(touchSensorValueRaw)

            # TODO: デバッグ録画時、スタートした瞬間を映像に記録するため、カラーセンサーの色を変えている(後で消す)
            # self.color.reflected_light_intensity

            # 倒立振子スタート時の時間取得
            tBalancerStart = time.clock()

            while not touchSensorPressed:

                ###############################################################
                ##  Loop info
                ###############################################################
                tLoopStart = time.clock()

                ###############################################################
                ##  Reading the Gyro.
                ###############################################################
                gyroRateRaw = FastRead(gyroSensorValueRaw)
                gyroRate = (gyroRateRaw - gyroOffset) * radiansPerSecondPerRawGyroUnit # 躯体の角速度(rad/sec)。ジャイロから得た角速度をオフセット値で調整している

                ###############################################################
                ##  Reading the Motor Position
                ###############################################################
                motorAngleLast = motorAngle
                motorAngleRaw = (FastRead(motorEncoderLeft) + FastRead(motorEncoderRight)) * 0.5
                motorAngle = (motorAngleRaw * radiansPerRawMotorUnit) + gyroEstimatedAngle # 左右モーターの現在の平均回転角度(rad) + 躯体の(推定)回転角度

                #motorAngularSpeedReference = self.speed * radPerSecPerPercentSpeed # 左右モーターの目標平均回転角速度(rad/sec)。入力値speedを角速度(rad)に変換したもの。
                # K_THETA_DOT(7.5): 最大モーター角速度だと思われる値。 self.speed(モータ最大角速度を100%とする、目標割合)にかけ合わせて
                # FIXME: self.speedは現在 0 が前提になるように実装されてしまっている。
                motorAngularSpeedReference = ((1.0 - a_r) * ((self.speed / 100.0) * 7.5)) + (a_r * motorAngularSpeedReference)
                motorAngleReference = motorAngleReference + (motorAngularSpeedReference * loopTimeSec) # 左右モーターの目標平均回転角度(rad)。初期値は0になる。入力値speedがずっと0でも0になる

                motorAngleError = motorAngle - motorAngleReference # 左右モーターの現在の平均回転角度と目標平均回転角度との誤差(rad)

                ###############################################################
                ##  Computing Motor Speed
                ###############################################################
                motorAngularSpeed = (((1.0 - a_d) * motorAngle) + (a_d * motorAngleLast) - motorAngleLast) / loopTimeSec # 左右モーターの平均回転角速度(rad/sec)。左右モーターの現在平均回転角度をローパスフィルターに通して、前回の平均回転角度との差分を周期で割っている。
                motorAngularSpeedError = motorAngularSpeed - motorAngularSpeedReference # 左右モーターの現在の平均回転角速度と目標平均回転角速度との誤差(rad/sec)

                ###############################################################
                ##  Reading the Voltage.
                ###############################################################
                voltageRaw = FastRead(batteryVoltageRaw) #バッテリー電圧(μV)

                ###############################################################
                ##  Computing the motor duty cycle value
                ###############################################################
                voltageTarget = ((gainGyroAngle  * gyroEstimatedAngle)
                   + (gainGyroRate   * gyroRate)
                   + (gainMotorAngle * motorAngleError)
                   + (gainMotorAngularSpeed * motorAngularSpeedError)
                   + (gainMotorAngleErrorAccumulated * motorAngleErrorAccumulated))
                voltageEstimateMax = (battery_gain * voltageRaw / 1000) - battery_offset
                motorDutyCycle = (voltageTarget / voltageEstimateMax) * 100

                ###############################################################
                ##  Apply the signal to the motor, and add steering
                ###############################################################
                SetDuty(motorDutyCycleRight, motorDutyCycle + self.steering)
                duty = SetDuty(motorDutyCycleLeft, motorDutyCycle - self.steering, 0.975) # 右車輪のモーター出力が弱いので、左車輪のPWM値を3つ目の引数で調節(%)してる。まだ偏ってるので調節必要

                ###############################################################
                ##  Update angle estimate and Gyro Offset Estimate
                ###############################################################
                gyroEstimatedAngle = gyroEstimatedAngle + (gyroRate * loopTimeSec) # 次回の躯体の（推定）回転角度(rad)
                gyroOffset = ((1 - gyroDriftCompensationRate) * gyroOffset) + (gyroDriftCompensationRate * gyroRateRaw) # ジャイロの角速度を補正するオフセット値(rad/sec)の更新。 現状gyroDriftCompensationRateが極小なので、ほぼほぼ前回算出したオフセット値寄りになる

                ###############################################################
                ##  Update Accumulated Motor Error
                ###############################################################
                motorAngleErrorAccumulated = motorAngleErrorAccumulated + (motorAngleError * loopTimeSec) # モーター角度誤差累積(rad*t) もしかして積分？ 前回の累積に、今回の目標平均回転角度との誤差を周期でかけたものを足している

                ###############################################################
                ##  Read the touch sensor (the kill switch)
                ###############################################################
                touchSensorPressed = FastRead(touchSensorValueRaw)

                # 実行時間、PWM値(duty cycle value)に関わる値をログに出力
                tLoopEnd = time.clock()
                logs[log_pointer] = "{}   {}   {}   {}   {}   {}   {}   {}   {}   {}   {}".format(
                    tLoopEnd - tBalancerStart,
                    tLoopEnd - tLoopStart,
                    gyroRateRaw,
                    motorAngleRaw,
                    gyroEstimatedAngle,
                    gyroRate,
                    motorAngleError,
                    motorAngularSpeedError,
                    motorAngleErrorAccumulated,
                    duty,
                    voltageRaw)
                log_pointer += 1

                ###############################################################
                ##  Busy wait for the loop to complete
                ###############################################################
                while ((time.clock() - tLoopStart) < loopTimeSec): # clock()の値にはsleep中の経過時間が含まれないので、このwhileの条件文の算出時間をsleep代わりにしている(算出時間はバラバラ…)
                    time.sleep(0.0001)

            shutdown()
            for log_ in logs:
                if log_ != "":
                    print(log_)

        # Exit cleanly so that all motors are stopped
        except (KeyboardInterrupt, Exception) as e:
            log.exception(e)
            shutdown()
