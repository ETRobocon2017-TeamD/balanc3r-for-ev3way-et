import sys
import logging
import signal
from time import sleep, clock, strftime
from shared_memory import SharedMemory
from line_tracer import LineTracer
from odometry import Odometry

g_log = logging.getLogger(__name__)

########################################################################
##
## Guide：ラインの状態を感知して、前進後退速度と旋回速度を算出する関数
##
########################################################################
def guide(sh_mem, setting, log_datetime):
    print('Im Guide')

    def shutdown_child(signum=None, frame=None):
        try:
            sleep(0.2)

            log_file = open("./log/log_{}_guide.csv".format(log_datetime), 'w')
            for log in logs:
                if log != "":
                    log_file.write("{}\n".format(log))
            log_file.close()

            if ('line_tracer' in globals()) or ('line_tracer' in locals()):
                line_tracer.shutdown()

            sys.exit()

        except Exception as ex:
            g_log.exception(ex)
            raise ex

    def wait_for_input():
        print('Guide Waiting ...')
        sh_mem.write_guide_is_ready_mem(1)
        while not sh_mem.read_touch_sensor_mem():
            sleep(0.1)

    signal.signal(signal.SIGTERM, shutdown_child)

    try:
        # ここで変数定義などの事前準備を行う
        # Time of each loop, measured in miliseconds.
        loop_time_millisec = 25
        # Time of each loop, measured in seconds.
        loop_time_sec = loop_time_millisec / 1000.0

        # ログ記録用
        logs = ["" for _ in range(10000)]
        log_pointer = 0

        wait_for_input() # 設置が終わるまで待つ

        line_tracer = LineTracer(setting)

        # プログラム実行時にキャリブレーションを実行する場合は以下のコードを実行する
        # print('Calibrate ColorSensor ...')
        # line_tracer.calibrate_color_sensor()

        print('Configurating Odometry ...')
        odometry = Odometry()

        print('Guide is Ready')

        # タッチセンサー押し待ち
        wait_for_input()

        speed_reference = 0
        direction = 0
        odometry_speed_reference = 0
        odometry_direction = 0
        refrection_raw = 0

        angle_l = 0
        angle_r = 0

        # スタート時の時間取得
        t_line_trace_start = clock()

        while True:
            ###############################################################
            ##  Loop info
            ###############################################################
            t_loop_start = clock()

            # ここでライントレースする
            speed_reference, direction, refrection_raw = line_tracer.line_tracing()

            # 角度を算出してオドメトリーを使用
            # angle_l = sh_mem.read_motor_encoder_left_mem()
            # angle_r = sh_mem.read_motor_encoder_right_mem()
            # odometry_speed_reference, odometry_direction = odometry.target_trace(angle_l,angle_r)
            # direction = odometry_direction
            # speed_reference = odometry_speed_reference

            # 左右モーターの角度は下記のように取得
            # print(read_motor_encoder_left_mem())
            # print(read_motor_encoder_right_mem())

            # 前進後退・旋回スピードは下記のように入力
            sh_mem.write_speed_mem(speed_reference)
            sh_mem.write_steering_mem(int(round(direction)))

            # 実行時間、PID制御に関わる値をログに出力
            t_loop_end = clock()
            logs[log_pointer] = "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(
                t_loop_end - t_line_trace_start,
                t_loop_end - t_loop_start,
                speed_reference,
                direction,
                refrection_raw,
                line_tracer.refrection_target,
                line_tracer.e_b,
                line_tracer.p_b,
                line_tracer.i_b,
                line_tracer.d_b,
                angle_l,
                angle_r,
                odometry.pre_pos_x,
                odometry.pre_pos_y,
                odometry_direction,
                odometry_speed_reference)
            log_pointer += 1

            ###############################################################
            ##  Busy wait for the loop to complete
            ###############################################################
            sleep(max(loop_time_sec - (clock() - t_loop_start), 0.002))

    except Exception as e:
        print("It's a Guide Exception")
        g_log.exception(e)
        shutdown_child()
