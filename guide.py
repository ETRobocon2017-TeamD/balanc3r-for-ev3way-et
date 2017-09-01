import sys
import logging
import signal
from time import sleep, clock, strftime
from shared_memory import SharedMemory
from line_tracer import LineTracer

g_log = logging.getLogger(__name__)

########################################################################
##
## Guide：ラインの状態を感知して、前進後退速度と旋回速度を算出する関数
##
########################################################################
def guide(sh_mem):
    print('Im Guide')

    def shutdown_child(signum=None, frame=None):
        sleep(0.2)

        log_datetime = strftime("%Y%m%d%H%M%S")
        log_file = open("./log/log_guide_{}.csv".format(log_datetime), 'w')
        for log in logs:
            if log != "":
                log_file.write("{}\n".format(log))
        log_file.close()

        line_tracer.shutdown()
        sys.exit()

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

        print('Calibrate ColorSensor ...')
        line_tracer = LineTracer()
        line_tracer.calibrate_color_sensor()

        # タッチセンサー押し待ち
        print('Guide Waiting ...')
        while not sh_mem.read_touch_sensor_mem():
            sleep(0.1)

        speed_reference = 0
        direction = 0
        refrection_raw = 0

        # スタート時の時間取得
        t_line_trace_start = clock()

        while True:
            ###############################################################
            ##  Loop info
            ###############################################################
            t_loop_start = clock()

            # ここでライントレースする
            speed_reference, direction, refrection_raw = line_tracer.line_tracing()

            # 左右モーターの角度は下記のように取得
            # print(read_motor_encoder_left_mem())
            # print(read_motor_encoder_right_mem())

            # 前進後退・旋回スピードは下記のように入力
            sh_mem.write_speed_mem(speed_reference)
            sh_mem.write_steering_mem(int(round(direction)))

            # 実行時間、PID制御に関わる値をログに出力
            t_loop_end = clock()
            logs[log_pointer] = "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(
                t_loop_end - t_line_trace_start,
                t_loop_end - t_loop_start,
                speed_reference,
                direction,
                refrection_raw,
                line_tracer.refrection_target,
                line_tracer.e_b,
                line_tracer.p_b,
                line_tracer.i_b,
                line_tracer.d_b)
            log_pointer += 1

            ###############################################################
            ##  Busy wait for the loop to complete
            ###############################################################
            sleep(max(loop_time_sec - (clock() - t_loop_start), 0.002))

    except Exception as e:
        print("It's a Guide Exception")
        g_log.exception(e)
        shutdown_child()
