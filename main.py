import os
import sys
from time import tzset, strftime, sleep
import signal
import logging
import subprocess
import re
from shared_memory import SharedMemory
from runner import runner
from guide import guide
from pistol import pistol
from ev3dev.auto import Motor

g_log = logging.getLogger(__name__)

###############################################################
## カーネル空間をデバイスのkworkerのpidを特定
###############################################################
def renice_driver_kworkers():
    nice_value = -15

    kworkers_bytes = subprocess.check_output("ps aux | grep [k]worker/0:", shell=True)
    kworkers_text = kworkers_bytes.decode()
    kworker_lines = kworkers_text.split("\n")
    # 最終行は無効なので消去する
    kworker_lines.pop()
    for ps_line in kworker_lines:
        ps_columns = re.split("\s*", ps_line)
        subprocess.check_output(("renice {} -p {}".format(nice_value, ps_columns[1])), shell=True)
    os.nice(nice_value) # 自分のnice値も下げる

########################################################################
##
## メイン：Guide関数とRunner関数用の子プロセスをフォークする
##
########################################################################
if __name__ == '__main__':

    guide_pid = 0
    runner_pid = 0

    def shutdown():
        try:
            if ('guide_pid' in globals()) or ('guide_pid' in locals()):
                print('Kill Guide')
                os.kill(guide_pid, signal.SIGTERM)
            if ('runner_pid' in globals()) or ('runner_pid' in locals()):
                print('Kill Runner')
                os.kill(runner_pid, signal.SIGTERM)
            print('Done')

        except Exception as ex:
            print("It's a Pistol Exception in shutdown")
            g_log.exception(ex)

    try:
        #しっぽモーターの調整
        tail_motor = Motor('outA')
        tail_motor.run_timed(time_sp=1000, speed_sp=-200, stop_action='hold') # しっぽを一番上に上げる
        sleep(1)
        tail_motor.reset() # しっぽが一番上の状態を0度とする
        tail_motor.run_to_abs_pos(position_sp=100, stop_action='hold', speed_sp=300) # ちょうど安定して立つ角度にする

        # logフォルダの生成
        if not os.path.exists('./log/'):
            os.mkdir('./log/')
        # 日本時間に変更
        os.environ['TZ'] = "JST-9"
        tzset()
        log_datetime = strftime("%Y%m%d%H%M%S")
        print("Start time is {}".format(log_datetime))

        renice_driver_kworkers()

        # プロセス間共有メモリ
        sh_mem = SharedMemory()

        runner_pid = os.fork()

        if runner_pid == 0:
            # NOTE: 倒立振子ライブラリを使う場合はrunner()を、ライントレーサー開発等で倒立振子ライブラリを使いたくない場合はrunner_stub()を使用すること
            runner(sh_mem, log_datetime)
            print('Runner Done')
            sys.exit()

        guide_pid = os.fork()

        if guide_pid == 0:  # In a child process
            guide(sh_mem, log_datetime)
            # TODO: ガイドが提示する前進後退・旋回スピードはanonymous memoryで共有する
            # sh_mem.write_speed_mem(50)
            # sh_mem.write_steering_mem(50)
            print('Guide Done')
            sys.exit()

        pistol(sh_mem)

        shutdown()

    except KeyboardInterrupt as ex:
        if (runner_pid > 0) and (guide_pid > 0):
            print("It's a KeyboardInterrupt")
            shutdown()
        elif (runner_pid == 0) or (guide_pid == 0):
            sleep(10)

    except Exception as ex:
        if (runner_pid > 0) and (guide_pid > 0):
            print("It's a Pistol Exception")
            g_log.exception(ex)
            shutdown()
        elif (runner_pid == 0):
            print("It's a Runner Exception in shutdown")
            g_log.exception(ex)
        elif (guide_pid == 0):
            print("It's a Guide Exception in shutdown")
            g_log.exception(ex)
