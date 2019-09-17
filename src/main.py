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
from settings import load_settings

g_log = logging.getLogger(__name__)

###############################################################
##
## 確実に倒立真摯に関係ないプロセスのナイス値を下げる
##
###############################################################
def renice_low_priority_processes():
    nice_value = 19
    process_names = "-e [b]rickman"
    cmds = "ps lax | grep {} | awk '{{print $3}}' | xargs sudo renice {} -p".format(process_names, nice_value)
    subprocess.check_output(cmds, shell=True)


###############################################################
##
## カーネル空間にあるデバイス制御に使われそうなkworkerのpidを特定し、nice値を上げる
##
###############################################################
def renice_driver_kworkers(nice_value):
    process_names = "-e '[k]worker/u2' -e '[k]worker/0'" # -e [w]riteback -e [k]blockd -e [r]cu_preempt -e [r]cu_sched -e [r]cu_bh -e [e]v3-tac -e [t]i-ads7"
    cmds = "ps aux | grep {} | awk '{{print $2}}' | xargs sudo renice {} -p".format(process_names, nice_value)
    subprocess.check_output(cmds, shell=True)

###############################################################
##
## ライントレースプログラムに関わる全てのプロセスのnice値を調整する
##
###############################################################
def renice_processes():
    renice_low_priority_processes()
    nice_value = -20
    renice_driver_kworkers(nice_value)
    os.nice(nice_value) # 自分のnice値も下げる

###############################################################
##
## しっぽモーターを自立できる角度で固定する
##
###############################################################
def stand_on_tail_motor():
    tail_motor = Motor('outA')
    tail_motor.run_timed(time_sp=1000, speed_sp=-200, stop_action='hold') # しっぽを一番上に上げる
    sleep(1)
    tail_motor.reset() # しっぽが一番上の状態を0度とする
    # ちょうど安定して立つ角度にする。 充電バッテリーの時:97、乾電池の時:93
    tail_motor.run_to_abs_pos(position_sp=93, stop_action='hold', speed_sp=300)

###############################################################
##
## プログラムのタイムゾーンを日本標準時に設定
##
###############################################################
def set_timezone_jst():
    os.environ['TZ'] = "JST-9"
    tzset()
    
########################################################################
##
## メイン：Guide関数とRunner関数用の子プロセスをフォークする
##
########################################################################
def main():
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
        #しっぽモーターを使って自立させる
        stand_on_tail_motor()

        # logフォルダの生成
        if not os.path.exists('./log/'):
            os.mkdir('./log/')
            
        # 日本時間に変更
        set_timezone_jst()

        # 日付フォーマッタを YYYYmmddHHMMSS に指定した
        log_datetime = strftime("%Y%m%d%H%M%S")
        print("Start time is {}".format(log_datetime))

        # ナイス値設定
        renice_processes()

        # プロセス間共有メモリ
        sh_mem = SharedMemory()

        # 設定ファイル読み込み
        setting = load_settings()

        runner_pid = os.fork()

        if runner_pid == 0:
            # スケジューラー設定
            if setting['use_fifo']:
                priority = os.sched_get_priority_min(os.SCHED_FIFO)
                sched_params = os.sched_param(priority)
                os.sched_setscheduler(runner_pid, os.SCHED_FIFO, sched_params)
            # NOTE: 倒立振子ライブラリを使う場合はrunner()を、ライントレーサー開発等で倒立振子ライブラリを使いたくない場合はrunner_stub()を使用すること
            runner(sh_mem, setting, log_datetime)
            print('Runner Done')
            sys.exit()

        guide_pid = os.fork()

        if guide_pid == 0:  # In a child process
            guide(sh_mem, setting, log_datetime)
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

###############################################################
##
## エントリーポイント
##
###############################################################
if __name__ == '__main__':
    main()
