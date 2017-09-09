from time import sleep
from ev3dev.auto import TouchSensor, Sound
from device import read_device
from sys import stdin
import os
import fcntl
import termios

def pistol(sh_mem):
    print('Im Pistol')

    def wait_for_ready(interval):
        ready = False

        sh_mem.write_touch_sensor_mem(0)
        while not ready:
            ready = sh_mem.read_runner_is_ready_mem() and sh_mem.read_guide_is_ready_mem()
            sleep(interval)
        sh_mem.write_runner_is_ready_mem(0)
        sh_mem.write_guide_is_ready_mem(0)

    def wait_for_input(interval):
        try:
            # 入力待ちの合図
            Sound.beep('-f 500')

            # fdの端末属性を取得
            # newに変更を加えて、適応する
            # oldは、後で元に戻すため
            old = termios.tcgetattr(fd)
            new = termios.tcgetattr(fd)
            #new[3]はlflags
            #ECHO(入力された文字を表示するか否かのフラグ)を外す
            #ICANON(カノニカルモードのフラグ)を外す
            new[3] &= ~termios.ECHO & ~termios.ICANON
            # 書き換えたnewをfdに適応する
            termios.tcsetattr(fd, termios.TCSANOW, new)

            # 現在の設定を保存
            fcntl_old = fcntl.fcntl(fd, fcntl.F_GETFL)
            # stdinをNONBLOCKに設定
            fcntl.fcntl(fd, fcntl.F_SETFL, fcntl_old | os.O_NONBLOCK)

            touch_sensor_pressed = 0

            while not touch_sensor_pressed:
                sleep(interval)
                touch_sensor_pressed = read_device(touch_sensor_devfd)

                ch = stdin.read(1)
                if ch == '\n':
                    touch_sensor_pressed = 1

                sh_mem.write_touch_sensor_mem(touch_sensor_pressed)

        finally:
            termios.tcsetattr(fd, termios.TCSANOW, old)
            fcntl.fcntl(fd, fcntl.F_SETFL, fcntl_old)

    try:
        # touchSensorの読み込み
        touch = TouchSensor()
        touch_sensor_devfd = open(touch._path + "/value0", "rb")
        # 標準入力のファイルディスクリプタを取得
        fd = stdin.fileno()

        wait_for_ready(0.1) # 設置前の準備が終わるのを待つ

        print('-- PUSH BUTTON OR ENTER KEY. (CALIBRATION) --')

        wait_for_input(0.1) # 設置＆ユーザーの入力を待つ

        sleep(0.1)

        wait_for_ready(0.1) # キャリブレーションを待つ

        print('-- PUSH BUTTON OR ENTER KEY. (RUN) --')

        wait_for_input(0.1) # ユーザーの入力を待つ

        sleep(1)
        touch_sensor_pressed = 0

        # ユーザーの入力を待つ（止めるための）
        while not touch_sensor_pressed:
            sleep(0.2)
            touch_sensor_pressed = read_device(touch_sensor_devfd)
            sh_mem.write_touch_sensor_mem(touch_sensor_pressed)


    finally:
        touch_sensor_devfd.close()
