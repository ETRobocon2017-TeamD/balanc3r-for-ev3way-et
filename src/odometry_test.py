# coding:utf-8
from odometry import Odometry
import datetime
import time

class Testloop:

    def __init__(self):
        self.odometry = Odometry()
        self.BASE_SLEEP_TIME_US = 0.250  * 1000000

    def main_loop(self):
        logs = ["" for _ in range(10000)]
        log_pointer = 0
        speed = 0
        direction = 0

        angle_l = 0
        angle_r = 0


        print('ready')
        for _ in range(1000):
            start = datetime.datetime.now()

            direction , speed = self.odometry.target_trace(angle_l,angle_r)

            angle_l += 5
            angle_r += 10

            # 余った時間はsleep
            # elapsed_microsecond = (datetime.datetime.now() - start).microseconds
            # if elapsed_microsecond < self.BASE_SLEEP_TIME_US:
            #     sleep_time = (self.BASE_SLEEP_TIME_US - elapsed_microsecond) / 1000000
            #     time.sleep(sleep_time)

            logs[log_pointer] = "{}, {}, {}, {}".format(
                angle_l,
                angle_r,
                speed,
                direction)
            log_pointer += 1

        log_file = open("./log_test.csv", 'w')
        for log in logs:
            if log != "":
                log_file.write("{}\n".format(log))
        log_file.close()

        #ログ記録用
        self.odometry.shutdown(0000)


if __name__ == '__main__':
    test = Testloop()
    test.main_loop()
