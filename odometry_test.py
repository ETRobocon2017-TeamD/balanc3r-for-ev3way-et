# coding:utf-8
from odometry import Odometry
import datetime
import time

class Testloop:

    def __init__(self):
        pass

    def main_loop(self):
        odometry = Odometry()
        direction = 0
        speed = 0

        angle_l = 0
        angle_r = 0

        print('ready')
        for _ in range(1000):
            direction , speed = odometry.target_trace(angle_l, angle_r)
            angle_l += 5
            angle_r += 10

if __name__ == '__main__':
    test = Testloop()
    test.main_loop()
