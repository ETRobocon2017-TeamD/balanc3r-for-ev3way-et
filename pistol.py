from time import sleep
from ev3dev.auto import TouchSensor
from device import read_device

def pistol(sh_mem):
    print('Im Pistol')
    
    try:
        # touchSensorの読み込み
        touch = TouchSensor()
        touch_sensor_devfd = open(touch._path + "/value0", "rb")
        touch_sensor_pressed = read_device(touch_sensor_devfd)
        touch_sensor_pressed = False

        while not touch_sensor_pressed:
            sleep(0.1)
            touch_sensor_pressed = read_device(touch_sensor_devfd)
            sh_mem.write_touch_sensor_mem(touch_sensor_pressed)

        sleep(1)
        touch_sensor_pressed = read_device(touch_sensor_devfd)

        while not touch_sensor_pressed:
            sleep(0.2)
            touch_sensor_pressed = read_device(touch_sensor_devfd)
            sh_mem.write_touch_sensor_mem(touch_sensor_pressed)

    finally:
        touch_sensor_devfd.close()
