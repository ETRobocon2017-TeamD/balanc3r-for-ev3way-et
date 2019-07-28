from ev3dev.auto import ColorSensor, TouchSensor
from device import read_device
from time import sleep

def main():
    print("Start")
    color = ColorSensor()
    color.mode = color.MODE_REF_RAW #raw値
    color_reflection_fd = open(color._path + "/value0", "rb")

    # touchSensorの読み込み
    touch = TouchSensor()
    touch_sensor_devfd = open(touch._path + "/value0", "rb")
    touch_sensor_pressed = False
    interval = 0.2

    while not touch_sensor_pressed:
        touch_sensor_pressed = read_device(touch_sensor_devfd)

        color_val = read_device(color_reflection_fd)
        print("color:{}", color_val)
        sleep(interval)

#     except KeyboardInterrupt as ex:
#         pass
        
if __name__ == "__main__":
    main()
