import mmap

########################################################################
##
## File I/O functions
##
########################################################################

# mmapオブジェクト読み取り用関数
def read_anon_mem(memfd):
    memfd.seek(0)
    bts = memfd.read()
    length = bts[0]
    idx_end = 1 + length
    return int.from_bytes(bts[1:idx_end], byteorder='little', signed=True)

# mmapオブジェクト書き込み用関数
def write_anon_mem(memfd, value):
    memfd.seek(0)
    val_length = value.bit_length()
    bts = bytearray(1 + val_length)
    bts[0] = val_length
    bts[1:] = value.to_bytes(val_length, byteorder='little', signed=True)
    memfd.write(bytes(bts))

class SharedMemory:
    __slots__ = ('touch_sensor_memfd', 'speed_memfd', 'steering_memfd', 'motor_encoder_left_memfd', 'motor_encoder_right_memfd', 'runner_is_ready_memfd', 'guide_is_ready_memfd')

    def __init__(self):
        MMAP_SIZE = 18 # マップするバイト数

        # タッチセンサーの値をanonymous memoryで共有
        self.touch_sensor_memfd = mmap.mmap(-1, MMAP_SIZE)
        # 前進後退スピードの値をanonymous memoryで共有
        self.speed_memfd = mmap.mmap(-1, MMAP_SIZE)
        # 旋回スピードの値をanonymous memoryで共有
        self.steering_memfd = mmap.mmap(-1, MMAP_SIZE)

        self.motor_encoder_left_memfd = mmap.mmap(-1, MMAP_SIZE)
        self.motor_encoder_right_memfd = mmap.mmap(-1, MMAP_SIZE)

        # Runnerが準備済みかどうかをanonymous memoryで共有
        self.runner_is_ready_memfd = mmap.mmap(-1, MMAP_SIZE)
        # Guideが準備済みかどうかをanonymous memoryで共有
        self.guide_is_ready_memfd = mmap.mmap(-1, MMAP_SIZE)

        write_anon_mem(self.touch_sensor_memfd, 0)
        write_anon_mem(self.speed_memfd, 0)
        write_anon_mem(self.steering_memfd, 0)
        write_anon_mem(self.motor_encoder_left_memfd, 0)
        write_anon_mem(self.motor_encoder_right_memfd, 0)
        write_anon_mem(self.runner_is_ready_memfd, 0)
        write_anon_mem(self.guide_is_ready_memfd, 0)

    def read_touch_sensor_mem(self):
        return read_anon_mem(self.touch_sensor_memfd)

    def write_touch_sensor_mem(self, value):
        write_anon_mem(self.touch_sensor_memfd, value)

    def read_speed_mem(self):
        return read_anon_mem(self.speed_memfd)

    def write_speed_mem(self, value):
        write_anon_mem(self.speed_memfd, value)

    def read_steering_mem(self):
        return read_anon_mem(self.steering_memfd)

    def write_steering_mem(self, value):
        write_anon_mem(self.steering_memfd, value)

    def read_motor_encoder_left_mem(self):
        return read_anon_mem(self.motor_encoder_left_memfd)

    def write_motor_encoder_left_mem(self, value):
        write_anon_mem(self.motor_encoder_left_memfd, value)

    def read_motor_encoder_right_mem(self):
        return read_anon_mem(self.motor_encoder_right_memfd)

    def write_motor_encoder_right_mem(self, value):
        write_anon_mem(self.motor_encoder_right_memfd, value)

    def read_runner_is_ready_mem(self):
        return read_anon_mem(self.runner_is_ready_memfd)

    def write_runner_is_ready_mem(self, value):
        write_anon_mem(self.runner_is_ready_memfd, value)

    def read_guide_is_ready_mem(self):
        return read_anon_mem(self.guide_is_ready_memfd)

    def write_guide_is_ready_mem(self, value):
        write_anon_mem(self.guide_is_ready_memfd, value)
