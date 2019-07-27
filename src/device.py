########################################################################
##
## File I/O functions
##
########################################################################

# Function for fast reading from sensor files
def read_device(fd):
    fd.seek(0)
    return int(fd.read().decode().strip())

# Function for fast writing to motor files
def write_device(fd, value):
    fd.truncate(0)
    fd.write(str(int(value)))
    fd.flush()

# Function to set the duty cycle of the motors
def set_duty(motor_duty_devfd, duty, barance=1):
    # Clamp the value between -100 and 100
    duty = min(max(duty, -100), 100) * barance
    # Apply the signal to the motor
    
    write_device(motor_duty_devfd, duty)
    return duty
