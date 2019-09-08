import time


def main():
    logs = ["" for _ in range(10000)]
    log_pointer = 0
    time_time = 0

    is_going = True


    while is_going:
        time_time = time.perf_counter()

        logs[log_pointer] = time_time
        log_pointer += 1

        if(log_pointer >=10000):
            is_going = False

    log_file = open('./log/time_perf_counter.csv','w')
    for log in logs:
        if log != "":
            log_file.write("{}\n".format(log))
    log_file.close()


if __name__ == '__main__':
    main()
