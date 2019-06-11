from functools import wraps
from time import clock

def timefn(fn):
    @wraps(fn)
    def measure_time(*args, **kwargs):
        t1 = clock()
        result = fn(*args, **kwargs)
        t2 = clock()

        # python2
        #print ("@timefn:" + fn.func_name + " took " + str(t2 - t1) + " seconds")
        
        # python3
        print ("@timefn:" + fn.__name__ + " took " + "{0:f}".format((t2 - t1) * 1000) + " milli seconds")
        return result
    return measure_time
