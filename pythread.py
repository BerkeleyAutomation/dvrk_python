import threading, time, signal
from datetime import timedelta

WAIT_TIME_MS = 10

class ProgramKilled(Exception):
    pass

t = 0.0;
count = 0.0

def foo():
    global t
    global count
    t = t + 0.01;
    if t > 1:
        t = 0.0
        count += 1
        print count

def signal_handler(signum, frame):
    raise ProgramKilled


class Job(threading.Thread):
    def __init__(self, interval, execute, *args, **kwargs):
        threading.Thread.__init__(self)
        self.daemon = False
        self.stopped = threading.Event()
        self.interval = interval
        self.execute = execute
        self.args = args
        self.kwargs = kwargs

    def stop(self):
        self.stopped.set()
        self.join()

    def run(self):
        while not self.stopped.wait(self.interval.total_seconds()):
            self.execute(*self.args, **self.kwargs)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    job = Job(interval=timedelta(milliseconds=WAIT_TIME_MS), execute=foo)
    job.start()

    while True:
        try:
            time.sleep(1)
        except ProgramKilled:
            print "Program killed: running cleanup code"
            job.stop()
            break