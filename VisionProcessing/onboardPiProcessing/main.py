import threading
import queue

q = queue.Queue()

def worker():
    while True:
        item = q.get()
        print(f'Working on {item}')
        print(f'Finished {item}')
        q.task_done()

# Turn-on the worker thread.
t = threading.Thread(target=worker, daemon=True).start()

# Send thirty task requests to the worker.
for item in range(2000):
    q.put(item)

# Block until all tasks are done.
while True:
    pass