import threading
import time
share = 0
def aa():
    global share
    lock.acquire()         # 鎖定
    i = 0
    while i<5:
        i = i + 1
        time.sleep(0.5)
        print('A:', i)
        if i==2:
            share = 99
            lock.release()  # i 等於 2 時解除鎖定

def bb():
    global share

    lock.acquire()          # 鎖定
    i = 0
    while i<50:
        i = i + 10
        time.sleep(0.5)
        print('B:', i)
        print(share)
    lock.release()

lock = threading.Lock()         # 建立 Lock
a = threading.Thread(target=aa)
b = threading.Thread(target=bb)

a.start()
b.start()
