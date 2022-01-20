#!/usr/bin/env python3
import os
import random
import time

def log(msg):
  os.system(f"log -t crasher '{msg}'")

def crasher():
  cnt = 0
  start = time.monotonic()
  while True:
    # TODO: also do tethering

    cnt += 1
    d = random.choice(['enable', 'disable'])
    w = random.choice(['enable', 'disable'])

    log(f"#{str(cnt).ljust(4)}: data={d} wifi={w}, {round(time.monotonic() - start)}s")
    print(f"#{str(cnt).ljust(4)}: data={d} wifi={w}, {round(time.monotonic() - start)}s")

    os.system(f"LD_LIBRARY_PATH= svc data {d}")
    time.sleep(random.uniform(0., 1.))

    os.system(f"LD_LIBRARY_PATH= svc wifi {w}")
    time.sleep(random.uniform(10., 5 * 60.))


if __name__ == "__main__":
  crasher()
