#!/usr/bin/env python3
import argparse
import os
import sys
from typing import Any

from selfdrive.car.car_helpers import interface_names
from selfdrive.test.openpilotci import get_url
from selfdrive.test.process_replay.compare_logs import compare_logs
from selfdrive.test.process_replay.process_replay import CONFIGS, replay_process, check_enabled
from tools.lib.logreader import LogReader


source_segments = [
  ("BODY", "937ccb7243511b65|2022-05-24--16-03-09--1"),        # COMMA.BODY
  ("HYUNDAI", "02c45f73a2e5c6e9|2021-01-01--19-08-22--1"),     # HYUNDAI.SONATA
  ("HYUNDAI", "d824e27e8c60172c|2022-09-13--11-26-50--2"),     # HYUNDAI.KIA_EV6
  ("TOYOTA", "0982d79ebb0de295|2021-01-04--17-13-21--13"),     # TOYOTA.PRIUS (INDI)
  ("TOYOTA2", "0982d79ebb0de295|2021-01-03--20-03-36--6"),     # TOYOTA.RAV4  (LQR)
  ("TOYOTA3", "f7d7e3538cda1a2a|2021-08-16--08-55-34--6"),     # TOYOTA.COROLLA_TSS2
  ("HONDA", "eb140f119469d9ab|2021-06-12--10-46-24--27"),      # HONDA.CIVIC (NIDEC)
  ("HONDA2", "7d2244f34d1bbcda|2021-06-25--12-25-37--26"),     # HONDA.ACCORD (BOSCH)
  ("CHRYSLER", "4deb27de11bee626|2021-02-20--11-28-55--8"),    # CHRYSLER.PACIFICA
  ("RAM", "2f4452b03ccb98f0|2022-09-07--13-55-08--10"),        # CHRYSLER.RAM_1500
  ("SUBARU", "341dccd5359e3c97|2022-09-12--10-35-33--3"),      # SUBARU.OUTBACK
  ("GM", "0c58b6a25109da2b|2021-02-23--16-35-50--11"),         # GM.VOLT
  ("NISSAN", "35336926920f3571|2021-02-12--18-38-48--46"),     # NISSAN.XTRAIL
  ("VOLKSWAGEN", "de9592456ad7d144|2021-06-29--11-00-15--6"),  # VOLKSWAGEN.GOLF
  ("MAZDA", "bd6a637565e91581|2021-10-30--15-14-53--4"),       # MAZDA.CX9_2021

  # Enable when port is tested and dashcamOnly is no longer set
  #("TESLA", "bb50caf5f0945ab1|2021-06-19--17-20-18--3"),      # TESLA.AP2_MODELS
  #("VOLKSWAGEN2", "3cfdec54aa035f3f|2022-07-19--23-45-10--2"),  # VOLKSWAGEN.PASSAT_NMS
]

segments = [
  ("BODY", "regen9D38397D30D|2022-09-09--13-12-48--0"),
  ("HYUNDAI", "regenB3953B393C0|2022-09-09--14-49-37--0"),
  ("HYUNDAI", "regen8DB830E5376|2022-09-13--17-24-37--0"),
  ("TOYOTA", "regen8FCBB6F06F1|2022-09-09--13-14-07--0"),
  ("TOYOTA2", "regen956BFA75300|2022-09-09--14-51-24--0"),
  ("TOYOTA3", "regenE909BC2F430|2022-09-09--20-44-49--0"),
  ("HONDA", "regenD1D10209015|2022-09-09--14-53-09--0"),
  ("HONDA2", "regen3F7C2EFDC08|2022-09-09--19-41-19--0"),
  ("CHRYSLER", "regen92783EAE66B|2022-09-09--13-15-44--0"),
  ("RAM", "regenBE5DAAEF30F|2022-09-13--17-06-24--0"),
  ("SUBARU", "regen8A363AF7E14|2022-09-13--17-20-39--0"),
  ("GM", "regen31EA3F9A37C|2022-09-09--21-06-36--0"),
  ("NISSAN", "regenAA21ADE5921|2022-09-09--19-44-37--0"),
  ("VOLKSWAGEN", "regenA1BF4D17761|2022-09-09--19-46-24--0"),
  ("MAZDA", "regen1994C97E977|2022-09-13--16-34-44--0"),
]

# dashcamOnly makes don't need to be tested until a full port is done
excluded_interfaces = ["mock", "ford", "mazda", "tesla"]

BASE_URL = "https://commadataci.blob.core.windows.net/openpilotci/"

# run the full test (including checks) when no args given
FULL_TEST = len(sys.argv) <= 1


def test_process(cfg, lr, cmp_log_fn, ignore_fields=None, ignore_msgs=None):
  if ignore_fields is None:
    ignore_fields = []
  if ignore_msgs is None:
    ignore_msgs = []

  cmp_log_path = cmp_log_fn if os.path.exists(cmp_log_fn) else BASE_URL + os.path.basename(cmp_log_fn)
  cmp_log_msgs = list(LogReader(cmp_log_path))

  log_msgs = replay_process(cfg, lr)

  # check to make sure openpilot is engaged in the route
  if cfg.proc_name == "controlsd":
    if not check_enabled(log_msgs):
      segment = cmp_log_fn.split("/")[-1].split("_")[0]
      raise Exception(f"Route never enabled: {segment}")

  try:
    return compare_logs(cmp_log_msgs, log_msgs, ignore_fields+cfg.ignore, ignore_msgs, cfg.tolerance)
  except Exception as e:
    return str(e)

def format_diff(results, ref_commit):
  diff1, diff2 = "", ""
  diff2 += f"***** tested against commit {ref_commit} *****\n"

  failed = False
  for segment, result in list(results.items()):
    diff1 += f"***** results for segment {segment} *****\n"
    diff2 += f"***** differences for segment {segment} *****\n"

    for proc, diff in list(result.items()):
      diff1 += f"\t{proc}\n"
      diff2 += f"*** process: {proc} ***\n"

      if isinstance(diff, str):
        diff1 += f"\t\t{diff}\n"
        failed = True
      elif len(diff):
        cnt = {}
        for d in diff:
          diff2 += f"\t{str(d)}\n"

          k = str(d[1])
          cnt[k] = 1 if k not in cnt else cnt[k] + 1

        for k, v in sorted(cnt.items()):
          diff1 += f"\t\t{k}: {v}\n"
        failed = True
  return diff1, diff2, failed

if __name__ == "__main__":

  parser = argparse.ArgumentParser(description="Regression test to identify changes in a process's output")

  # whitelist has precedence over blacklist in case both are defined
  parser.add_argument("--whitelist-procs", type=str, nargs="*", default=[],
                        help="Whitelist given processes from the test (e.g. controlsd)")
  parser.add_argument("--whitelist-cars", type=str, nargs="*", default=[],
                        help="Whitelist given cars from the test (e.g. HONDA)")
  parser.add_argument("--blacklist-procs", type=str, nargs="*", default=[],
                        help="Blacklist given processes from the test (e.g. controlsd)")
  parser.add_argument("--blacklist-cars", type=str, nargs="*", default=[],
                        help="Blacklist given cars from the test (e.g. HONDA)")
  parser.add_argument("--ignore-fields", type=str, nargs="*", default=[],
                        help="Extra fields or msgs to ignore (e.g. carState.events)")
  parser.add_argument("--ignore-msgs", type=str, nargs="*", default=[],
                        help="Msgs to ignore (e.g. carEvents)")
  args = parser.parse_args()

  cars_whitelisted = len(args.whitelist_cars) > 0
  procs_whitelisted = len(args.whitelist_procs) > 0

  process_replay_dir = os.path.dirname(os.path.abspath(__file__))
  try:
    ref_commit = open(os.path.join(process_replay_dir, "ref_commit")).read().strip()
  except FileNotFoundError:
    print("couldn't find reference commit")
    sys.exit(1)

  print(f"***** testing against commit {ref_commit} *****")

  # check to make sure all car brands are tested
  if FULL_TEST:
    tested_cars = {c.lower() for c, _ in segments}
    untested = (set(interface_names) - set(excluded_interfaces)) - tested_cars
    assert len(untested) == 0, f"Cars missing routes: {str(untested)}"

  results: Any = {}
  for car_brand, segment in segments:
    if (cars_whitelisted and car_brand.upper() not in args.whitelist_cars) or \
       (not cars_whitelisted and car_brand.upper() in args.blacklist_cars):
      continue

    print(f"***** testing route segment {segment} *****\n")

    results[segment] = {}

    r, n = segment.rsplit("--", 1)
    lr = LogReader(get_url(r, n))

    for cfg in CONFIGS:
      if (procs_whitelisted and cfg.proc_name not in args.whitelist_procs) or \
         (not procs_whitelisted and cfg.proc_name in args.blacklist_procs):
        continue

      cmp_log_fn = os.path.join(process_replay_dir, f"{segment}_{cfg.proc_name}_{ref_commit}.bz2")
      results[segment][cfg.proc_name] = test_process(cfg, lr, cmp_log_fn, args.ignore_fields, args.ignore_msgs)

  diff1, diff2, failed = format_diff(results, ref_commit)
  with open(os.path.join(process_replay_dir, "diff.txt"), "w") as f:
    f.write(diff2)
  print(diff1)

  if failed:
    print("TEST FAILED")
    print("\n\nTo update the reference logs for this test run:")
    print("./update_refs.py")
  else:
    print("TEST SUCCEEDED")

  sys.exit(int(failed))
