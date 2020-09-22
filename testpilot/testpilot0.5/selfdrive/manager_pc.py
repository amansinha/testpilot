#!/usr/bin/env python2.7
import os
import sys
import fcntl
import errno
import signal
import subprocess

sys.path.append('../')
from common.basedir import BASEDIR
sys.path.append(os.path.join(BASEDIR, "pyextra"))
os.environ['BASEDIR'] = BASEDIR

def unblock_stdout():
  # get a non-blocking stdout
  child_pid, child_pty = os.forkpty()
  if child_pid != 0: # parent

    # child is in its own process group, manually pass kill signals
    signal.signal(signal.SIGINT, lambda signum, frame: os.kill(child_pid, signal.SIGINT))
    signal.signal(signal.SIGTERM, lambda signum, frame: os.kill(child_pid, signal.SIGTERM))

    fcntl.fcntl(sys.stdout, fcntl.F_SETFL,
       fcntl.fcntl(sys.stdout, fcntl.F_GETFL) | os.O_NONBLOCK)

    while True:
      try:
        dat = os.read(child_pty, 4096)
      except OSError as e:
        if e.errno == errno.EIO:
          break
        continue

      if not dat:
        break

      try:
        sys.stdout.write(dat)
      except (OSError, IOError):
        pass

    os._exit(os.wait()[1])

if __name__ == "__main__":
  unblock_stdout()

import glob
import shutil
import hashlib
import importlib
import subprocess
import traceback
from multiprocessing import Process

import zmq
from setproctitle import setproctitle  #pylint: disable=no-name-in-module

from common.params import Params
import cereal
ThermalStatus = cereal.log.ThermalData.ThermalStatus

from selfdrive.services import service_list
from selfdrive.swaglog import cloudlog
import selfdrive.messaging as messaging
from selfdrive.registration import register
from selfdrive.version import version, dirty
import selfdrive.crash as crash

from selfdrive.loggerd.config import ROOT

# comment out anything you don't want to run
managed_processes = {
#  "thermald": "selfdrive.thermald",
#  "uploader": "selfdrive.loggerd.uploader",
  "controlsd": "selfdrive.controls.controlsd_pc",
  "plannerd": "selfdrive.controls.plannerd_pc",
  "radard": "selfdrive.controls.radard_pc",
#  "ubloxd": "selfdrive.locationd.ubloxd",
#  "mapd": "selfdrive.mapd.mapd",
#  "loggerd": ("selfdrive/loggerd", ["./loggerd"]),
#  "logmessaged": "selfdrive.logmessaged",
#  "tombstoned": "selfdrive.tombstoned",
#  "logcatd": ("selfdrive/logcatd", ["./logcatd"]),
#  "proclogd": ("selfdrive/proclogd", ["./proclogd"]),
#  "boardd": ("selfdrive/boardd", ["./boardd"]),   # not used directly
#  "pandad": "selfdrive.pandad",
#  "ui": ("selfdrive/ui", ["./start.sh"]),
  "calibrationd": "selfdrive.locationd.calibrationd",
  "visiond": ("selfdrive/visiond", ["./visiond"]),
#  "sensord": ("selfdrive/sensord", ["./sensord"]),
#  "gpsd": ("selfdrive/sensord", ["./gpsd"]),
#  "updated": "selfdrive.updated",
"ui": "tools.replay.ui",
}

running = {}
def get_running():
  return running

# due to qualcomm kernel bugs SIGKILLing visiond sometimes causes page table corruption
unkillable_processes = ['visiond']

# processes to end with SIGINT instead of SIGTERM
interrupt_processes = []

persistent_processes = [
  'thermald',
  'logmessaged',
  'logcatd',
  'tombstoned',
  'uploader',
  'ui',
  'gpsd',
  'updated',
]

car_started_processes = [
  'controlsd',
  'plannerd',
  # 'loggerd',
  # 'sensord',
  'radard',
  'calibrationd',
  'visiond',
  'ui',
  # 'proclogd',
  # 'ubloxd',
  # 'mapd',
]

def register_managed_process(name, desc, car_started=False):
  global managed_processes, car_started_processes, persistent_processes
  print("registering %s" % name)
  managed_processes[name] = desc
  if car_started:
    car_started_processes.append(name)
  else:
    persistent_processes.append(name)

# ****************** process management functions ******************
def launcher(proc, gctx):
  try:
    # import the process
    mod = importlib.import_module(proc)

    # rename the process
    setproctitle(proc)

    # exec the process
    mod.main(gctx)
  except KeyboardInterrupt:
    cloudlog.warning("child %s got SIGINT" % proc)
  except Exception:
    # can't install the crash handler becuase sys.excepthook doesn't play nice
    # with threads, so catch it here.
    crash.capture_exception()
    raise

def nativelauncher(pargs, cwd):
  # exec the process
  os.chdir(cwd)

  # because when extracted from pex zips permissions get lost -_-
  os.chmod(pargs[0], 0o700)

  os.execvp(pargs[0], pargs)

def start_managed_process(name):
  if name in running or name not in managed_processes:
    return
  proc = managed_processes[name]
  if isinstance(proc, str):
    cloudlog.info("starting python %s" % proc)
    running[name] = Process(name=name, target=launcher, args=(proc, gctx))
  else:
    pdir, pargs = proc
    cwd = os.path.join(BASEDIR, pdir)
    cloudlog.info("starting process %s" % name)
    running[name] = Process(name=name, target=nativelauncher, args=(pargs, cwd))
  running[name].start()

def prepare_managed_process(p):
  proc = managed_processes[p]
  if isinstance(proc, str):
    # import this python
    cloudlog.info("preimporting %s" % proc)
    importlib.import_module(proc)
  else:
    # build this process
    cloudlog.info("building %s" % (proc,))
    try:
      subprocess.check_call(["make", "-j4"], cwd=os.path.join(BASEDIR, proc[0]))
    except subprocess.CalledProcessError:
      # make clean if the build failed
      cloudlog.warning("building %s failed, make clean" % (proc, ))
      subprocess.check_call(["make", "clean"], cwd=os.path.join(BASEDIR, proc[0]))
      subprocess.check_call(["make", "-j4"], cwd=os.path.join(BASEDIR, proc[0]))

def kill_managed_process(name):
  if name not in running or name not in managed_processes:
    return
  cloudlog.info("killing %s" % name)

  if running[name].exitcode is None:
    if name in interrupt_processes:
      os.kill(running[name].pid, signal.SIGINT)
    else:
      running[name].terminate()

    # give it 5 seconds to die
    running[name].join(5.0)
    if running[name].exitcode is None:
      if name in unkillable_processes:
        cloudlog.critical("unkillable process %s failed to exit! rebooting in 15 if it doesn't die" % name)
        running[name].join(15.0)
        if running[name].exitcode is None:
          cloudlog.critical("would have FORCE REBOOTed PHONE!")
          raise RuntimeError
      else:
        cloudlog.info("killing %s with SIGKILL" % name)
        os.kill(running[name].pid, signal.SIGKILL)
        running[name].join()

  cloudlog.info("%s is dead with %d" % (name, running[name].exitcode))
  del running[name]


def cleanup_all_processes(signal, frame):
  cloudlog.info("caught ctrl-c %s %s" % (signal, frame))


  for name in list(running.keys()):
    kill_managed_process(name)
  cloudlog.info("everything is dead")


# ****************** run loop ******************

def manager_init(should_register=True):
  global gctx

  if should_register:
    reg_res = register()
    if reg_res:
      dongle_id, dongle_secret = reg_res
    else:
      raise Exception("server registration failed")
  else:
    dongle_id = "c"*16

  # set dongle id
  cloudlog.info("dongle id is " + dongle_id)
  os.environ['DONGLE_ID'] = dongle_id

  cloudlog.info("dirty is %d" % dirty)
  if not dirty:
    os.environ['CLEAN'] = '1'

  cloudlog.bind_global(dongle_id=dongle_id, version=version, dirty=dirty, is_eon=True)
  crash.bind_user(id=dongle_id)
  crash.bind_extra(version=version, dirty=dirty, is_eon=True)

  os.umask(0)
  try:
    os.mkdir(ROOT, 0o777)
  except OSError:
    pass

  # set gctx
  gctx = {}


def manager_thread():
  # now loop
  context = zmq.Context()
  #thermal_sock = messaging.sub_sock(context, service_list['thermal'].port)

  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  # save boot log
  #subprocess.call(["./loggerd", "--bootlog"], cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))

  for p in car_started_processes:
    start_managed_process(p)

  while 1:
    pass
    # check the status of all processes, did any of them die?
    # for p in running:
      # cloudlog.debug("   running %s %s" % (p, running[p]))

def manager_prepare():
  # build cereal first
  subprocess.check_call(["make", "-j4"], cwd=os.path.join(BASEDIR, "cereal"))

  # build all processes
  os.chdir(os.path.dirname(os.path.abspath(__file__)))
  for p in managed_processes:
    prepare_managed_process(p)

def main():
  # params = Params()
  # params.manager_start()

  # # set unset params
  # if params.get("IsMetric") is None:
  #   params.put("IsMetric", "0")
  # if params.get("RecordFront") is None:
  #   params.put("RecordFront", "0")
  # if params.get("IsFcwEnabled") is None:
  #   params.put("IsFcwEnabled", "1")
  # if params.get("HasAcceptedTerms") is None:
  #   params.put("HasAcceptedTerms", "0")
  # if params.get("IsUploadVideoOverCellularEnabled") is None:
  #   params.put("IsUploadVideoOverCellularEnabled", "1")
  # if params.get("IsDriverMonitoringEnabled") is None:
  #   params.put("IsDriverMonitoringEnabled", "1")
  # if params.get("IsGeofenceEnabled") is None:
  #   params.put("IsGeofenceEnabled", "-1")
  # if params.get("SpeedLimitOffset") is None:
  #   params.put("SpeedLimitOffset", "0")
  # if params.get("LongitudinalControl") is None:
  #   params.put("LongitudinalControl", "0")
  # if params.get("LimitSetSpeed") is None:
  #   params.put("LimitSetSpeed", "0")

  try:
    manager_init(False)
    manager_prepare()
  finally:
    pass

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    crash.capture_exception()
  finally:
    cleanup_all_processes(None, None)

if __name__ == "__main__":
  main()
  # manual exit because we are forked
  sys.exit(0)
