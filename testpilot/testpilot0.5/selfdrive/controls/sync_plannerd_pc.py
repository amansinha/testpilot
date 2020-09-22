#!/usr/bin/env python
import zmq

from cereal import car
from common.params import Params
from common.realtime import sec_since_boot
from selfdrive.swaglog import cloudlog
from selfdrive.services import service_list
from selfdrive.controls.lib.planner_pc import Planner
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.pathplanner_pc import PathPlanner
import selfdrive.messaging as messaging
from selfdrive.car.toyota.interface import CarInterface as ToyotaInterface


def plannerd_thread():
  ##### AS
  # context = zmq.Context()
  # params = Params()

  # # Get FCW toggle from settings
  # fcw_enabled = params.get("IsFcwEnabled") == "1"

  # cloudlog.info("plannerd is waiting for CarParams")
  # CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  # cloudlog.info("plannerd got CarParams: %s", CP.carName)
  context = zmq.Context()
  fcw_enabled = False
  cloudlog.info("plannerd is waiting for CarParams")
  CP = ToyotaInterface.get_params("TOYOTA PRIUS 2017", {})
  CP.steerRatio = 1.0
  cloudlog.info("plannerd got CarParams: %s", CP.carName)
  ##### AS

  PL = Planner(CP, fcw_enabled)
  PP = PathPlanner(CP)

  VM = VehicleModel(CP)

  poller = zmq.Poller()
  car_state_sock = messaging.sub_sock(context, service_list['carState'].port, conflate=True, poller=poller)
  live100_sock = messaging.sub_sock(context, service_list['live100'].port, conflate=True, poller=poller)
  live20_sock = messaging.sub_sock(context, service_list['live20'].port, conflate=True, poller=poller)
  model_sock = messaging.sub_sock(context, service_list['model'].port, conflate=True, poller=poller)
  live_map_data_sock = messaging.sub_sock(context, service_list['liveMapData'].port, conflate=True, poller=poller)

  car_state = messaging.new_message()
  car_state.init('carState')
  live100 = messaging.new_message()
  live100.init('live100')
  model = messaging.new_message()
  model.init('model')
  live20 = messaging.new_message()
  live20.init('live20')
  live_map_data = messaging.new_message()
  live_map_data.init('liveMapData')

  startup = True
  while True:
    recv_live100 = False
    recv_carstate = False
    recv_model = False
    recv_live20 = False
    while not (recv_live100 and recv_carstate and recv_model and recv_live20):
      for socket, event in poller.poll():
        if socket is live100_sock:
          live100 = messaging.recv_one(socket)
          recv_live100 = True
        elif socket is car_state_sock:
          car_state = messaging.recv_one(socket)
          recv_carstate = True
          if startup:
            reset_speed = car_state.carState.vEgo
            reset_accel = min(car_state.carState.aEgo, 0.0)
            PL.v_acc = reset_speed
            PL.a_acc = reset_accel
            PL.v_acc_start = reset_speed
            PL.a_acc_start = reset_accel
            PL.v_cruise = reset_speed
            PL.a_cruise = reset_accel
            startup = False
        elif socket is model_sock:
          model = messaging.recv_one(socket)
          recv_model = True
        elif socket is live_map_data_sock:
          live_map_data = messaging.recv_one(socket)
        elif socket is live20_sock:
          live20 = messaging.recv_one(socket)
          recv_live20 = True
        if recv_model and recv_live100 and recv_carstate:
          PP.update(CP, VM, car_state, model, live100)
    PL.update(car_state, CP, VM, PP, live20, live100, model, live_map_data)


def main(gctx=None):
  if not hasattr(sec_since_boot, 'frames_received'):
    sec_since_boot.frames_received = gctx['timer']
  plannerd_thread()


if __name__ == "__main__":
  main()
