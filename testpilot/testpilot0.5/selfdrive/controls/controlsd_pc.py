#!/usr/bin/env python
import gc
import zmq
import json
from cereal import car, log
from common.numpy_fast import clip
from common.realtime import sec_since_boot, set_realtime_priority, Ratekeeper
from common.profiler import Profiler
from common.params import Params
import selfdrive.messaging as messaging
from selfdrive.config import Conversions as CV
from selfdrive.services import service_list
from selfdrive.car.car_helpers import get_car
from selfdrive.controls.lib.drive_helpers import learn_angle_offset, \
                                                 get_events, \
                                                 create_event, \
                                                 EventTypes as ET, \
                                                 update_v_cruise, \
                                                 initialize_v_cruise
from selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState, STARTING_TARGET_SPEED
from selfdrive.controls.lib.latcontrol import LatControl
from selfdrive.controls.lib.alertmanager import AlertManager
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.driver_monitor import DriverStatus
from selfdrive.controls.lib.planner import _DT_MPC
from selfdrive.locationd.calibration_helpers import Calibration, Filter
from selfdrive.car.toyota.interface import CarInterface as ToyotaInterface
from selfdrive.car.toyota.values import CAR

ThermalStatus = log.ThermalData.ThermalStatus
State = log.Live100Data.ControlState


import numpy as np
def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(dtype = str(A.dtype), shape = A.shape)
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)


##### AS
def updateInternalCS(CS, vraw, steer_angle, steer_rate, cruise_speed_kph):
  CS.can_valid = True
  CS.cam_can_valid = True

  # update prevs, update must run once per loop
  CS.prev_left_blinker_on = False
  CS.prev_right_blinker_on = False

  CS.door_all_closed = True
  CS.seatbelt = True

  CS.brake_pressed = False
  if CS.CP.enableGasInterceptor:
    CS.pedal_gas = 0
  else:
    CS.pedal_gas = 0
  CS.car_gas = CS.pedal_gas
  CS.esp_disabled = False

  # calc best v_ego estimate, by averaging two opposite corners
  #self.v_wheel_fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
  #self.v_wheel_fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
  #self.v_wheel_rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
  #self.v_wheel_rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
  #self.v_wheel = float(np.mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr]))
  v_wheel_old = CS.v_wheel if hasattr(CS, 'v_wheel') else 0.0 
  CS.v_wheel = vraw

  # Kalman filter
  if abs(CS.v_wheel - CS.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
    CS.v_ego_kf.x = [[CS.v_wheel], [0.0]]
    v_wheel_old = CS.v_wheel

  CS.v_ego_raw = CS.v_wheel
  for val in np.linspace(v_wheel_old, CS.v_wheel,6)[1:]:
    v_ego_x = CS.v_ego_kf.update(val)
  CS.v_ego = float(v_ego_x[0])
  CS.a_ego = float(v_ego_x[1])
  CS.standstill = not CS.v_wheel > 0.001

  CS.angle_steers = steer_angle
  CS.angle_steers_rate = steer_rate
  #can_gear = int(cp.vl["GEAR_PACKET"]['GEAR'])
  CS.gear_shifter = 'drive'#parse_gear_shifter(can_gear, self.shifter_values)
  CS.main_on = True
  CS.left_blinker_on = False
  CS.right_blinker_on = False
  
  # we could use the override bit from dbc, but it's triggered at too high torque values
  CS.steer_override = False

  CS.user_brake = 0
  CS.v_cruise_pcm = cruise_speed_kph
  CS.pcm_acc_active = True
  CS.gas_pressed = False
  CS.low_speed_lockout = False
  CS.brake_lights = False
  if CS.CP.carFingerprint == CAR.PRIUS:
    CS.generic_toggle = False
  else:
    CS.generic_toggle = False


def returnNewCS(CI):
  canMonoTimes = []
  # create message
  ret = car.CarState.new_message()
  # speeds
  ret.vEgo = CI.CS.v_ego
  ret.vEgoRaw = CI.CS.v_ego_raw
  ret.aEgo = CI.CS.a_ego
  ret.yawRate = CI.VM.yaw_rate(CI.CS.angle_steers * CV.DEG_TO_RAD, CI.CS.v_ego)
  ret.standstill = CI.CS.standstill
  # ret.wheelSpeeds.fl = self.CS.v_wheel_fl
  # ret.wheelSpeeds.fr = self.CS.v_wheel_fr
  # ret.wheelSpeeds.rl = self.CS.v_wheel_rl
  # ret.wheelSpeeds.rr = self.CS.v_wheel_rr

  # gear shifter
  ret.gearShifter = CI.CS.gear_shifter

  # gas pedal
  ret.gas = CI.CS.car_gas
  ret.gasPressed = False

  # brake pedal
  ret.brake = CI.CS.user_brake
  ret.brakePressed = CI.CS.brake_pressed != 0
  ret.brakeLights = CI.CS.brake_lights

  # steering wheel
  ret.steeringAngle = CI.CS.angle_steers
  ret.steeringRate = CI.CS.angle_steers_rate

  # ret.steeringTorque = CI.CS.steer_torque_driver
  ret.steeringPressed = CI.CS.steer_override

  # cruise state
  ret.cruiseState.enabled = CI.CS.pcm_acc_active
  ret.cruiseState.speed = CI.CS.v_cruise_pcm * CV.KPH_TO_MS
  ret.cruiseState.available = bool(CI.CS.main_on)
  ret.cruiseState.speedOffset = 0.

  ret.cruiseState.standstill = False

  buttonEvents = []
  ret.buttonEvents = buttonEvents
  ret.leftBlinker = bool(CI.CS.left_blinker_on)
  ret.rightBlinker = bool(CI.CS.right_blinker_on)

  ret.doorOpen = not CI.CS.door_all_closed
  ret.seatbeltUnlatched = not CI.CS.seatbelt

  ret.genericToggle = CI.CS.generic_toggle

  # events
  events = []
  # enable request in prius is simple, as we activate when Toyota is active (rising edge)
  if ret.cruiseState.enabled and not CI.cruise_enabled_prev:
    events.append(create_event('pcmEnable', [ET.ENABLE]))
  ret.events = events
  ret.canMonoTimes = canMonoTimes

  CI.gas_pressed_prev = ret.gasPressed
  CI.brake_pressed_prev = ret.brakePressed
  CI.cruise_enabled_prev = ret.cruiseState.enabled

  return ret.as_reader()
##### As


def isActive(state):
  """Check if the actuators are enabled"""
  return state in [State.enabled, State.softDisabling]


def isEnabled(state):
  """Check if openpilot is engaged"""
  return (isActive(state) or state == State.preEnabled)


def data_sample(CI, CC, plan_sock, path_plan_sock, calibration, poller, cal_status, cal_perc, state, plan, path_plan, cruise_speed_kph):
  """Receive data from sockets and create events for battery, temperature and disk space"""

  # receive the values
  updateInternalCS(CI.CS, plan.plan.vTarget, path_plan.pathPlan.angleSteers, 0, cruise_speed_kph)
  CS = returnNewCS(CI)

  events = list(CS.events)

  # Receive from sockets
  cal = None
  for socket, event in poller.poll(0):
    if socket is calibration:
      cal = messaging.recv_one(socket)
    elif socket is plan_sock:
      plan = messaging.recv_one(socket)
    elif socket is path_plan_sock:
      path_plan = messaging.recv_one(socket)

  # Handle calibration
  if cal is not None:
    cal_status = cal.liveCalibration.calStatus
    cal_perc = cal.liveCalibration.calPerc

  # if cal_status != Calibration.CALIBRATED:
  #   if cal_status == Calibration.UNCALIBRATED:
  #     events.append(create_event('calibrationIncomplete', [ET.NO_ENTRY, ET.SOFT_DISABLE, ET.PERMANENT]))
  #   else:
  #     events.append(create_event('calibrationInvalid', [ET.NO_ENTRY, ET.SOFT_DISABLE]))

  return CS, events, cal_status, cal_perc, plan, path_plan


def state_transition(CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM):
  """Compute conditional state transitions and execute actions on state transitions"""
  enabled = isEnabled(state)

  v_cruise_kph_last = v_cruise_kph

  # if stock cruise is completely disabled, then we can use our own set speed logic
  if not CP.enableCruise:
    v_cruise_kph = update_v_cruise(v_cruise_kph, CS.buttonEvents, enabled)
  elif CP.enableCruise and CS.cruiseState.enabled:
    v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH

  # decrease the soft disable timer at every step, as it's reset on
  # entrance in SOFT_DISABLING state
  soft_disable_timer = max(0, soft_disable_timer - 1)

  # DISABLED
  if state == State.disabled:
    if get_events(events, [ET.ENABLE]):
      if get_events(events, [ET.NO_ENTRY]):
        for e in get_events(events, [ET.NO_ENTRY]):
          AM.add(str(e) + "NoEntry", enabled)

      else:
        if get_events(events, [ET.PRE_ENABLE]):
          state = State.preEnabled
        else:
          state = State.enabled
        AM.add("enable", enabled)
        v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, v_cruise_kph_last)

  # ENABLED
  elif state == State.enabled:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add("disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE]):
        AM.add(e, enabled)

    elif get_events(events, [ET.SOFT_DISABLE]):
      state = State.softDisabling
      soft_disable_timer = 300   # 3s
      for e in get_events(events, [ET.SOFT_DISABLE]):
        AM.add(e, enabled)

  # SOFT DISABLING
  elif state == State.softDisabling:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add("disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE]):
        AM.add(e, enabled)

    elif not get_events(events, [ET.SOFT_DISABLE]):
      # no more soft disabling condition, so go back to ENABLED
      state = State.enabled

    elif get_events(events, [ET.SOFT_DISABLE]) and soft_disable_timer > 0:
      for e in get_events(events, [ET.SOFT_DISABLE]):
        AM.add(e, enabled)

    elif soft_disable_timer <= 0:
      state = State.disabled

  # PRE ENABLING
  elif state == State.preEnabled:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add("disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE]):
        AM.add(e, enabled)

    elif not get_events(events, [ET.PRE_ENABLE]):
      state = State.enabled

  return state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last


def state_control(plan, path_plan, CS, CP, state, events, v_cruise_kph, v_cruise_kph_last, AM, rk,
                  LaC, LoC, VM, angle_offset, passive, is_metric, cal_perc):
  """Given the state, this function returns an actuators packet"""

  actuators = car.CarControl.Actuators.new_message()

  enabled = isEnabled(state)
  active = isActive(state)

  # send FCW alert if triggered by planner
  if plan.fcw:
    AM.add("fcw", enabled)

  # State specific actions

  if state in [State.preEnabled, State.disabled]:
    LaC.reset()
    LoC.reset(v_pid=CS.vEgo)

  elif state in [State.enabled, State.softDisabling]:
    # parse warnings from car specific interface
    for e in get_events(events, [ET.WARNING]):
      extra_text = ""
      if e == "belowSteerSpeed":
        if is_metric:
          extra_text = str(int(round(CP.minSteerSpeed * CV.MS_TO_KPH))) + " kph"
        else:
          extra_text = str(int(round(CP.minSteerSpeed * CV.MS_TO_MPH))) + " mph"
      AM.add(e, enabled, extra_text_2=extra_text)

  # Run angle offset learner at 20 Hz (##### AS ie every time now)
  #angle_offset = learn_angle_offset(active, CS.vEgo, angle_offset,
  #                                  path_plan.cPoly, path_plan.cProb, CS.steeringAngle,
  #                                  CS.steeringPressed)

  cur_time = sec_since_boot()  # TODO: This won't work in replay
  mpc_time = plan.l20MonoTime / 1e9
  ##### AS
  #_DT = 0.01 # 100Hz
  _DT = 0.05 # 20 HZ
  ##### AS

  dt = min(cur_time - mpc_time, _DT_MPC + _DT) + _DT  # no greater than dt mpc + dt, to prevent too high extraps
  a_acc_sol = plan.aStart + (dt / _DT_MPC) * (plan.aTarget - plan.aStart)
  v_acc_sol = plan.vStart + dt * (a_acc_sol + plan.aStart) / 2.0

  # Gas/Brake PID loop
  #actuators.gas, actuators.brake = LoC.update(active, CS.vEgo, CS.brakePressed, CS.standstill, CS.cruiseState.standstill,
  #                                            v_cruise_kph, v_acc_sol, plan.vTargetFuture, a_acc_sol, CP)
  # Steering PID loop and lateral MPC
  #actuators.steer, actuators.steerAngle = LaC.update(active, CS.vEgo, CS.steeringAngle,
  #                                                  CS.steeringPressed, CP, VM, path_plan)

  # Send a "steering required alert" if saturation count has reached the limit
  #if LaC.sat_flag and CP.steerLimitAlert:
  #  AM.add("steerSaturated", enabled)

  # Parse permanent warnings to display constantly
  for e in get_events(events, [ET.PERMANENT]):
    extra_text_1, extra_text_2 = "", ""
    if e == "calibrationIncomplete":
      extra_text_1 = str(cal_perc) + "%"
      if is_metric:
        extra_text_2 = str(int(round(Filter.MIN_SPEED * CV.MS_TO_KPH))) + " kph"
      else:
        extra_text_2 = str(int(round(Filter.MIN_SPEED * CV.MS_TO_MPH))) + " mph"
    AM.add(str(e) + "Permanent", enabled, extra_text_1=extra_text_1, extra_text_2=extra_text_2)

  AM.process_alerts(sec_since_boot())

  return actuators, v_cruise_kph, angle_offset, v_acc_sol, a_acc_sol


def data_send(plan, path_plan, CS, CI, CP, VM, state, events, actuators, v_cruise_kph, rk, carstate,
              carcontrol, live100, AM,
              LaC, LoC, angle_offset, passive, start_time, v_acc, a_acc, carla_socket):
  """Send actuators and hud commands to the car, send live100 and MPC logging"""
  plan_ts = plan.logMonoTime
  plan = plan.plan

  CC = car.CarControl.new_message()

  if not passive:
    CC.enabled = isEnabled(state)
    CC.actuators = actuators

    CC.cruiseControl.override = True
    CC.cruiseControl.cancel = not CP.enableCruise or (not isEnabled(state) and CS.cruiseState.enabled)

    # Some override values for Honda
    brake_discount = (1.0 - clip(actuators.brake * 3., 0.0, 1.0))  # brake discount removes a sharp nonlinearity
    CC.cruiseControl.speedOverride = float(max(0.0, (LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount) if CP.enableCruise else 0.0)
    CC.cruiseControl.accelOverride = CI.calc_accel_override(CS.aEgo, plan.aTarget, CS.vEgo, plan.vTarget)

    CC.hudControl.setSpeed = float(v_cruise_kph * CV.KPH_TO_MS)
    CC.hudControl.speedVisible = isEnabled(state)
    CC.hudControl.lanesVisible = isEnabled(state)
    CC.hudControl.leadVisible = plan.hasLead
    CC.hudControl.rightLaneVisible = bool(path_plan.pathPlan.rProb > 0.5)
    CC.hudControl.leftLaneVisible = bool(path_plan.pathPlan.lProb > 0.5)
    CC.hudControl.visualAlert = AM.visual_alert
    CC.hudControl.audibleAlert = AM.audible_alert

    # send car controls over can
    CI.apply(CC)

  force_decel = False

  # live100
  dat = messaging.new_message()
  dat.init('live100')
  dat.live100 = {
    "alertText1": AM.alert_text_1,
    "alertText2": AM.alert_text_2,
    "alertSize": AM.alert_size,
    "alertStatus": AM.alert_status,
    "alertBlinkingRate": AM.alert_rate,
    "alertType": AM.alert_type,
    "alertSound": "",  # no EON sounds yet
    "awarenessStatus": max(1.0, 0.0) if isEnabled(state) else 0.0,
    "driverMonitoringOn": True,
    "canMonoTimes": list(CS.canMonoTimes),
    "planMonoTime": plan_ts,
    "pathPlanMonoTime": path_plan.logMonoTime,
    "enabled": isEnabled(state),
    "active": isActive(state),
    "vEgo": CS.vEgo,
    "vEgoRaw": CS.vEgoRaw,
    "angleSteers": CS.steeringAngle,
    "curvature": VM.calc_curvature(CS.steeringAngle * CV.DEG_TO_RAD, CS.vEgo),
    "steerOverride": CS.steeringPressed,
    "state": state,
    "engageable": not bool(get_events(events, [ET.NO_ENTRY])),
    "longControlState":  LongCtrlState.pid, #LoC.long_control_state,
    "vPid": float(LoC.v_pid),
    "vCruise": float(v_cruise_kph),
    "upAccelCmd": float(LoC.pid.p),
    "uiAccelCmd": float(LoC.pid.i),
    "ufAccelCmd": float(LoC.pid.f),
    "angleSteersDes": float(LaC.angle_steers_des),
    "upSteer": float(LaC.pid.p),
    "uiSteer": float(LaC.pid.i),
    "ufSteer": float(LaC.pid.f),
    "vTargetLead": float(v_acc),
    "aTarget": float(a_acc),
    "jerkFactor": float(plan.jerkFactor),
    "angleOffset": float(angle_offset),
    "gpsPlannerActive": plan.gpsPlannerActive,
    "vCurvature": plan.vCurvature,
    "decelForTurn": plan.decelForTurn,
    "cumLagMs": -rk.remaining * 1000.,
    "startMonoTime": start_time,
    "mapValid": plan.mapValid,
    "forceDecel": bool(force_decel),
  }
  live100.send(dat.to_bytes())

  # carState
  cs_send = messaging.new_message()
  cs_send.init('carState')
  cs_send.carState = CS
  cs_send.carState.events = events
  carstate.send(cs_send.to_bytes())

  ##### AS
   #send CS.steeringAngle, CS.vEgoRaw to carla
  send_array(carla_socket, np.array([CS.vEgoRaw, CS.steeringAngle]))
  carla_socket.recv()
  ##### AS
  # carControl
  # cc_send = messaging.new_message()
  # cc_send.init('carControl')
  # cc_send.carControl = CC
  # carcontrol.send(cc_send.to_bytes())

  # if (rk.frame % 36000) == 0:    # update angle offset every 6 minutes
  #   params.put("ControlsParams", json.dumps({'angle_offset': angle_offset}))

  return CC


def controlsd_thread(gctx=None, rate=100):
  gc.disable()

  # start the loop
  set_realtime_priority(3)

  ##### AS
  context = zmq.Context()
  live100 = messaging.pub_sock(context, service_list['live100'].port)
  carstate = messaging.pub_sock(context, service_list['carState'].port)
  carcontrol = messaging.pub_sock(context, service_list['carControl'].port)
  carla_socket = context.socket(zmq.PAIR)
  carla_socket.bind("tcp://*:5560")

  is_metric = True
  passive = True
  ##### AS

  # No sendcan if passive
  if not passive:
    sendcan = messaging.pub_sock(context, service_list['sendcan'].port)
  else:
    sendcan = None

  # Sub sockets
  poller = zmq.Poller()
  #thermal = messaging.sub_sock(context, service_list['thermal'].port, conflate=True, poller=poller)
  #health = messaging.sub_sock(context, service_list['health'].port, conflate=True, poller=poller)
  cal = messaging.sub_sock(context, service_list['liveCalibration'].port, conflate=True, poller=poller)
  #driver_monitor = messaging.sub_sock(context, service_list['driverMonitoring'].port, conflate=True, poller=poller)
  plan_sock = messaging.sub_sock(context, service_list['plan'].port, conflate=True, poller=poller)
  path_plan_sock = messaging.sub_sock(context, service_list['pathPlan'].port, conflate=True, poller=poller)
  #logcan = messaging.sub_sock(context, service_list['can'].port)

  CC = car.CarControl.new_message()
  CP = ToyotaInterface.get_params("TOYOTA PRIUS 2017", {})
  CP.steerRatio = 1.0
  CI = ToyotaInterface(CP, sendcan)

  if CI is None:
    raise Exception("unsupported car")

  # if stock camera is connected, then force passive behavior
  if not CP.enableCamera:
    passive = True
    sendcan = None

  if passive:
    CP.safetyModel = car.CarParams.SafetyModels.noOutput

  LoC = LongControl(CP, CI.compute_gb)
  VM = VehicleModel(CP)
  LaC = LatControl(CP)
  AM = AlertManager()

  if not passive:
    AM.add("startup", False)

  state = State.enabled
  soft_disable_timer = 0
  v_cruise_kph = 50 ##### !!! change
  v_cruise_kph_last = 0 ##### !! change
  cal_status = Calibration.INVALID
  cal_perc = 0

  plan = messaging.new_message()
  plan.init('plan')
  path_plan = messaging.new_message()
  path_plan.init('pathPlan')

  rk = Ratekeeper(rate, print_delay_threshold=2. / 1000)
  angle_offset = 0.

  prof = Profiler(False)  # off by default

  startup = True ##### AS
  while True:
    start_time = int(sec_since_boot() * 1e9)
    prof.checkpoint("Ratekeeper", ignore=True)

    # Sample data and compute car events
    CS, events, cal_status, cal_perc, plan, path_plan  =\
      data_sample(CI, CC, plan_sock, path_plan_sock, cal, poller, cal_status, cal_perc, state, plan, path_plan, v_cruise_kph)
    prof.checkpoint("Sample")

    ##### AS since we dont do preenabled state
    if startup:
      LaC.reset()
      LoC.reset(v_pid=CS.vEgo)
      if cal_status != Calibration.CALIBRATED:
        continue
      startup = False

    path_plan_age = (start_time - path_plan.logMonoTime) / 1e9
    plan_age = (start_time - plan.logMonoTime) / 1e9
    if not path_plan.pathPlan.valid or plan_age > 0.5 or path_plan_age > 0.5:
      print 'planner time too long'
      #events.append(create_event('plannerError', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    events += list(plan.plan.events)

    # Only allow engagement with brake pressed when stopped behind another stopped car
    #if CS.brakePressed and plan.plan.vTargetFuture >= STARTING_TARGET_SPEED and not CP.radarOffCan and CS.vEgo < 0.3:
    #  events.append(create_event('noTarget', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

    if not passive:
      # update control state
      state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last = \
        state_transition(CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM)
      prof.checkpoint("State transition")

    # Compute actuators (runs PID loops and lateral MPC)
    actuators, v_cruise_kph, angle_offset, v_acc, a_acc = \
      state_control(plan.plan, path_plan.pathPlan, CS, CP, state, events, v_cruise_kph,
                    v_cruise_kph_last, AM, rk,
                    LaC, LoC, VM, angle_offset, passive, is_metric, cal_perc)

    prof.checkpoint("State Control")

    # Publish data
    CC = data_send(plan, path_plan, CS, CI, CP, VM, state, events, actuators, v_cruise_kph, rk, carstate, carcontrol,
                   live100, AM, LaC, LoC, angle_offset, passive, start_time, v_acc, a_acc, carla_socket)
    prof.checkpoint("Sent")

    rk.keep_time()  # Run at 100Hz, no 20 Hz
    prof.display()


def main(gctx=None):
  controlsd_thread(gctx, 20)


if __name__ == "__main__":
  main()
