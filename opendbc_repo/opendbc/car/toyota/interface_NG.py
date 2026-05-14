from opendbc.car import Bus, structs, get_safety_config, uds
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, TSS2_CAR, RADAR_ACC_CAR, NO_DSU_CAR, \
                                                  MIN_ACC_SPEED, EPS_SCALE, NO_STOP_TIMER_CAR, ANGLE_CONTROL_CAR, \
                                                  ToyotaSafetyFlags, UNSUPPORTED_DSU_CAR
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.interfaces import CarInterfaceBase

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, dp_params, docs) -> structs.CarParams:
    ret.brand = "toyota"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = EPS_SCALE[candidate]

    if candidate in UNSUPPORTED_DSU_CAR:
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.UNSUPPORTED_DSU.value

    # BRAKE_MODULE is on a different address for these cars
    if DBC[candidate][Bus.pt] == "toyota_new_mc_pt_generated":
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.ALT_BRAKE.value

    if ret.flags & ToyotaFlags.SECOC.value:
      ret.secOcRequired = True
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.SECOC.value
      ret.dashcamOnly = is_release

    if candidate in ANGLE_CONTROL_CAR:
      ret.steerControlType = SteerControlType.angle
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.LTA.value

      # LTA control can be more delayed and winds up more often
      ret.steerActuatorDelay = 0.18
      ret.steerLimitTimer = 0.8
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
      ret.steerLimitTimer = 0.4

    stop_and_go = candidate in TSS2_CAR

    # In TSS2 cars, the camera does long control
    found_ecus = [fw.ecu for fw in car_fw]

    if Ecu.hybrid in found_ecus:
      ret.flags |= ToyotaFlags.HYBRID.value

    # 0x343 should not be present on bus 2 on cars other than TSS2_CAR unless we are re-routing DSU
    dsu_bypass = False
    if (0x343 in fingerprint[2] or 0x4CB in fingerprint[2]) and candidate not in TSS2_CAR:
      print("----------------------------------------------")
      print("dragonpilot: DSU_BYPASS detected!")
      print("----------------------------------------------")
      # rick - disable for now, breaks TOYOTA_AVALON_2019 model tests.
      # dsu_bypass = True
      # ret.flags |= ToyotaFlags.DSU_BYPASS.value

    if 0x23 in fingerprint[0]:
      print("----------------------------------------------")
      print("dragonpilot: ZSS detected!")
      print("----------------------------------------------")
      ret.flags |= ToyotaFlags.ZSS.value

    if candidate == CAR.TOYOTA_PRIUS:
      stop_and_go = True
      # Only give steer angle deadzone to for bad angle sensor prius
      for fw in car_fw:
        if fw.ecu == "eps" and not fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00':
          if ret.flags & ToyotaFlags.ZSS.value:
            CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
          else:
            ret.steerActuatorDelay = 0.25
            CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)

    elif candidate in (CAR.LEXUS_RX, CAR.LEXUS_RX_TSS2):
      stop_and_go = True
      ret.wheelSpeedFactor = 1.035

    elif candidate in (CAR.TOYOTA_AVALON, CAR.TOYOTA_AVALON_2019, CAR.TOYOTA_AVALON_TSS2):
      # starting from 2019, all Avalon variants have stop and go
      # https://engage.toyota.com/static/images/toyota_safety_sense/TSS_Applicability_Chart.pdf
      stop_and_go = candidate != CAR.TOYOTA_AVALON

    elif candidate in (CAR.TOYOTA_RAV4_TSS2, CAR.TOYOTA_RAV4_TSS2_2022, CAR.TOYOTA_RAV4_TSS2_2023, CAR.TOYOTA_RAV4_PRIME, CAR.TOYOTA_SIENNA_4TH_GEN):
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP = [0.0]
      ret.lateralTuning.pid.kpBP = [0.0]
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.1]
      ret.lateralTuning.pid.kf = 0.00007818594

      # 2019+ RAV4 TSS2 uses two different steering racks and specific tuning seems to be necessary.
      # See https://github.com/commaai/openpilot/pull/21429#issuecomment-873652891
      for fw in car_fw:
        if fw.ecu == "eps" and (fw.fwVersion.startswith(b'\x02') or fw.fwVersion in [b'8965B42181\x00\x00\x00\x00\x00\x00']):
          ret.lateralTuning.pid.kpV = [0.15]
          ret.lateralTuning.pid.kiV = [0.05]
          ret.lateralTuning.pid.kf = 0.00004
          break

    # ==============================================================================
    # 🌟 新增：Corolla TSS2 橫向動態絲滑調校 (解決階梯感 + 防止直線畫龍)
    # ==============================================================================
    elif candidate == CAR.TOYOTA_COROLLA_TSS2:
      ret.lateralTuning.init('pid')
      
      # 啟用車速分段 (Breakpoints)，單位為 m/s。
      # [0.0, 15.0, 30.0] 分別代表時速 0 km/h, 54 km/h, 108 km/h
      ret.lateralTuning.pid.kpBP = [0.0, 15.0, 30.0]
      ret.lateralTuning.pid.kiBP = [0.0, 15.0, 30.0]
      
      # 動態 P 值 (kpV)：
      # 中低速維持 0.08~0.1 確保過彎無階梯感；高速直線拉高到 0.12 來穩住中線，防止偏航
      ret.lateralTuning.pid.kpV = [0.06, 0.10, 0.12]
      
      # 動態 I 值 (kiV)：
      # 高速時降到極低 (0.01)，避免誤差在直線上累積導致鐘擺效應 (畫龍)
      ret.lateralTuning.pid.kiV = [0.02, 0.01, 0.005]
      
      # 前饋 (kf)：稍微降回 0.00005，避免模型在直線上的微小預測噪聲被放大
      ret.lateralTuning.pid.kf = 0.00008
      
      # 致動器延遲：調回預設的 0.12。延遲過高是直線乒乓的主因！
      ret.steerActuatorDelay = 0.10
    # ==============================================================================

    elif candidate in (CAR.TOYOTA_CHR, CAR.TOYOTA_CAMRY, CAR.TOYOTA_SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_NX):
      # TODO: Some of these platforms are not advertised to have full range ACC, do they really all have sng?
      stop_and_go = True

    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: Some TSS-P platforms have BSM, but are flipped based on region or driving direction.
    # Detect flipped signals and enable for C-HR and others
    ret.enableBsm = 0x3F6 in fingerprint[0] and candidate in TSS2_CAR

    # No radar dbc for cars without DSU which are not TSS 2.0
    # TODO: make an adas dbc file for dsu-less models
    ret.radarUnavailable = Bus.radar not in DBC[candidate] or candidate in (NO_DSU_CAR - TSS2_CAR)

    # since we don't yet parse radar on TSS2/TSS-P radar-based ACC cars, gate longitudinal behind experimental toggle
    if candidate in (RADAR_ACC_CAR | NO_DSU_CAR):
      ret.alphaLongitudinalAvailable = candidate in RADAR_ACC_CAR

      # Disabling radar is only supported on TSS2 radar-ACC cars
      if alpha_long and candidate in RADAR_ACC_CAR:
        ret.flags |= ToyotaFlags.DISABLE_RADAR.value

      # RADAR_ACC_CAR = CHR TSS2 / RAV4 TSS2
      # NO_DSU_CAR = CAMRY / CHR
      if 0x2FF in fingerprint[0] or 0x2AA in fingerprint[0]:
        print("----------------------------------------------")
        print("dragonpilot: RADAR_FILTER detected!")
        print("----------------------------------------------")
        ret.alphaLongitudinalAvailable = False
        ret.flags |= ToyotaFlags.RADAR_FILTER.value | ToyotaFlags.DISABLE_RADAR.value

    sdsu_active = False
    if not (candidate in (RADAR_ACC_CAR | NO_DSU_CAR)) and 0x2FF in fingerprint[0]:
      print("----------------------------------------------")
      print("dragonpilot: SDSU detected!")
      print("----------------------------------------------")

      sdsu_active = True
      stop_and_go = True

      ret.flags |= ToyotaFlags.SDSU.value
      ret.alphaLongitudinalAvailable = False

    # openpilot longitudinal enabled by default:
    #  - cars w/ DSU disconnected
    #  - TSS2 cars with camera sending ACC_CONTROL where we can block it
    # openpilot longitudinal behind experimental long toggle:
    #  - TSS2 radar ACC cars (disables radar)

    ret.openpilotLongitudinalControl = (candidate in (TSS2_CAR - RADAR_ACC_CAR) or
                                        bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value) or \
        dsu_bypass) or \
      sdsu_active

    if dp_params & structs.DPFlags.ToyotaStockLon:
      ret.openpilotLongitudinalControl = False
      ret.alphaLongitudinalAvailable = False

    ret.autoResumeSng = ret.openpilotLongitudinalControl and candidate in NO_STOP_TIMER_CAR

    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if stop_and_go else MIN_ACC_SPEED

    if candidate in TSS2_CAR:
      ret.flags |= ToyotaFlags.RAISED_ACCEL_LIMIT.value

      # 🌟 優化 1：縮小靜止放生區間，讓 MPC 精準控制到最後一刻
      # 從 0.25 降到 0.1，配合 carcontroller 的 permit_braking 閾值，實現絲滑煞停與起步
      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      # 放緩停止前的減速率，減少點頭感
      ret.stoppingDecelRate = 0.15 

      # Hybrids have much quicker longitudinal actuator response
      if ret.flags & ToyotaFlags.HYBRID.value:
        ret.longitudinalActuatorDelay = 0.05
      else:
        # 🌟 優化 2：修正非油電車的預測延遲！
        # 配合我們在 carcontroller.py 做的 PID 強化，將預測延遲從 0.5 降至 0.15
        # 避免大腦「預判過度」導致動態過於激進
        ret.longitudinalActuatorDelay = 0.15

    if dp_params & structs.DPFlags.ToyotaLockCtrl:
      ret.flags |= ToyotaFlags.LOCK_CTRL.value
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.LOCK_CTRL.value

    if dp_params & structs.DPFlags.ToyotaTSS1SnG:
      ret.flags |= ToyotaFlags.TSS1_SNG.value

    return ret

  @staticmethod
  def init(CP, can_recv, can_send, communication_control=None):
    # disable radar if alpha longitudinal toggled on radar-ACC car
    if not CP.flags & ToyotaFlags.RADAR_FILTER.value and CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if communication_control is None:
        communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
      disable_ecu(can_recv, can_send, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    # re-enable radar if alpha longitudinal toggled on radar-ACC car
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX, uds.MESSAGE_TYPE.NORMAL])
    CarInterface.init(CP, can_recv, can_send, communication_control)

