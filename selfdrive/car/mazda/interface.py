#!/usr/bin/env python3
from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car.mazda.values import CAR, LKAS_LIMITS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, \
    get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):

    @staticmethod
    def compute_gb(accel, speed):
        return float(accel) / 4.0

    @staticmethod
    def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, disable_radar=False):
        ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

        ret.carName = "mazda"
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.mazda)]
        ret.radarOffCan = True

        ret.dashcamOnly = False  # 強制關閉行車記錄器模式

        ret.steerActuatorDelay = 0.1 + 0.1  # 預設0.1 轉向延遲補償
        ret.steerLimitTimer = 0.8 + 1  # 預設0.8 警告延遲時間
        tire_stiffness_factor = 0.70  # not optimized yet 避震？輪胎？

        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

        if candidate in (CAR.CX5, CAR.CX5_2022):
            ret.mass = 3655 * CV.LB_TO_KG + STD_CARGO_KG  # 車重 預設3655, 18 CX-5 2.0 = 3450
            ret.wheelbase = 2.7  # 軸距
            ret.steerRatio = 15.5  # 預設15.5 轉向角度
        elif candidate == CAR.CX5_PID:  # 測試用
            ret.lateralTuning.init('pid')
            ret.steerRateCost = 1.0

            ret.mass = 3450 * CV.LB_TO_KG + STD_CARGO_KG  # 車重 預設3655, 18 CX-5 2.0 = 3450
            ret.wheelbase = 2.7  # 軸距
            ret.steerRatio = 15.5  # 預設15.5 轉向角度
            # 增加pid參數 2022/08/23(測試Failed)
            ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
            ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.19], [0.019]]
            ret.lateralTuning.pid.kf = 0.00006

        elif candidate in (CAR.CX9, CAR.CX9_2021):
            ret.mass = 4217 * CV.LB_TO_KG + STD_CARGO_KG
            ret.wheelbase = 3.1
            ret.steerRatio = 17.6
        elif candidate == CAR.MAZDA3:
            ret.mass = 2875 * CV.LB_TO_KG + STD_CARGO_KG
            ret.wheelbase = 2.7
            ret.steerRatio = 14.0
        elif candidate == CAR.MAZDA6:
            ret.mass = 3443 * CV.LB_TO_KG + STD_CARGO_KG
            ret.wheelbase = 2.83
            ret.steerRatio = 15.5

        if candidate not in (CAR.CX5_2022,):
            ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

        ret.centerToFront = ret.wheelbase * 0.41  # 車輛重心配比 車輛重心到前輪軸的距離，用來計算轉彎慣性

        # TODO: get actual value, for now starting with reasonable value for
        # civic and scaling by mass and wheelbase
        ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

        # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
        # mass and CG position, so all cars will have approximately similar dyn behaviors
        ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                             tire_stiffness_factor=tire_stiffness_factor)

        CarInterfaceBase.configure_lqr_tune(ret.lateralTuning)
        return ret

    # returns a car.CarState
    def _update(self, c):
        ret = self.CS.update(self.cp, self.cp_cam)
        ret.cruiseState.enabled, ret.cruiseState.available = self.dp_atl_mode(ret)

        # events
        events = self.create_common_events(ret)
        events = self.dp_atl_warning(ret, events)

        if self.CS.lkas_disabled:
            events.add(EventName.lkasDisabled)
        elif self.dragonconf.dpMazdaSteerAlert and self.CS.low_speed_alert:
            events.add(EventName.belowSteerSpeed)

        ret.events = events.to_msg()

        return ret

    def apply(self, c):
        return self.CC.update(c, self.CS)
