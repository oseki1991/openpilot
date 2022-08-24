from cereal import car
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.mazda import mazdacan
from selfdrive.car.mazda.values import CarControllerParams, Buttons

VisualAlert = car.CarControl.HUDControl.VisualAlert

latState_tmp = 0


class CarController:

    def __init__(self, dbc_name, CP, VM):
        self.CP = CP
        self.apply_steer_last = 0
        self.packer = CANPacker(dbc_name)
        self.brake_counter = 0
        self.frame = 0

    def update(self, CC, CS):
        can_sends = []
        global latState_tmp
        apply_steer = 0

        if CC.latActive:
            # calculate steer and also set limits due to driver torque
            new_steer = int(round(CC.actuators.steer * CarControllerParams.STEER_MAX))
            apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last,
                                                        CS.out.steeringTorque, CarControllerParams)

        if CC.cruiseControl.cancel:
            # 全速域跟車自動起步？
            # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
            # a race condition with the stock system, where the second cancel from openpilot
            # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
            # read 3 messages and most likely sync state before we attempt cancel.
            self.brake_counter = self.brake_counter + 1
            if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
                # Cancel Stock ACC if it's enabled while OP is disengaged
                # Send at a rate of 10hz until we sync with stock ACC state
                can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter,
                                                            Buttons.CANCEL))
        else:
            self.brake_counter = 0
            if CC.cruiseControl.resume and self.frame % 5 == 0:
                # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
                # Send Resume button when planner wants car to move
                can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter,
                                                            Buttons.RESUME))

        self.apply_steer_last = apply_steer

        # send HUD alerts  原車HUD告警
        if self.frame % 50 == 0:
            # VisualAlert.steerRequired 方向盤接管告警
            # VisualAlert.ldw 未知 車道偏移？
            ldw = CC.hudControl.visualAlert == VisualAlert.ldw
            steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
            # TODO: find a way to silence audible warnings so we can add more hud alerts
            steer_required = steer_required and CS.lkas_allowed_speed
            # 關閉所有HUD告警  (測試OK)
            steer_required = False
            can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))
            """
            # TODO: 0284 方向燈關閉車道維持 (未測試)
            if CS.leftBlinker or CS.rightBlinker:  # 判斷方向燈狀態
                # 全時車道維持 打方向燈關閉維持
                if CC.latActive and latState_tmp == 0:
                    latState_tmp = 1  # 暫存方向燈開啟前的車道維持狀態
                    can_sends.append(
                        mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter,
                                                   Buttons.CANCEL))
             TODO: 0284 方向燈結束 恢復車道維持 (未測試)
            else:
                if latState_tmp == 1:
                    # 恢復打方向燈之前的狀態
                    latState_tmp = 0
                    can_sends.append(
                        mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter,
                                                   Buttons.RESUME))
            """

        # send steering command
        can_sends.append(mazdacan.create_steering_control(self.packer, self.CP.carFingerprint,
                                                          self.frame, apply_steer, CS.cam_lkas))

        new_actuators = CC.actuators.copy()
        new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX

        self.frame += 1
        return new_actuators, can_sends
