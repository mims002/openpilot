from cereal import car
from openpilot.common.numpy_fast import clip
import time
from openpilot.common.conversions import Conversions as CV
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_std_steer_angle_limits, AngleRateLimit
from openpilot.selfdrive.car.ford import fordcan
from openpilot.selfdrive.car.ford.values import (
    CANFD_CAR,
    CarControllerParams,
    CarControllerParamsBronco,
)

LongCtrlState = car.CarControl.Actuators.LongControlState
VisualAlert = car.CarControl.HUDControl.VisualAlert


def apply_ford_curvature_limits(
    apply_curvature, apply_curvature_last, current_curvature, v_ego_raw
):
    # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
    if v_ego_raw > 9:
        apply_curvature = clip(
            apply_curvature,
            current_curvature - CarControllerParams.CURVATURE_ERROR,
            current_curvature + CarControllerParams.CURVATURE_ERROR,
        )

    # Curvature rate limit after driver torque limit
    apply_curvature = apply_std_steer_angle_limits(
        apply_curvature, apply_curvature_last, v_ego_raw, CarControllerParams
    )

    return clip(
        apply_curvature,
        -CarControllerParams.CURVATURE_MAX,
        CarControllerParams.CURVATURE_MAX,
    )


def apply_ford_angle(desired_angle, CS):
    # desired_angle = apply_std_steer_angle_limits(
    #     desired_angle, CS.out.steeringAngleDeg, CS.out.vEgoRaw, CarControllerParamsBronco
    # )
    
    normalized_angle = desired_angle - CS.out.steeringAngleDeg
    clipped_angle = clip(normalized_angle, -5.8, 5.8)
    
    return clipped_angle


class CarController:
    def __init__(self, dbc_name, CP, VM):
        self.CP = CP
        self.VM = VM
        self.packer = CANPacker(dbc_name)
        self.CAN = fordcan.CanBus(CP)
        self.frame = 0

        self.apply_curvature_last = 0
        self.apply_angle_last = 0
        self.main_on_last = False
        self.lkas_enabled_last = False
        self.steer_alert_last = False
        self.last_timeout_at = time.time()
        self.last_timeout_duration = 0
        self.on_count = 0
        

    def update(self, CC, CS, now_nanos):
        can_sends = []

        actuators = CC.actuators
        hud_control = CC.hudControl

        main_on = CS.out.cruiseState.available
        steer_alert = hud_control.visualAlert in (
            VisualAlert.steerRequired,
            VisualAlert.ldw,
        )
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw

        ### acc buttons ###
        if CC.cruiseControl.cancel:
            can_sends.append(
                fordcan.create_button_msg(
                    self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True
                )
            )
            can_sends.append(
                fordcan.create_button_msg(
                    self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True
                )
            )
        elif (
            CC.cruiseControl.resume
            and (self.frame % CarControllerParams.BUTTONS_STEP) == 0
        ):
            can_sends.append(
                fordcan.create_button_msg(
                    self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True
                )
            )
            can_sends.append(
                fordcan.create_button_msg(
                    self.packer, self.CAN.main, CS.buttons_stock_values, resume=True
                )
            )
        # if stock lane centering isn't off, send a button press to toggle it off
        # the stock system checks for steering pressed, and eventually disengages cruise control
        elif (
            CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0
            and (self.frame % CarControllerParams.ACC_UI_STEP) == 0
        ):
            can_sends.append(
                fordcan.create_button_msg(
                    self.packer,
                    self.CAN.camera,
                    CS.buttons_stock_values,
                    tja_toggle=True,
                )
            )

        ### lateral control ###
        if CC.latActive and CS.lkas_available:
            # apply rate limits, curvature error limit, and clip to signal range
            current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
            apply_curvature = apply_ford_curvature_limits(
                actuators.curvature,
                self.apply_curvature_last,
                current_curvature,
                CS.out.vEgoRaw,
            )
            apply_angle = apply_ford_angle(
                actuators.steeringAngleDeg, CS
            )

            self.apply_curvature_last = apply_curvature
        else:
            apply_curvature = 0.0
            apply_angle = 0

        # send steer msg at 20Hz
        if (self.frame % CarControllerParams.STEER_STEP) == 0:
            if True or self.CP.carFingerprint in CANFD_CAR:
                # TODO: extended mode
                mode = 1 if CC.latActive else 0
                counter = (self.frame // CarControllerParams.STEER_STEP) % 0xF
                can_sends.append(
                    fordcan.create_lat_ctl2_msg(
                        self.packer,
                        self.CAN,
                        mode,
                        0.0,
                        0.0,
                        -apply_curvature,
                        0.0,
                        counter,
                    )
                )
            else:
                can_sends.append(
                    fordcan.create_lat_ctl_msg(
                        self.packer,
                        self.CAN,
                        CC.latActive,
                        CS.lateral_motion_control,
                    )
                )

        # send lka msg at 33Hz
        if (self.frame % CarControllerParams.LKA_STEP) == 0:
            if not CS.lkas_available:
                self.last_timeout_duration = time.time() - self.last_timeout_at
                self.last_timeout_at = time.time()
                
            if time.time() - self.last_timeout_at > self.last_timeout_duration and CS.lkas_available:
                self.last_timeout_duration = time.time() - self.last_timeout_at
                near_timeout = False
            elif time.time() - self.last_timeout_at >= self.last_timeout_duration - 500:
                near_timeout = True
            else:
                near_timeout = False
            
            if CC.latActive and CS.lkas_available and self.on_count < 200:
                new_direction = 2 if CS.out.steeringAngleDeg > 0 else 4
                self.on_count += 1
            else:
                new_direction = 0
                self.on_count = 0
            
            if  abs(apply_angle) >= 5 :
                ramp_type = 1
            else:
                ramp_type = 0

            message = fordcan.create_lka_msg(
                self.packer,
                self.CAN,
                CC.latActive and CS.lkas_available,
                apply_angle,
                -apply_curvature,
                new_direction,
                ramp_type
            )

            can_sends.append(message)

        ### longitudinal control ###
        # send acc msg at 50Hz
        if (
            self.CP.openpilotLongitudinalControl
            and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0
        ):
            # Both gas and accel are in m/s^2, accel is used solely for braking
            accel = clip(
                actuators.accel,
                CarControllerParams.ACCEL_MIN,
                CarControllerParams.ACCEL_MAX,
            )
            gas = accel
            if not CC.longActive or gas < CarControllerParams.MIN_GAS:
                gas = CarControllerParams.INACTIVE_GAS
            stopping = CC.actuators.longControlState == LongCtrlState.stopping
            can_sends.append(
                fordcan.create_acc_msg(
                    self.packer,
                    self.CAN,
                    CC.longActive,
                    gas,
                    accel,
                    stopping,
                    v_ego_kph=40 * CV.MPH_TO_KPH,
                )
            )

        ### ui ###
        send_ui = (
            (self.main_on_last != main_on)
            or (self.lkas_enabled_last != CC.latActive)
            or (self.steer_alert_last != steer_alert)
        )
        # send lkas ui msg at 1Hz or if ui state changes
        if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
            can_sends.append(
                fordcan.create_lkas_ui_msg(
                    self.packer,
                    self.CAN,
                    main_on,
                    CC.latActive,
                    steer_alert,
                    hud_control,
                    CS.lkas_status_stock_values,
                )
            )
        # send acc ui msg at 5Hz or if ui state changes
        if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
            can_sends.append(
                fordcan.create_acc_ui_msg(
                    self.packer,
                    self.CAN,
                    self.CP,
                    main_on,
                    CC.latActive,
                    fcw_alert,
                    CS.out.cruiseState.standstill,
                    hud_control,
                    CS.acc_tja_status_stock_values,
                )
            )

        self.main_on_last = main_on
        self.lkas_enabled_last = CC.latActive
        self.steer_alert_last = steer_alert
        self.apply_angle_last = apply_angle
        
        new_actuators = actuators.copy()
        new_actuators.curvature = self.apply_curvature_last
        new_actuators.steeringAngleDeg = apply_angle + CS.out.steeringAngleDeg
        self.frame += 1
        return new_actuators, can_sends
