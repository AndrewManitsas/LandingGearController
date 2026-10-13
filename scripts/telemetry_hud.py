#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float64, Int32, String, Float32MultiArray
from geometry_msgs.msg import Vector3

class TelemetryHUD(Node):
    def __init__(self):
        super().__init__('telemetry_hud')

        self.altitude = 0.0
        self.progress_x = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.elevator_deg = 0.0
        self.gear_angle_rad = 0.0
        self.verdict = 0
        self.abort = False
        self.fsm_state = "INITIALIZING"
        self.features = [0.0] * 7

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.create_subscription(Float64, '/sensing/altitude_agl', self.alt_cb, sensor_qos)
        self.create_subscription(Float64, '/flight/progress_x', self.prog_cb, 10)
        self.create_subscription(Vector3, '/control/attitude', self.att_cb, 10)
        self.create_subscription(Float64, '/control/elevator_cmd', self.elev_cb, 10)
        self.create_subscription(Float64, '/control/gear_status', self.gear_cb, 10)
        self.create_subscription(Int32, '/intelligence/verdict', self.verdict_cb, 10)
        self.create_subscription(Bool, '/intelligence/abort', self.abort_cb, 10)
        self.create_subscription(String, '/flight/supervisor_state', self.state_cb, 10)
        self.create_subscription(Float32MultiArray, '/sensing/features', self.feat_cb, sensor_qos)

        self.timer = self.create_timer(0.1, self.render_hud)

    def alt_cb(self, msg: Float64):
        self.altitude = msg.data

    def prog_cb(self, msg: Float64):
        self.progress_x = msg.data

    def att_cb(self, msg: Vector3):
        self.roll_deg = math.degrees(msg.x)
        self.pitch_deg = math.degrees(msg.y)

    def elev_cb(self, msg: Float64):
        self.elevator_deg = math.degrees(msg.data)

    def gear_cb(self, msg: Float64):
        self.gear_angle_rad = msg.data

    def verdict_cb(self, msg: Int32):
        self.verdict = msg.data

    def abort_cb(self, msg: Bool):
        self.abort = msg.data

    def state_cb(self, msg: String):
        self.fsm_state = msg.data

    def feat_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 7:
            self.features = list(msg.data)

    def render_hud(self):
        C_RESET  = "\033[0m"
        C_BOLD   = "\033[1m"
        C_GREEN  = "\033[92m"
        C_RED    = "\033[91m"
        C_YELLOW = "\033[93m"
        C_CYAN   = "\033[96m"
        C_WHITE  = "\033[97m"

        if self.verdict == 0:
            verdict_str = f"{C_GREEN}{C_BOLD}[ SAFE RUNWAY - CLEAR TO LAND ]{C_RESET}"
        elif self.verdict == 1:
            verdict_str = f"{C_RED}{C_BOLD}[ UNSAFE: STEEP SLOPE (>15 deg) ]{C_RESET}"
        elif self.verdict == 2:
            verdict_str = f"{C_YELLOW}{C_BOLD}[ UNSAFE: VEGETATION CANOPY ]{C_RESET}"
        else:
            verdict_str = f"{C_WHITE}[ SCANNING ]{C_RESET}"

        abort_str = f"{C_RED}{C_BOLD}TRUE (INHIBIT ACTIVE){C_RESET}" if self.abort else f"{C_GREEN}FALSE (NOMINAL){C_RESET}"

        gear_pct = min(100, max(0, int((self.gear_angle_rad / 1.5708) * 100)))
        bar_len = 20
        filled = int((gear_pct / 100) * bar_len)
        gear_bar = f"[{'#' * filled}{'.' * (bar_len - filled)}] {gear_pct:3d}% ({self.gear_angle_rad:.2f} rad)"

        slope_deg = math.degrees(self.features[4])
        delta_d = self.features[2]

        sys.stdout.write("\033[H\033[J")
        sys.stdout.write(f"{C_BOLD}{C_CYAN}======================================================================{C_RESET}\n")
        sys.stdout.write(f"{C_BOLD}{C_CYAN}          UAV AUTONOMOUS APPROACH & TOUCHDOWN DECISION HUD             {C_RESET}\n")
        sys.stdout.write(f"{C_BOLD}{C_CYAN}======================================================================{C_RESET}\n")
        sys.stdout.write(f" Corridor Pos (X): {C_BOLD}{self.progress_x:6.1f} m / 100 m{C_RESET} | Airspeed:  3.00 m/s\n")
        sys.stdout.write(f" Flight State:     {C_BOLD}{self.fsm_state:<18}{C_RESET} | Altitude:  {C_BOLD}{self.altitude:6.2f} m{C_RESET}\n")
        sys.stdout.write(f" Stabilator Trim:  {self.elevator_deg:6.2f} deg          | Pitch:     {self.pitch_deg:6.2f} deg\n")
        sys.stdout.write(f"----------------------------------------------------------------------\n")
        sys.stdout.write(f" TERRAIN VERDICT:  {verdict_str}\n")
        sys.stdout.write(f" AI Abort Trigger: {abort_str}\n")
        sys.stdout.write(f" Terrain Slope:    {slope_deg:6.2f} deg          | Delta d:   {delta_d:+6.3f} m\n")
        sys.stdout.write(f"----------------------------------------------------------------------\n")
        sys.stdout.write(f" LANDING GEAR:     {gear_bar}\n")
        if self.abort:
            sys.stdout.write(f" STATUS:           {C_RED}{C_BOLD}EMERGENCY LOCKOUT: Gear stowed to prevent airframe rollover.{C_RESET}\n")
        elif self.fsm_state == "TOUCHDOWN":
            sys.stdout.write(f" STATUS:           {C_GREEN}{C_BOLD}TOUCHDOWN COMPLETE: Forward velocity halted on runway.{C_RESET}\n")
        elif self.fsm_state == "CORRIDOR_TRANSIT":
            sys.stdout.write(f" STATUS:           Approach corridor transit. Landing gear held stowed.{C_RESET}\n")
        elif self.gear_angle_rad > 1.2:
            sys.stdout.write(f" STATUS:           {C_GREEN}Gear down and locked. Approaching touchdown flare.{C_RESET}\n")
        else:
            sys.stdout.write(f" STATUS:           Deploying landing gear actuators for touchdown.{C_RESET}\n")
        sys.stdout.write(f"{C_BOLD}{C_CYAN}======================================================================{C_RESET}\n")
        sys.stdout.flush()

def main():
    rclpy.init()
    node = TelemetryHUD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()