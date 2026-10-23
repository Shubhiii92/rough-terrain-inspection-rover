"""
Rough-Terrain Inspection Rover Controller
Webots R2025a Python Controller

Implements:
- 4-wheel differential-drive velocity control
- Calibrated Webots DistanceSensor piecewise conversion
- Robust Finite-State Machine (FORWARD, STOP, TURN_LEFT, TURN_RIGHT, RECOVER, REVERSE)
- Obstacle avoidance and clearance estimation
- Fail-safe sensor verification
"""

import math
import sys

# Attempt importing Webots controller API
try:
    from controller import Robot, DistanceSensor, Motor
except ImportError:
    print("[ERROR] Webots 'controller' module could not be imported.")
    print("[INFO] Run this controller within Webots or configure PYTHONPATH to Webots controller lib.")
    sys.exit(1)


# ==============================================================================
# SENSOR CALIBRATION CONFIGURATION
# ==============================================================================
# Configured lookupTable defined in RoughTerrainRover.proto:
#   [ distance (m), signal, noise ]
#   0.0   -> 1000.0
#   0.3   ->  850.0
#   1.0   ->  500.0
#   2.0   ->    0.0
#
# Signal is monotonically decreasing with distance.
SENSOR_LOOKUP_TABLE = [
    (0.0, 1000.0),
    (0.3, 850.0),
    (1.0, 500.0),
    (2.0, 0.0)
]

MAX_SENSOR_RANGE_METERS = 2.0
OBSTACLE_THRESHOLD_METERS = 0.30  # Stop and avoid if obstacle closer than 30 cm
CRITICAL_DISTANCE_METERS = 0.14   # Emergency reverse if obstacle closer than 14 cm


def raw_signal_to_distance(raw_signal: float) -> float:
    """
    Converts raw Webots DistanceSensor signal into calibrated distance in meters.
    Performs piecewise-linear interpolation inverted from the sensor lookup table.
    
    Returns:
        float: Estimated distance in meters (0.0 to 2.0 m).
               Returns -1.0 if the reading is invalid (fail-safe trigger).
    """
    if raw_signal is None or math.isnan(raw_signal) or math.isinf(raw_signal):
        return -1.0  # Invalid sensor reading flag

    # Clamping bounds based on lookup table extremes
    if raw_signal >= 1000.0:
        return 0.0
    if raw_signal <= 0.0:
        return MAX_SENSOR_RANGE_METERS

    # Interpolate along segments
    for i in range(len(SENSOR_LOOKUP_TABLE) - 1):
        dist_start, sig_start = SENSOR_LOOKUP_TABLE[i]
        dist_end, sig_end = SENSOR_LOOKUP_TABLE[i + 1]

        # Verify if raw_signal lies within this segment [sig_start, sig_end]
        if (sig_start >= raw_signal >= sig_end) or (sig_end >= raw_signal >= sig_start):
            segment_span = sig_end - sig_start
            if segment_span == 0:
                return dist_start
            fraction = (raw_signal - sig_start) / segment_span
            return dist_start + fraction * (dist_end - dist_start)

    return MAX_SENSOR_RANGE_METERS


# ==============================================================================
# FINITE-STATE MACHINE DEFINITION
# ==============================================================================
class RoverState:
    FORWARD = "FORWARD"
    STOP = "STOP"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    RECOVER = "RECOVER"
    REVERSE = "REVERSE"


class RoughTerrainRoverController:
    """
    Autonomous differential-drive controller for rough-terrain inspection.
    """
    def __init__(self):
        # 1. Initialize Webots Robot instance
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        if self.time_step <= 0:
            self.time_step = 32

        # 2. Motor device names as strictly specified
        self.motor_names = [
            "left_front_motor",
            "left_rear_motor",
            "right_front_motor",
            "right_rear_motor"
        ]
        self.motors = {}
        for name in self.motor_names:
            motor = self.robot.getDevice(name)
            if motor is None:
                print(f"[ERROR] Motor device '{name}' not found on robot!")
            else:
                # Configure for continuous velocity control
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                self.motors[name] = motor

        # 3. Distance sensor devices
        self.sensor_left = self.robot.getDevice("front_left_distance")
        self.sensor_right = self.robot.getDevice("front_right_distance")

        if self.sensor_left:
            self.sensor_left.enable(self.time_step)
        else:
            print("[ERROR] Sensor 'front_left_distance' not found!")

        if self.sensor_right:
            self.sensor_right.enable(self.time_step)
        else:
            print("[ERROR] Sensor 'front_right_distance' not found!")

        # 4. Motion velocity parameters (rad/s)
        self.CRUISE_SPEED = 4.0     # Forward motion speed
        self.TURN_SPEED = 2.5       # In-place turn speed
        self.REVERSE_SPEED = 2.0    # Backing up speed

        # 5. State Machine and Timing Variables
        self.state = RoverState.FORWARD
        self.state_step_counter = 0
        self.turn_duration_steps = 30
        self.recover_duration_steps = 25
        self.consecutive_turns = 0
        self.step_index = 0

        print("=" * 68)
        print(" [ROVER INITIALIZED] Rough-Terrain Inspection Rover")
        print(f" Time step: {self.time_step} ms")
        print(f" Motors detected: {len(self.motors)}/4")
        print(f" Distance Sensors: Left={bool(self.sensor_left)}, Right={bool(self.sensor_right)}")
        print(f" Obstacle Threshold: {OBSTACLE_THRESHOLD_METERS:.2f} m")
        print(f" Critical Clearance: {CRITICAL_DISTANCE_METERS:.2f} m")
        print(" Initial State: FORWARD")
        print("=" * 68)

    def set_drive_velocities(self, left_speed: float, right_speed: float):
        """
        Applies differential speeds to left and right motor pairs.
        Both left wheels receive left_speed; both right wheels receive right_speed.
        """
        if "left_front_motor" in self.motors:
            self.motors["left_front_motor"].setVelocity(left_speed)
        if "left_rear_motor" in self.motors:
            self.motors["left_rear_motor"].setVelocity(left_speed)

        if "right_front_motor" in self.motors:
            self.motors["right_front_motor"].setVelocity(right_speed)
        if "right_rear_motor" in self.motors:
            self.motors["right_rear_motor"].setVelocity(right_speed)

    def transition_to(self, new_state: str, reason: str = ""):
        """
        Manages state machine transitions with informative logging.
        """
        old_state = self.state
        self.state = new_state
        self.state_step_counter = 0

        sim_time = self.robot.getTime()
        reason_str = f" | Reason: {reason}" if reason else ""
        print(f"[{sim_time:6.2f}s] STATE TRANSITION: {old_state} -> {new_state}{reason_str}")

    def read_calibrated_distances(self):
        """
        Reads sensors and applies lookup-table calibration.
        Fails safely if sensor devices are missing or return NaN.
        """
        if not self.sensor_left or not self.sensor_right:
            return -1.0, -1.0

        raw_left = self.sensor_left.getValue()
        raw_right = self.sensor_right.getValue()

        dist_left = raw_signal_to_distance(raw_left)
        dist_right = raw_signal_to_distance(raw_right)

        return dist_left, dist_right

    def run(self):
        """
        Main controller execution loop.
        """
        while self.robot.step(self.time_step) != -1:
            self.step_index += 1
            self.state_step_counter += 1

            # 1. Read calibrated sensor estimates
            dist_left, dist_right = self.read_calibrated_distances()

            # Fail-safe check: Stop if sensor readings are corrupted
            if dist_left < 0.0 or dist_right < 0.0:
                if self.state != RoverState.STOP:
                    self.set_drive_velocities(0.0, 0.0)
                    self.transition_to(RoverState.STOP, "Sensor failure / invalid reading (Fail-Safe Triggered)")
                continue

            # Periodic status telemetry (every 50 steps ~ 0.8-1.6s)
            if self.step_index % 50 == 0:
                print(f"[TELEMETRY {self.robot.getTime():5.2f}s] State: {self.state:<10} | "
                      f"Distances: L={dist_left:.2f}m, R={dist_right:.2f}m")

            # 2. Finite-State Machine execution
            if self.state == RoverState.FORWARD:
                # Drive forward at controlled cruise speed
                self.set_drive_velocities(self.CRUISE_SPEED, self.CRUISE_SPEED)

                # Emergency close proximity check
                if dist_left < CRITICAL_DISTANCE_METERS or dist_right < CRITICAL_DISTANCE_METERS:
                    self.set_drive_velocities(0.0, 0.0)
                    self.transition_to(
                        RoverState.RECOVER,
                        f"Critical obstacle proximity (L={dist_left:.2f}m, R={dist_right:.2f}m)"
                    )
                # Standard obstacle detection check
                elif dist_left < OBSTACLE_THRESHOLD_METERS or dist_right < OBSTACLE_THRESHOLD_METERS:
                    self.set_drive_velocities(0.0, 0.0)
                    self.transition_to(
                        RoverState.STOP,
                        f"Obstacle in range (L={dist_left:.2f}m, R={dist_right:.2f}m <= {OBSTACLE_THRESHOLD_METERS:.2f}m)"
                    )

            elif self.state == RoverState.STOP:
                # Halt rover and evaluate clearance
                self.set_drive_velocities(0.0, 0.0)

                # Wait 2 steps for physics stabilization before choosing turn direction
                if self.state_step_counter >= 2:
                    # Check for dead end or multiple turns without progress
                    if self.consecutive_turns >= 4:
                        self.consecutive_turns = 0
                        self.transition_to(RoverState.RECOVER, "Multiple turns failed to clear obstacles; initiating recovery")
                    elif dist_left < CRITICAL_DISTANCE_METERS and dist_right < CRITICAL_DISTANCE_METERS:
                        self.transition_to(RoverState.RECOVER, "Both sensors blocked at close range")
                    elif dist_left >= dist_right:
                        # Left side has greater or equal clearance
                        self.consecutive_turns += 1
                        self.transition_to(
                            RoverState.TURN_LEFT,
                            f"Left clearance ({dist_left:.2f}m) >= Right clearance ({dist_right:.2f}m)"
                        )
                    else:
                        # Right side has greater clearance
                        self.consecutive_turns += 1
                        self.transition_to(
                            RoverState.TURN_RIGHT,
                            f"Right clearance ({dist_right:.2f}m) > Left clearance ({dist_left:.2f}m)"
                        )

            elif self.state == RoverState.TURN_LEFT:
                # In-place differential turn to the left
                # Left wheels rotate backward, right wheels rotate forward
                self.set_drive_velocities(-self.TURN_SPEED, self.TURN_SPEED)

                # Enforce minimum turn duration to prevent twitching/rapid switching
                if self.state_step_counter >= self.turn_duration_steps:
                    # Check if forward path is clear
                    if dist_left > OBSTACLE_THRESHOLD_METERS and dist_right > OBSTACLE_THRESHOLD_METERS:
                        self.consecutive_turns = 0
                        self.transition_to(
                            RoverState.FORWARD,
                            f"Clearance established (L={dist_left:.2f}m, R={dist_right:.2f}m)"
                        )
                    elif self.state_step_counter >= self.turn_duration_steps * 2:
                        # Still blocked after full turn duration, halt and re-evaluate
                        self.transition_to(RoverState.STOP, "Turn limit reached, re-evaluating clearance")

            elif self.state == RoverState.TURN_RIGHT:
                # In-place differential turn to the right
                # Left wheels rotate forward, right wheels rotate backward
                self.set_drive_velocities(self.TURN_SPEED, -self.TURN_SPEED)

                if self.state_step_counter >= self.turn_duration_steps:
                    if dist_left > OBSTACLE_THRESHOLD_METERS and dist_right > OBSTACLE_THRESHOLD_METERS:
                        self.consecutive_turns = 0
                        self.transition_to(
                            RoverState.FORWARD,
                            f"Clearance established (L={dist_left:.2f}m, R={dist_right:.2f}m)"
                        )
                    elif self.state_step_counter >= self.turn_duration_steps * 2:
                        self.transition_to(RoverState.STOP, "Turn limit reached, re-evaluating clearance")

            elif self.state == RoverState.RECOVER or self.state == RoverState.REVERSE:
                # Back away from obstacle to gain turning clearance
                self.set_drive_velocities(-self.REVERSE_SPEED, -self.REVERSE_SPEED)

                if self.state_step_counter >= self.recover_duration_steps:
                    self.set_drive_velocities(0.0, 0.0)
                    self.transition_to(RoverState.STOP, "Recovery reverse completed; re-evaluating clearance")


# ==============================================================================
# ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    controller = RoughTerrainRoverController()
    controller.run()
