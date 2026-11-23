"""Simulation-specific cascaded altitude and vertical-speed controller."""

SPEED_FILTER_ALPHA = 0.4
ALTITUDE_KP = 0.6
MAX_VERTICAL_SPEED_M_S = 1.5
TAKEOFF_MAX_CLIMB_SPEED_M_S = 4.0
LANDING_DESCENT_SPEED_M_S = 0.5
VELOCITY_KP = 100.0
VELOCITY_KI = 25.0
INTEGRAL_LIMIT = 8.0
MAX_SAMPLE_GAP_S = 1.0
HOVER_THROTTLE = 1400
CONTROL_THROTTLE_MIN = 1050
CONTROL_THROTTLE_MAX = 1650
LOW_HEIGHT_MIN_CLIMB_SPEED_M_S = 0.5


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class AltitudeControl:
    """Update throttle only on new altitude samples."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.previous: tuple[float, float] | None = None
        self.speed = 0.0
        self.speed_ready = False
        self.integral = 0.0
        self.throttle = HOVER_THROTTLE

    def update(self, target_m: float, altitude_m: float, timestamp_s: float, *, landing: bool,
               takeoff: bool = False, low_height_takeoff: bool = False) -> int:
        if self.previous is None:
            self.previous = (altitude_m, timestamp_s)
            return self.throttle
        previous_altitude, previous_time = self.previous
        dt = timestamp_s - previous_time
        if dt <= 0:
            return self.throttle
        self.previous = (altitude_m, timestamp_s)
        if dt > MAX_SAMPLE_GAP_S:
            self.speed = 0.0
            self.speed_ready = False
            self.integral = 0.0
            return self.throttle
        measured_speed = (altitude_m - previous_altitude) / dt
        self.speed += SPEED_FILTER_ALPHA * (measured_speed - self.speed)
        self.speed_ready = True
        descent_limit = LANDING_DESCENT_SPEED_M_S if landing else MAX_VERTICAL_SPEED_M_S
        climb_limit = TAKEOFF_MAX_CLIMB_SPEED_M_S if takeoff else MAX_VERTICAL_SPEED_M_S
        target_speed = _clamp(ALTITUDE_KP * (target_m - altitude_m), -descent_limit,
                              climb_limit)
        if low_height_takeoff and target_m - altitude_m > 0.03:
            target_speed = max(target_speed, LOW_HEIGHT_MIN_CLIMB_SPEED_M_S)
        error = target_speed - self.speed
        self.integral = _clamp(self.integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        self.throttle = round(_clamp(HOVER_THROTTLE + VELOCITY_KP * error
                                     + VELOCITY_KI * self.integral,
                                     CONTROL_THROTTLE_MIN, CONTROL_THROTTLE_MAX))
        return self.throttle
