"""Debounced alerts for the rates actually received from SITL."""
from dataclasses import dataclass

RATE_LOW_HZ = 40.0
RATE_RECOVERED_HZ = 45.0
RATE_WINDOW_S = 1.0
RATE_STARTUP_S = 2.0
RATE_CONFIRM_WINDOWS = 2


@dataclass
class _StreamState:
    low_windows: int = 0
    recovered_windows: int = 0
    alerted: bool = False


class RateMonitor:
    def __init__(self, started_at: float):
        self.next_check_at = started_at + RATE_STARTUP_S
        self.streams = {'ATTITUDE': _StreamState(), 'VFR_HUD': _StreamState()}

    def check(self, now: float, attitude_hz: float, altitude_hz: float):
        if now < self.next_check_at:
            return ()
        self.next_check_at = now + RATE_WINDOW_S
        events = []
        for name, hz in (('ATTITUDE', attitude_hz), ('VFR_HUD', altitude_hz)):
            state = self.streams[name]
            if not state.alerted:
                state.low_windows = state.low_windows + 1 if hz < RATE_LOW_HZ else 0
                if state.low_windows >= RATE_CONFIRM_WINDOWS:
                    state.alerted = True
                    state.recovered_windows = 0
                    events.append((name, 'low', hz))
            else:
                state.recovered_windows = state.recovered_windows + 1 if hz >= RATE_RECOVERED_HZ else 0
                if state.recovered_windows >= RATE_CONFIRM_WINDOWS:
                    state.alerted = False
                    state.low_windows = 0
                    events.append((name, 'recovered', hz))
        return tuple(events)
