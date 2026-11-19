"""Bounded diagnostic checkpoints for target guidance, separate from telemetry recording."""
from loguru import logger

from .missions import Phase

CHECKPOINT_PERIOD_S = 0.2
TRACKING_PHASES = frozenset((Phase.SEARCH, Phase.CENTER, Phase.SETTLE, Phase.LOS, Phase.COMMIT))
LOS_PHASES = frozenset((Phase.LOS, Phase.COMMIT))


def _rounded(value):
    return None if value is None else round(value, 3)


class TrackingLog:
    def __init__(self):
        self.last_checkpoint_at = None
        self.last_visible = False
        self.last_stage = None
        self.last_contact = False
        self.last_result = None

    def observe(self, mission, sample, command, altitude_m, now):
        """Log transitions once and sample active guidance at no more than 5 Hz."""
        packet = mission.bbox
        box = mission._visual_box(now) if packet is not None else None
        visible = box is not None
        if visible != self.last_visible:
            logger.debug('target event={} at={:.3f} phase={} bbox={}',
                         'visible' if visible else 'lost', now, mission.phase.value, box)
            self.last_visible = visible
        if mission.contact and not self.last_contact:
            logger.info('target event=contact at={:.3f} phase={}', now, mission.phase.value)
            self.last_contact = True
        if mission.result != self.last_result:
            if mission.result is not None and mission.result != 'hit':
                logger.debug('target event=failure at={:.3f} phase={} reason={}',
                             now, mission.phase.value, mission.result)
            self.last_result = mission.result

        navigation = mission.last_navigation_result if mission.phase in LOS_PHASES else None
        if navigation is not None and navigation.stage != self.last_stage:
            logger.info('LOS stage={} at={:.3f} state={} reason={}',
                        navigation.stage.value, now, navigation.state.value, navigation.reason)
            self.last_stage = navigation.stage
        if mission.phase not in TRACKING_PHASES:
            return
        if self.last_checkpoint_at is not None and now - self.last_checkpoint_at < CHECKPOINT_PERIOD_S:
            return
        self.last_checkpoint_at = now
        diagnostics = None if navigation is None else navigation.diagnostics
        demand = None if navigation is None else navigation.command
        yaw = None if sample.attitude is None else sample.attitude.yaw
        datum = mission.ground_altitude_m
        relative_altitude = None if altitude_m is None or datum is None else altitude_m - datum
        image_age = None if packet is None else now - packet['capture_ns'] / 1e9
        bbox_error = None if box is None else box['x'] + box['width'] / 2 - mission.config.camera.cx_px
        logger.debug(
            'TRACK checkpoint at={:.3f} phase={} bbox={} image_age_s={} bbox_error_x_px={} '
            'altitude_m={} yaw_rad={} los_state={} los_stage={} los_reason={} vision={} '
            'processed_seq={} processed_bbox={} aligned_image_age_s={} '
            'bearing_elevation_rad={} bearing_azimuth_rad={} reference_elevation_rad={} reference_azimuth_rad={} '
            'ttc_s={} descent_feasible={} requested_tilt_rad={} measured_tilt_rad={} '
            'angle_limited={} vertical_limited={} thrust_saturated={} '
            'demand_roll_rad={} demand_pitch_rad={} demand_yaw_rate_rad_s={} demand_thrust_n={} '
            'rc_roll={} rc_pitch={} rc_yaw={} rc_throttle={} rc_arm={}',
            now, mission.phase.value, box, _rounded(image_age), _rounded(bbox_error),
            _rounded(relative_altitude), _rounded(yaw),
            None if navigation is None else navigation.state.value,
            None if navigation is None else navigation.stage.value,
            None if navigation is None else navigation.reason,
            None if diagnostics is None else diagnostics.vision_reason,
            None if diagnostics is None else diagnostics.processed_image_sequence,
            None if diagnostics is None else diagnostics.processed_bbox,
            None if diagnostics is None else _rounded(diagnostics.image_age_s),
            None if diagnostics is None else _rounded(diagnostics.elevation_rad),
            None if diagnostics is None else _rounded(diagnostics.azimuth_rad),
            None if diagnostics is None else _rounded(diagnostics.reference_elevation_rad),
            None if diagnostics is None else _rounded(diagnostics.reference_azimuth_rad),
            None if diagnostics is None else _rounded(diagnostics.optical_ttc_s),
            None if diagnostics is None else diagnostics.descent_feasible,
            None if diagnostics is None else _rounded(diagnostics.requested_tilt_rad),
            None if diagnostics is None else _rounded(diagnostics.measured_tilt_rad),
            None if diagnostics is None else diagnostics.angle_limited,
            None if diagnostics is None else diagnostics.vertical_limited,
            None if diagnostics is None else diagnostics.thrust_saturated,
            None if demand is None else _rounded(demand.roll_rad),
            None if demand is None else _rounded(demand.pitch_rad),
            None if demand is None else _rounded(demand.yaw_rate_rad_s),
            None if demand is None else _rounded(demand.thrust_n),
            command.roll, command.pitch, command.yaw, command.throttle, command.arm)
