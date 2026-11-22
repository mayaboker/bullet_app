"""Stateful rate, mission, and LOS diagnostics for the App control loop."""
from loguru import logger
from los_tracking.types import State

from .blackboard import StepContext
from .missions import Phase
from .rate_monitor import RateMonitor

LOG_PERIOD_S = 1.0


class RuntimeDiagnostics:
    """Remember reported changes and log periodic flight measurements."""

    def __init__(self, connected_at: float):
        self.rates = RateMonitor(connected_at)
        self.last_log = 0.0
        self.previous_phase = None
        self.previous_result = None
        self.previous_navigation = None
        self.previous_alignment_counts = (0, 0)
        self.previous_estimator_counts = (0, 0, 0, 0, 0, 0)
        self.previous_sync_ready = False

    def observe_events(self, mission, context: StepContext, view, *, target_enabled: bool) -> None:
        """Report rate alerts and mission changes after state is published to View."""
        now = context.now_s
        sample = context.blackboard.telemetry
        for name, event, hz in self.rates.check(now, sample.attitude_hz, sample.vfr_hud_hz):
            if event == 'low':
                logger.warning('{} rate low: {:.1f} Hz', name, hz)
            else:
                logger.info('{} rate recovered: {:.1f} Hz', name, hz)
            view.publish_rate_alert(name, event, hz)
        if mission.phase != self.previous_phase:
            logger.info('state: {} ({})', mission.phase.value, mission.mode.value)
            self.previous_phase = mission.phase
        if mission.result is not None and mission.result != self.previous_result:
            if mission.result == 'hit':
                logger.info('mission result: {}', mission.result)
            else:
                logger.warning('mission result: {}', mission.result)
            self.previous_result = mission.result
        if target_enabled:
            sync_ready = mission.sync_ready(now)
            if sync_ready != self.previous_sync_ready:
                logger.info('MAVLink TIMESYNC {}', 'ready' if sync_ready else 'lost')
                self.previous_sync_ready = sync_ready
        navigation = context.blackboard.features.navigation_result
        if navigation is not None:
            current_navigation = (navigation.state, navigation.reason)
            if current_navigation != self.previous_navigation:
                if navigation.state == State.FAULT:
                    estimator = mission.sensor_estimator
                    tick = estimator.last_tick_s
                    latest_barometer = estimator.barometer_latest
                    logger.warning(
                        'LOS state=fault reason={} vision={} nav_attitude_age={} nav_barometer_age={} '
                        'source_attitude_age={} source_barometer_age={} tick_lag={} latest_barometer_minus_tick={} '
                        'mapped_attitude_age={} attitude_mapping_rejections={} '
                        'barometer_interpolations_total={} barometer_missing_ticks_total={} VFR_HUD={:.1f} Hz',
                        navigation.reason, navigation.diagnostics.vision_reason,
                        None if navigation.diagnostics.attitude_age_s is None else round(navigation.diagnostics.attitude_age_s, 3),
                        None if navigation.diagnostics.barometer_age_s is None else round(navigation.diagnostics.barometer_age_s, 3),
                        None if estimator.attitude_latest is None else round(now - estimator.attitude_latest.timestamp_s, 3),
                        None if latest_barometer is None else round(now - latest_barometer.timestamp_s, 3),
                        None if tick is None else round(now - tick, 3),
                        None if tick is None or latest_barometer is None else round(latest_barometer.timestamp_s - tick, 3),
                        None if mission.last_attitude_mapped_age_s is None else round(mission.last_attitude_mapped_age_s, 3),
                        mission.attitude_mapping_rejections,
                        estimator.barometer_interpolations, estimator.barometer_missing_ticks,
                        sample.vfr_hud_hz)
                else:
                    logger.info('LOS state={} reason={} vision={}', navigation.state.value,
                                navigation.reason, navigation.diagnostics.vision_reason)
                self.previous_navigation = current_navigation

    def observe_periodic(self, mission, context: StepContext, command, view) -> None:
        """Log flight and LOS measurements once per second when altitude exists."""
        now = context.now_s
        altitude = context.altitude_m
        if now - self.last_log < LOG_PERIOD_S or altitude is None:
            return
        sample = context.blackboard.telemetry
        navigation = context.blackboard.features.navigation_result
        datum = mission.ground_altitude_m if mission.ground_altitude_m is not None else altitude
        logger.info('altitude={:.2f} m speed={:.2f} m/s throttle={} armed={} view={}',
                    altitude - datum, mission.controller.speed, command.throttle,
                    context.firmware_armed, view.view_alive(now))
        logger.info('SITL rates ATTITUDE={:.1f} Hz VFR_HUD={:.1f} Hz, ages attitude={:.3f} s altitude={:.3f} s',
                    sample.attitude_hz, sample.vfr_hud_hz,
                    now - sample.attitude_at if sample.attitude_at else float('nan'),
                    now - sample.vfr_hud_at if sample.vfr_hud_at else float('nan'))
        if mission.mission_kind == 'target-hit':
            sync = sample.timesync
            logger.info('TIMESYNC ready={} samples={} rtt_ms={} age_s={} offset_s={}',
                        mission.sync_ready(now), 0 if sync is None else sync.accepted_samples,
                        None if sync is None else round(sync.rtt_ns / 1e6, 2),
                        None if sync is None else round(now - sync.updated_at, 3),
                        None if sync is None else round(sync.offset_ns / 1e9, 6))
        navigation_input = context.blackboard.features.navigation_input
        if mission.phase in (Phase.LOS, Phase.COMMIT) and navigation is not None and navigation_input is not None:
            target, attitude = navigation_input.target, navigation_input.attitude
            counts = (navigation.diagnostics.alignment_waits,
                      navigation.diagnostics.alignment_replacements)
            logger.info('LOS timing image_age={} attitude_age={} image_minus_attitude={} '
                        'aligned_age={} waits={} replacements={} vision={}',
                        None if target is None else round(now - target.timestamp_s, 3),
                        None if attitude is None else round(now - attitude.timestamp_s, 3),
                        None if target is None or attitude is None else round(target.timestamp_s - attitude.timestamp_s, 3),
                        None if navigation.diagnostics.image_age_s is None else round(navigation.diagnostics.image_age_s, 3),
                        counts[0] - self.previous_alignment_counts[0],
                        counts[1] - self.previous_alignment_counts[1],
                        navigation.diagnostics.vision_reason)
            self.previous_alignment_counts = counts
            estimator = mission.sensor_estimator
            estimator_counts = (estimator.skipped_ticks, estimator.expired_boxes,
                                estimator.overflow_boxes, estimator.attitude_interpolations,
                                estimator.barometer_interpolations, estimator.barometer_missing_ticks)
            logger.info('LOS estimator attitude_source_age={} altitude_source_age={} '
                        'mapped_attitude_age={} attitude_mapping_rejections={} '
                        'attitude_prediction_age={} altitude_prediction_age={} '
                        'queue_depth={} skipped_ticks_last_s={} expired_boxes_last_s={} overflow_boxes_last_s={} '
                        'attitude_interpolations_last_s={} barometer_interpolations_last_s={} barometer_missing_ticks_last_s={}',
                        None if estimator.attitude_latest is None else round(now - estimator.attitude_latest.timestamp_s, 3),
                        None if estimator.barometer_latest is None else round(now - estimator.barometer_latest.timestamp_s, 3),
                        None if mission.last_attitude_mapped_age_s is None else round(mission.last_attitude_mapped_age_s, 3),
                        mission.attitude_mapping_rejections,
                        None if estimator.last_attitude_prediction_age_s is None else round(estimator.last_attitude_prediction_age_s, 3),
                        None if estimator.last_altitude_prediction_age_s is None else round(estimator.last_altitude_prediction_age_s, 3),
                        estimator.queue_depth,
                        estimator_counts[0] - self.previous_estimator_counts[0],
                        estimator_counts[1] - self.previous_estimator_counts[1],
                        estimator_counts[2] - self.previous_estimator_counts[2],
                        estimator_counts[3] - self.previous_estimator_counts[3],
                        estimator_counts[4] - self.previous_estimator_counts[4],
                        estimator_counts[5] - self.previous_estimator_counts[5])
            self.previous_estimator_counts = estimator_counts
        self.last_log = now
