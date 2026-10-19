"""Tracking, correction cadence and local control eligibility are separate facts."""
import math

QUALITY_VERSION=2   # 2 (2026-10-19): bridge counts IMU gaps instead of faulting; older binaries are refused
QUALITY_FIELDS=('quality_version','tracking_valid','continued_features','state_stamp',
                'msckf_update_features','slam_update_features','correction_applied','correction_age_s')
REBUILD_MESSAGE='OpenVINS 브리지 업데이트 필요: bash scripts/rebuild_openvins.sh'


def check_message_schema(message_type):
    fields=message_type.get_fields_and_field_types()
    if any(key not in fields for key in QUALITY_FIELDS):raise RuntimeError(REBUILD_MESSAGE)


def quality(packet,cfg,continuity=True):
    if packet.get('quality_version')!=QUALITY_VERSION:raise RuntimeError(REBUILD_MESSAGE)
    state_delta=float(packet['state_stamp'])-packet['stamp']
    state_current=math.isfinite(state_delta) and abs(state_delta)<1e-5
    features=int(packet['features']);continued=int(packet['continued_features'])
    tracking=bool(packet['initialized'] and state_current and packet['tracking_valid'] and
                  min(features,continued)>=cfg['uncertainty']['min_features'])
    age=float(packet['correction_age_s']);known=math.isfinite(age) and age>=0
    limit=cfg['openvins']['max_correction_gap_s']
    lag=float(packet['transport_lag_s']);imu_age=packet['arrival']-packet.get('imu_arrival',-math.inf)
    if not packet['initialized']:reason='openvins_initializing'
    elif not state_current:reason='state_timestamp_mismatch'
    elif not tracking:reason='insufficient_continuous_tracks'
    elif not continuity:reason='relative_pose_discontinuity'
    elif not math.isfinite(lag) or not 0<=lag<=cfg['tuning']['motion_age_s']:reason='vio_output_delayed'   # the controller re-checks 0.3 s at rest
    elif not math.isfinite(imu_age) or not 0<=imu_age<=cfg['tuning']['imu_age_s']:reason='imu_stale'
    elif not known:reason='waiting_for_correction'
    elif age>limit:reason='correction_stale'
    else:reason='openvins_corrected' if packet['correction_applied'] else 'openvins_tracking_between_updates'
    ok=reason in ('openvins_corrected','openvins_tracking_between_updates')
    return dict(ok=ok,reason=reason,tracking_ok=tracking,state_current=state_current,
        corrected_this_frame=bool(packet['correction_applied']),correction_recent=known and age<=.2,
        correction_age_s=age if known else None,max_correction_gap_s=limit,
        correction_source=('zupt' if packet['zero_velocity_update'] else 'visual') if packet['correction_applied'] else 'none_this_frame',
        msckf_update_features=int(packet['msckf_update_features']),slam_update_features=int(packet['slam_update_features']),
        features=features,continued_features=continued,state_time_error_s=state_delta,
        transport_lag_s=lag,imu_age_s=imu_age,relative_continuity=bool(continuity),
        tracked_points=packet.get('points',[]))
