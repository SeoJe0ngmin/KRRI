"""Selected delayed-linear ON-command regressions; coast is not fitted."""
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path

from .endpoint_response import DelayedLinearEndpointResponse
from .rotation_artifact import (
    ArtifactValidationError, RotationResponseSelection, canonical_model_sha256,
    _no_duplicate_object, _reject_json_constant, _finite_number,
)


@dataclass(frozen=True)
class DelayedLinearTrajectoryResponse(DelayedLinearEndpointResponse):
    endpoint_only: bool = False
    trajectory_only: bool = True
    uses_embedded_inertia: bool = False
    source: str = '2026-09-15 142911 fixed-camera tag ON-command fit; no coast'

    def angle_at(self, elapsed_sec):
        return self.total_angle(elapsed_sec)

    def rate_at(self, elapsed_sec):
        if not math.isfinite(float(elapsed_sec)) or elapsed_sec < 0:
            raise ValueError('Elapsed time must be finite and nonnegative')
        return self.slope_deg_s if elapsed_sec > self.startup_delay_sec else 0.

    def plan(self, *args, **kwargs):
        plan = super().plan(*args, **kwargs)
        return replace(plan, predicted_stop_angle_deg=plan.predicted_total_deg,
                       predicted_stop_rate_deg_s=self.rate_at(plan.hold_sec),
                       note='ON-command trajectory; coast unmodeled; settle and remeasure')


def load_trajectory_data(path, inference_model_path):
    payload = Path(path).read_bytes()
    data = json.loads(payload, object_pairs_hook=_no_duplicate_object,
                      parse_constant=_reject_json_constant)
    if (data.get('schema_version') != 1 or
            data.get('model_type') != 'selected_delayed_linear_trajectory' or
            data.get('operator_selected') is not True):
        raise ArtifactValidationError('Unsupported trajectory selection')
    model_id = canonical_model_sha256(inference_model_path)
    independent = (data.get('pose_model_independent') is True and
                   data.get('calibration_measurements') == 'external_laser_and_fixed_camera_tag')
    # This trajectory calibration uses external laser/tag measurements, not
    # pallet-network output. Keep its original model hash as provenance while
    # binding the returned runtime identity to the model actually selected.
    if not independent and data.get('model_sha256') != model_id:
        raise ArtifactValidationError('Trajectory selection and runtime model hashes differ')
    if data.get('rotation_strength') != 30 or data.get('forward_command_byte') != 67:
        raise ArtifactValidationError('Unexpected calibrated CAN strengths')
    if data.get('rotation_training_session') != '20260915_142911_607501_8f10e8':
        raise ArtifactValidationError('Unexpected rotation training session')
    for name, limit in (('forward', 2.), ('rotation', 60.)):
        p = data[name]
        k = _finite_number(p['slope'], name+' slope')
        delay = _finite_number(p['effective_delay_s'], name+' delay')
        intercept = _finite_number(p['intercept'], name+' intercept')
        hi = _finite_number(p['fit_max_s'], name+' range')
        sd = _finite_number(p['errors']['SD'], name+' residual SD')
        if not (0 < k <= limit and 0 <= delay < hi and hi > 0 and sd >= 0):
            raise ArtifactValidationError('Invalid trajectory parameters')
        if not math.isclose(intercept, -k*delay, abs_tol=1e-9):
            raise ArtifactValidationError('Intercept and effective delay disagree')
        if p['trial_count'] != (10 if name == 'forward' else 6):
            raise ArtifactValidationError('Unexpected trajectory training trial count')
    return data, 'sha256:'+hashlib.sha256(payload).hexdigest(), model_id


def load_selected_trajectory(path, inference_model_path):
    data, artifact_id, model_id = load_trajectory_data(path, inference_model_path)
    p = data['rotation']
    response = DelayedLinearTrajectoryResponse(
        p['slope'], p['effective_delay_s'], 0., p['fit_max_s'], p['errors']['SD'])
    adjustment = data.get('heuristic_adjustment')
    if adjustment:
        shift = adjustment.get('effective_delay_delta_s_by_model', {}).get(
            'rotation', adjustment['effective_delay_delta_s'])
        response = replace(response, source=response.source +
                           f"; heuristic effective-delay shift {shift:+.3f}s (not refitted)")
    return RotationResponseSelection(response, True, str(path), artifact_id, model_id, '')
