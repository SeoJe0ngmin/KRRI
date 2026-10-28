"""Full FP32 inference policy and the existing predictor's single-frame path."""
import torch
from .performance import span, timed_lock, gpu_span


def configure_fp32_device(use_gpu, device_index):
    if not use_gpu:
        return 'cpu'
    if not torch.cuda.is_available():
        raise RuntimeError('GPU inference requires CUDA. Start with RUN_JETSON.sh and .venv-jetson.')
    torch.cuda.set_device(device_index)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = True
    return f'cuda:{device_index}'


@torch.inference_mode()
def single_frame_result(predictor, image, confidence):
    """Reuse unchanged preprocessing, model, filtering and coordinate restoration.

    The live perception API consumes one numpy frame and never asks YOLO to
    save/track/display anything. Avoid source-loader setup and six profiling
    synchronizations per frame; returned CPU keypoints synchronize completion.
    The predictor lock serializes callers.
    """
    with timed_lock(predictor._lock, 'inference.predictor_lock'):
        predictor.args.conf = confidence
        predictor.batch = (['frame'], [image], [''])
        with span('inference.tensor_preprocess_mixed', 'host_mixed'):
            tensor = predictor.preprocess([image])
        with span('inference.forward_launch', 'host_mixed'):
            with gpu_span('inference.forward_stream', tensor.device):
                predictions = predictor.inference(tensor)
        with span('inference.postprocess_mixed', 'host_mixed'):
            return predictor.postprocess(predictions, tensor, [image])[0]
