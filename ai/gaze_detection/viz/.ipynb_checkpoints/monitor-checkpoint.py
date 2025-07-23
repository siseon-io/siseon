# gaze_detection/viz/monitor.py

def init_tensorboard(log_dir):
    from torch.utils.tensorboard import SummaryWriter
    writer = SummaryWriter(log_dir)
    return writer

def log_metrics(writer, metrics: dict, step: int):
    for k,v in metrics.items():
        writer.add_scalar(k, v, step)
