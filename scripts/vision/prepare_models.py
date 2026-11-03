"""Prepare local TensorRT engines from pinned upstream model sources."""
import hashlib
import importlib.util
import json
import os
import shutil
import urllib.request

import gdown
import torch
from ultralytics import YOLO

from .settings import model_dir
from .tensorrt_engine import build

OSNET_REVISION = 'f8cd150fdf77e8d9e1ed143b7f308c2c609ded50'
OSNET_WEIGHTS = '1Kkx2zW89jq_NETu4u42CFZTMVD5Hwm6e'


def download(url, path):
    if not path.exists():
        temporary = path.with_suffix(path.suffix+'.tmp')
        urllib.request.urlretrieve(url, temporary)
        temporary.replace(path)


def main():
    visible_devices = os.environ.get('CUDA_VISIBLE_DEVICES')
    torch.set_num_threads(2)
    root = model_dir()
    root.mkdir(parents=True, exist_ok=True)
    source = f'https://raw.githubusercontent.com/KaiyangZhou/deep-person-reid/{OSNET_REVISION}/'
    download(source+'torchreid/models/osnet.py', root/'osnet.py')
    download(source+'LICENSE', root/'OSNet-LICENSE')
    weights = root/'osnet_msmt17.pth'
    if not weights.exists():
        temporary = weights.with_suffix('.tmp')
        gdown.download(id=OSNET_WEIGHTS, output=str(temporary), quiet=False)
        temporary.replace(weights)
    spec = importlib.util.spec_from_file_location('finav_osnet', root/'osnet.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    network = module.osnet_x0_25(num_classes=1, pretrained=False).eval()
    state = torch.load(weights, map_location='cpu', weights_only=True)
    state = state.get('state_dict', state)
    state = {key.removeprefix('module.'): value for key, value in state.items()}
    state = {key: value for key, value in state.items() if not key.startswith('classifier.')}
    missing, unexpected = network.load_state_dict(state, strict=False)
    if set(missing) != {'classifier.weight', 'classifier.bias'} or unexpected:
        raise RuntimeError(f'Unexpected OSNet checkpoint: {missing}, {unexpected}')
    if not (root/'osnet.onnx').exists():
        torch.onnx.export(network, torch.zeros(1, 3, 256, 128), str(root/'osnet.onnx'),
                          opset_version=17, input_names=['images'], output_names=['features'])
    download('https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt', root/'yolov8n.pt')
    if not (root/'yolov8n.onnx').exists():
        exported = YOLO(root/'yolov8n.pt').export(format='onnx', imgsz=320, batch=1,
                      dynamic=False, simplify=False, opset=17, device='cpu')
        if str(root/'yolov8n.onnx') != str(exported):
            shutil.copy2(exported, root/'yolov8n.onnx')
    # Ultralytics CPU export hides CUDA globally; restore it before building GPU engines.
    if visible_devices is None:
        os.environ.pop('CUDA_VISIBLE_DEVICES', None)
    else:
        os.environ['CUDA_VISIBLE_DEVICES'] = visible_devices
    for name in ('yolov8n', 'osnet'):
        if not (root/f'{name}.engine').exists():
            print(f'Building {name} TensorRT engine (first build can take several minutes)...', flush=True)
            build(root/f'{name}.onnx', root/f'{name}.engine')
    import tensorrt
    metadata = {'osnet_revision': OSNET_REVISION, 'osnet_weights_google_drive_id': OSNET_WEIGHTS,
                'osnet_training': 'MSMT17 combineall', 'tensorrt': tensorrt.__version__,
                'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir()
                           if p.suffix in ('.pth', '.pt', '.onnx', '.engine', '.py')}}
    (root/'manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(f'Models ready: {root}')


if __name__ == '__main__':
    main()
