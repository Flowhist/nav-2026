import math
import os
from pathlib import Path

import yaml


def share_dir():
    if os.environ.get('FINAV_SHARE_DIR'):
        return Path(os.environ['FINAV_SHARE_DIR'])
    source = Path(__file__).resolve().parents[2]
    if (source / 'CMakeLists.txt').exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('finav'))


def model_dir():
    return Path(os.environ.get('FINAV_VISION_MODELS',
                str(Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))/'finav/vision')))


def load_config(path=None):
    with open(path or share_dir()/'config/person_follow.yaml') as stream:
        config = yaml.safe_load(stream)
    integer_limits = {'max_people': 16, 'confirm_frames': 20, 'gallery_size': 32, 'min_crop_height': 480}
    for key, value in config.items():
        if key in ('ultrasonic_topic', 'ultrasonic_bus'):
            if not isinstance(value, str) or not value:
                raise ValueError('Invalid '+key)
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid '+key)
        if key in integer_limits and (type(value) is not int or value > integer_limits[key]):
            raise ValueError('Invalid '+key)
    for key in ('enroll_confidence', 'match_similarity', 'match_margin', 'update_similarity', 'max_overlap'):
        if config[key] > 1:
            raise ValueError('Invalid '+key)
    if not (config['frame_timeout'] <= .35 and 1.8 <= config['follow_distance'] <= 4
            and config['max_linear'] <= .15 and config['max_angular'] <= .3
            and config['ultrasonic_stop'] >= .3
            and config['update_similarity'] >= config['match_similarity']):
        raise ValueError('Configuration exceeds validated low-speed envelope')
    return config
