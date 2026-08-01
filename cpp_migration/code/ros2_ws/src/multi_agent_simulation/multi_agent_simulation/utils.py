import json
CONFIG_PATH = "/root/ros2_ws/src/shared_infrastructure/config/config.json"

def get_agilex_base_pose():
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return config.get('agilex_base_pose')
    except Exception as e:
        print(f"[Utils] Failed to get agilex_base_pose: {e}")
        return []