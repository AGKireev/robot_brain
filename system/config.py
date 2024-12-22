import os
import json
import threading

this_path = os.path.dirname(os.path.realpath(__file__))
config_file = os.path.join(this_path, "../config.json")

config_lock = threading.RLock()

if not os.path.exists(config_file):
    raise FileNotFoundError(f"File '{config_file}' not found.")


def read(param: str | None = None) -> any:
    with config_lock:
        with open(config_file, "r") as f:
            try:
                config_data = json.load(f)
            except json.JSONDecodeError:
                raise ValueError(f"File '{config_file}' is empty or contains invalid JSON.")

            if param:
                if param not in config_data:
                    raise ValueError(f"Parameter '{param}' not found in '{config_file}'.")
                return config_data[param]

            if not config_data:
                raise ValueError(f"File '{config_file}' is empty.")
            return config_data


def write(key: str, sub_key: str | None = None, value: any = None):
    existing_data = read()
    
    if sub_key:
        existing_data[key][sub_key] = value
    else:
        existing_data[key] = value

    with config_lock:
        with open(config_file, "w") as f:
            json.dump(existing_data, f, indent=4)
