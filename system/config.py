import os
import json
import shutil
import logging
import threading
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

CONFIG_FILE = "config.json"
CONFIG_TEMPLATE = "config_template.json"

# Thread-safe lock for config operations
config_lock = threading.RLock()

def ensure_config_exists() -> None:
    """Ensure config.json exists, create from template if it doesn't."""
    with config_lock:
        if not os.path.exists(CONFIG_FILE):
            if not os.path.exists(CONFIG_TEMPLATE):
                raise FileNotFoundError(f"Configuration template file {CONFIG_TEMPLATE} not found!")
            logger.info(f"Creating {CONFIG_FILE} from template...")
            shutil.copy2(CONFIG_TEMPLATE, CONFIG_FILE)
            logger.info(f"Created {CONFIG_FILE} from template.")

def read(section: Optional[str] = None) -> Dict[str, Any]:
    """Read configuration from JSON file.
    
    Args:
        section: Optional section name to read. If None, returns entire config.
        
    Returns:
        Configuration dictionary.
    """
    ensure_config_exists()
    
    with config_lock:
        try:
            with open(CONFIG_FILE, 'r') as f:
                config = json.load(f)
        except json.JSONDecodeError as e:
            logger.error(f"Error parsing {CONFIG_FILE}: {e}")
            raise
        except Exception as e:
            logger.error(f"Error reading {CONFIG_FILE}: {e}")
            raise
        
        if section is not None:
            if section not in config:
                raise KeyError(f"Section '{section}' not found in config")
            return config[section]
        return config

def write(section: str, key: Optional[str], value: Any) -> None:
    """Write configuration to JSON file.
    
    Args:
        section: Section name to write to
        key: Key within section to write. If None, value should be dict to replace entire section
        value: Value to write
    """
    ensure_config_exists()
    
    with config_lock:
        try:
            with open(CONFIG_FILE, 'r') as f:
                config = json.load(f)
        except Exception as e:
            logger.error(f"Error reading {CONFIG_FILE}: {e}")
            raise

        if key is None:
            config[section] = value
        else:
            if section not in config:
                config[section] = {}
            config[section][key] = value

        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
        except Exception as e:
            logger.error(f"Error writing to {CONFIG_FILE}: {e}")
            raise
