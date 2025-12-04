import subprocess
import os
import atexit
from envUtil import cleanUp
import signal

atexit.register(cleanUp)
def signal_handler(sig, frame):
    cleanUp()
    raise SystemExit("Exiting due to signal")
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Start Webots without blocking
webots_process = subprocess.Popen(
    ["webots", "--port=1235", f"{__file__[:-17]}worlds/inference.wbt"]
)

# Start controller
webots_controller = os.path.join(os.environ["WEBOTS_HOME"], "webots-controller")
subprocess.run(
    [webots_controller, "--port=1235", f"{__file__[:-17]}controllers/noonRobotTrainer/noonRobotTrainer.py"]
)

