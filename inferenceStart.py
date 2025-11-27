import subprocess
import os

# Start Webots without blocking
webots_process = subprocess.Popen(
    ["webots", "--port=1235", f"{__file__[:-17]}worlds/inference.wbt"]
)

# Start controller
webots_controller = os.path.join(os.environ["WEBOTS_HOME"], "webots-controller")
subprocess.run(
    [webots_controller, "--port=1235", f"{__file__[:-17]}controllers/noonRobotTrainer/noonRobotTrainer.py"]
)