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
subprocess.run(
    ["webots", "--port=1235", f"{__file__[:-17]}worlds/inference.wbt"]
)


