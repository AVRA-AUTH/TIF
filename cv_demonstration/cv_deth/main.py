"""
Bulletproof Computer Vision Demonstrator - WebUI Launcher.
Runs the Live VLM WebUI Server matching NVIDIA live-vlm-webui architecture on http://localhost:8090.
"""

import os
os.environ.pop("SSLKEYLOGFILE", None)

from server import start_server

if __name__ == "__main__":
    start_server()
