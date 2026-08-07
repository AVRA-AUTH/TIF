"""
Zero-Latency Threaded Camera Module with Auto-Probing & DirectShow Support.
Automatically detects and connects to active webcams (indices 0, 1, 2, 3, 4) on Windows.
"""

import cv2
import threading
import time


class ThreadedCamera:
    def __init__(self, source=0, width=1280, height=720):
        self.requested_source = source
        self.width = width
        self.height = height
        self.cap = None
        self.source = None
        
        # 1. Attempt connection to requested source and auto-probe fallback indices
        candidate_sources = []
        if isinstance(source, int):
            candidate_sources = [source] + [i for i in [0, 1, 2, 3, 4] if i != source]
        else:
            candidate_sources = [source]

        for s in candidate_sources:
            print(f"[*] Probing camera index/source: {s}...")
            # Try DirectShow first on Windows (faster & non-blocking)
            cap = None
            if isinstance(s, int):
                cap = cv2.VideoCapture(s, cv2.CAP_DSHOW)
                if not cap.isOpened():
                    cap.release()
                    cap = cv2.VideoCapture(s)
            else:
                cap = cv2.VideoCapture(s)

            if cap is not None and cap.isOpened():
                if isinstance(s, int):
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                grabbed, frame = cap.read()
                if grabbed and frame is not None:
                    self.cap = cap
                    self.source = s
                    print(f"[+] Successfully connected to Camera Source: {s} ({frame.shape[1]}x{frame.shape[0]})")
                    break
                else:
                    cap.release()

        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError(f"Could not connect to any camera source (tried: {candidate_sources}). Please check USB/webcam connection.")

        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()
        self.thread = None

    def start(self):
        if self.started:
            return self
        self.started = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self

    def _update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            if not grabbed:
                time.sleep(0.01)
                continue
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self.read_lock:
            if not self.grabbed or self.frame is None:
                return False, None
            return True, self.frame.copy()

    def stop(self):
        self.started = False
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
