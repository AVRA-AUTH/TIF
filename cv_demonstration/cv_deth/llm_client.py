"""
Local Vision-Language Model (VLM) Client.
Supports Ollama (moondream, llava, llama3.2-vision, qwen2-vl) for fast, robust, local AI.
Also supports fallback to open local endpoints.
"""

import os
import threading
import json
import base64
import time
import cv2
import urllib.request
from config import PREFERRED_VLM_MODEL, VLM_COOLDOWN_SECONDS

class LocalLLMClient:
    def __init__(self, host="http://localhost:11434"):
        self.model_name = PREFERRED_VLM_MODEL
        self.host = host
        self.cache = {}
        self.lock = threading.Lock()
        self.ollama_available = False
        self.is_busy = False
        self.last_query_time = 0

        print(f"[Local VLM] Checking local Ollama VLM backend at {self.host}...")
        threading.Thread(target=self._check_ollama, daemon=True).start()

    def _check_ollama(self):
        try:
            req = urllib.request.Request(f"{self.host}/api/tags")
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                if resp.status == 200:
                    data = json.loads(resp.read().decode('utf-8'))
                    models = [m.get("name", "") for m in data.get("models", [])]
                    print(f"[Local VLM] Connected to Ollama! Installed models: {models}")
                    
                    # Check if preferred model is in installed list
                    if self.model_name in models:
                        print(f"[Local VLM] Selected Preferred Model: '{self.model_name}'")
                    else:
                        # Auto-select vision model if present
                        for m in models:
                            m_lower = m.lower()
                            if any(v in m_lower for v in ["gemma", "qwen", "moondream", "llava", "vlm"]):
                                self.model_name = m
                                break
                    
                    self.ollama_available = True
                    print(f"[Local VLM] Active Vision Model: '{self.model_name}'")
        except Exception:
            print("[Local VLM] Note: Ollama server not detected at localhost:11434.")

    def fetch_brand_info_async(self, query_text, track_id=None, crop_img=None, callback=None):
        now = time.time()
        if crop_img is None or self.is_busy or (now - self.last_query_time) < VLM_COOLDOWN_SECONDS:
            return

        cache_key = f"{track_id}_{query_text}" if track_id is not None else query_text

        with self.lock:
            if cache_key in self.cache:
                if callback:
                    callback(query_text, self.cache[cache_key])
                return

        self.is_busy = True
        self.last_query_time = now
        thread = threading.Thread(
            target=self._query_worker, 
            args=(query_text, crop_img, cache_key, callback), 
            daemon=True
        )
        thread.start()

    def _query_worker(self, query_text, crop_img, cache_key, callback):
        try:
            # 1. Encode crop image to JPEG base64
            success, buffer = cv2.imencode('.jpg', crop_img)
            if not success:
                return
            b64_image = base64.b64encode(buffer).decode('utf-8')

            if query_text:
                prompt = f"Identify the product in this image (detected text: '{query_text}'). Describe what it is in 1 short sentence."
            else:
                prompt = "Identify the product in this image in 1 short sentence."

            if self.ollama_available:
                print(f"[Local VLM] 🚀 Submitting crop ({cache_key}) to Ollama ('{self.model_name}')... Awaiting response...")
                # Query Ollama API with 120s timeout (allows model VRAM loading)
                payload = {
                    "model": self.model_name,
                    "prompt": prompt,
                    "images": [b64_image],
                    "stream": False
                }
                
                req = urllib.request.Request(
                    f"{self.host}/api/generate",
                    data=json.dumps(payload).encode('utf-8'),
                    headers={"Content-Type": "application/json"}
                )

                with urllib.request.urlopen(req, timeout=120.0) as resp:
                    if resp.status == 200:
                        res_data = json.loads(resp.read().decode('utf-8'))
                        answer = res_data.get("response", "").strip()
                        if answer:
                            print(f"[Local VLM] ✅ Received response for ({cache_key}): \"{answer}\"")
                            result = {
                                "brand_name": query_text.title(),
                                "summary": answer[:150] + ("..." if len(answer) > 150 else ""),
                                "category": f"Local VLM ({self.model_name})"
                            }
                            with self.lock:
                                self.cache[cache_key] = result
                            if callback:
                                callback(query_text, result)
                            return

            # Fallback mock summary if Ollama server is not currently running
            fallback_res = {
                "brand_name": query_text.title(),
                "summary": f"Detected item '{query_text}'. Start Ollama ('ollama run {self.model_name}') for live VLM analysis.",
                "category": "Local AI (Standby)"
            }
            with self.lock:
                self.cache[cache_key] = fallback_res
            if callback:
                callback(query_text, fallback_res)

        except Exception as e:
            print(f"[Local VLM] ⚠️ Query note ({cache_key}): {e}")
        finally:
            self.is_busy = False

    def get_cached_enrichment(self, query_text):
        with self.lock:
            return self.cache.get(query_text, None)
