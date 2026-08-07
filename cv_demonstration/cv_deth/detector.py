"""
Professional Production Computer Vision Engine.
Features Dual-Engine Tracking (YOLO11l + YOLO-World v2), Precision False Positive Filtering,
MobileSAM Mask Segmentation, High-Accuracy OCR, and Asynchronous Web Knowledge Enrichment.
"""

import os
os.environ.pop("SSLKEYLOGFILE", None)

import threading
import time
import torch
import cv2
import re
import numpy as np
from collections import defaultdict, deque, Counter
from ultralytics import YOLO, SAM
from ocr_reader import OCRReader
from llm_client import LocalLLMClient
from config import (
    PRIMARY_DETECTOR_MODEL,
    WORLD_DETECTOR_MODEL,
    SAM_MODEL_NAME,
    DEFAULT_PROMPTS,
    CONFIDENCE_THRESHOLD,
    WORLD_CONFIDENCE_THRESHOLD,
    VOTE_HISTORY_DEPTH,
    MIN_BOX_AREA,
    MAX_BOX_COVERAGE,
    DUPLICATE_IOU_THRESHOLD,
    ACTIVE_ENGINE_MODE,
    EXCLUDED_CLASSES
)


class YoloWorldDetector:
    def __init__(self, model_name=PRIMARY_DETECTOR_MODEL, prompts=None, conf_threshold=CONFIDENCE_THRESHOLD):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.engine_mode = ACTIVE_ENGINE_MODE
        
        # 1. Load Primary SOTA YOLO11 Large
        print(f"[Engine] Loading Primary SOTA Detector ({PRIMARY_DETECTOR_MODEL}) on {self.device.upper()}...")
        self.yolo_primary = YOLO(PRIMARY_DETECTOR_MODEL)
        self.yolo_primary.to(self.device)

        # 2. Load Open-Vocabulary YOLOv8s-Worldv2
        print(f"[Engine] Loading Open-Vocabulary Detector ({WORLD_DETECTOR_MODEL}) on {self.device.upper()}...")
        self.yolo_world = YOLO(WORLD_DETECTOR_MODEL)
        self.yolo_world.to(self.device)
        
        self.prompts = prompts if prompts is not None else DEFAULT_PROMPTS
        self.update_prompts(self.prompts)
        
        # 3. Load MobileSAM Engine
        try:
            print(f"[MobileSAM] Loading MobileSAM Engine ({SAM_MODEL_NAME}) on {self.device.upper()}...")
            self.sam = SAM(SAM_MODEL_NAME)
            self.sam.to(self.device)
            self.enable_sam = True
        except Exception as e:
            print(f"[MobileSAM] Warning: ({e}). Active with polygon fallback.")
            self.enable_sam = False

        # 4. Initialize OCR & Local VLM Knowledge Enricher
        self.ocr = OCRReader()
        self.enricher = LocalLLMClient()
        self.current_brand_info = None

        self.running = False
        self.thread = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.result_lock = threading.Lock()
        
        # Temporal Box Smoothing (EMA)
        self.track_memory = {}
        self.alpha = 0.45
        
        # Temporal Majority Voting
        self.track_vote_history = defaultdict(lambda: deque(maxlen=VOTE_HISTORY_DEPTH))
        self.track_conf_history = defaultdict(lambda: deque(maxlen=VOTE_HISTORY_DEPTH))

        # Spatial Anchor Stabilizer Memory (Eliminates Track ID Churn & Label Jitter)
        self.spatial_ocr_memory = []

        self.latest_results = {
            "objects": [],
            "person_profile": {},
            "brand_enrichment": None,
            "mode": self.engine_mode
        }
        self.new_frame_available = False

    def toggle_mode(self):
        self.engine_mode = 1 if self.engine_mode == 2 else 2
        print(f"[Engine Mode] Active: MODE {self.engine_mode} ({'Pure YOLO11l' if self.engine_mode == 1 else 'Hybrid Dual-Engine'})")
        return self.engine_mode

    def update_prompts(self, prompts):
        self.prompts = [p.strip() for p in prompts if p.strip()]
        if not self.prompts:
            self.prompts = ["person"]
        print(f"[YOLO-World] Updating zero-shot classes ({len(self.prompts)} active)...")
        self.yolo_world.set_classes(self.prompts)

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._inference_loop, daemon=True)
        self.thread.start()

    def process_frame_async(self, frame):
        with self.frame_lock:
            if not self.new_frame_available:
                self.latest_frame = frame.copy()
                self.new_frame_available = True

    def _inference_loop(self):
        while self.running:
            frame_to_process = None
            with self.frame_lock:
                if self.new_frame_available and self.latest_frame is not None:
                    frame_to_process = self.latest_frame
                    self.new_frame_available = False
            
            if frame_to_process is None:
                time.sleep(0.005)
                continue
            
            h_img, w_img, _ = frame_to_process.shape
            frame_area = float(h_img * w_img)

            objects_list = []
            sam_boxes = []
            person_box = None
            
            detected_gender = None
            detected_hair_prompt = None
            detected_glasses = False

            # Keywords to strictly exclude from segmentation masks (Humans & Attire)
            human_and_clothes_keywords = [
                "person", "man", "woman", "boy", "girl", "child", "hair", "bearded",
                "shirt", "jacket", "pants", "jeans", "hoodie", "suit", "blazer", "dress", "skirt", "shorts"
            ]

            # 1. Primary YOLO11 Tracking
            y11_results = self.yolo_primary.track(
                frame_to_process,
                persist=True,
                tracker="bytetrack.yaml",
                conf=CONFIDENCE_THRESHOLD,
                verbose=False
            )

            if y11_results and len(y11_results) > 0:
                res = y11_results[0]
                boxes = res.boxes
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    cls_ids = boxes.cls.cpu().numpy().astype(int)
                    confs = boxes.conf.cpu().numpy()
                    track_ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else [None] * len(xyxy)
                    names = res.names
                    
                    for i in range(len(xyxy)):
                        raw_box = xyxy[i].tolist()
                        cls_name = names[cls_ids[i]]
                        l_lower = cls_name.lower()
                        
                        if l_lower in EXCLUDED_CLASSES:
                            continue
                        
                        if not self._is_valid_bounding_box(raw_box, frame_area, is_person=(l_lower == "person")):
                            continue

                        t_id = int(track_ids[i]) if track_ids[i] is not None else None
                        smooth_box = self._apply_ema_smoothing(t_id, raw_box)
                        
                        if l_lower == "person":
                            person_box = smooth_box
                        elif not any(k in l_lower for k in human_and_clothes_keywords):
                            stable_label, stable_conf = self._get_stable_label_vote(t_id, cls_name.capitalize(), float(confs[i]))
                            
                            objects_list.append({
                                "box": smooth_box,
                                "label": stable_label,
                                "conf": stable_conf,
                                "track_id": t_id,
                                "polygon": None,
                                "ocr_text": None
                            })
                            sam_boxes.append(smooth_box)

            # 2. Hybrid Mode 2 Zero-Shot Tracking
            if self.engine_mode == 2:
                world_results = self.yolo_world.track(
                    frame_to_process,
                    persist=True,
                    tracker="bytetrack.yaml",
                    conf=WORLD_CONFIDENCE_THRESHOLD,
                    verbose=False
                )
                
                if world_results and len(world_results) > 0:
                    res_w = world_results[0]
                    boxes_w = res_w.boxes
                    if boxes_w is not None and len(boxes_w) > 0:
                        xyxy_w = boxes_w.xyxy.cpu().numpy()
                        cls_ids_w = boxes_w.cls.cpu().numpy().astype(int)
                        confs_w = boxes_w.conf.cpu().numpy()
                        track_ids_w = boxes_w.id.cpu().numpy().astype(int) if boxes_w.id is not None else [None] * len(xyxy_w)
                        
                        # SORT BY CONFIDENCE (Descending) to ensure highest-confidence class wins NMS overlap check
                        sorted_indices = np.argsort(-confs_w)
                        
                        for i in sorted_indices:
                            raw_box = xyxy_w[i].tolist()
                            cls_id = cls_ids_w[i]
                            label = self.prompts[cls_id] if cls_id < len(self.prompts) else f"Class_{cls_id}"
                            l_lower = label.lower()

                            if l_lower in EXCLUDED_CLASSES:
                                continue

                            if not self._is_valid_bounding_box(raw_box, frame_area, is_person=(l_lower in ["man", "woman", "person", "child"])):
                                continue

                            t_id = int(track_ids_w[i]) if track_ids_w[i] is not None else None
                            smooth_box = self._apply_ema_smoothing(t_id, raw_box)
                            
                            if l_lower in ["man", "woman", "boy", "girl", "child", "person"]:
                                person_box = smooth_box
                                if not detected_gender:
                                    detected_gender = label.capitalize()
                            elif "hair" in l_lower:
                                detected_hair_prompt = label.capitalize()
                            elif "glasses" in l_lower or "sunglasses" in l_lower:
                                detected_glasses = True
                                if not self._is_box_overlapping(smooth_box, sam_boxes):
                                    stable_label, stable_conf = self._get_stable_label_vote(t_id, "Glasses", float(confs_w[i]))
                                    objects_list.append({"box": smooth_box, "label": stable_label, "conf": stable_conf, "track_id": t_id, "polygon": None, "ocr_text": None})
                                    sam_boxes.append(smooth_box)
                            elif not any(k in l_lower for k in human_and_clothes_keywords) and l_lower != "hand":
                                if not self._is_box_overlapping(smooth_box, sam_boxes):
                                    stable_label, stable_conf = self._get_stable_label_vote(t_id, label.capitalize(), float(confs_w[i]))
                                    objects_list.append({"box": smooth_box, "label": stable_label, "conf": stable_conf, "track_id": t_id, "polygon": None, "ocr_text": None})
                                    sam_boxes.append(smooth_box)

            # 3. MobileSAM Segmentation for Carried Objects
            if self.enable_sam and len(sam_boxes) > 0:
                try:
                    sam_results = self.sam(
                        frame_to_process,
                        bboxes=sam_boxes,
                        verbose=False,
                        device=self.device
                    )
                    if sam_results and len(sam_results) > 0 and sam_results[0].masks is not None:
                        poly_masks = sam_results[0].masks.xy
                        for idx in range(min(len(objects_list), len(poly_masks))):
                            poly = poly_masks[idx]
                            if poly is not None and len(poly) > 2:
                                objects_list[idx]["polygon"] = poly.astype(np.int32)
                except Exception:
                    pass

            for obj in objects_list:
                if obj["polygon"] is None:
                    obj["polygon"] = self._extract_fallback_perimeter(frame_to_process, obj["box"])

            # 4. Perform High-Precision OCR & Real Product Label Override (4.0s refresh interval)
            for obj in objects_list:
                ocr_t = self._perform_ocr_on_object(
                    frame_to_process, 
                    obj["box"], 
                    obj["label"], 
                    obj["track_id"], 
                    polygon=obj["polygon"]
                )
                if ocr_t:
                    obj["ocr_text"] = ocr_t
                    obj["display_label"] = ocr_t.upper()
                else:
                    obj["display_label"] = obj["label"].upper()

            # 5. Fast Live Greek & English Product Web Knowledge Search (Tavily AI Engine)
            for obj in objects_list:
                if obj.get("ocr_text"):
                    ocr_t = obj.get("ocr_text")
                    if len(ocr_t) > 6:
                        import threading
                        from ocr_reader import fetch_web_preview, extract_search_keywords
                        
                        def run_bg_search(text_q, target_obj):
                            clean_keywords = extract_search_keywords(text_q)
                            if len(clean_keywords) > 4:
                                is_new = True
                                if hasattr(self, '_last_web_keywords'):
                                    prev_w = set(self._last_web_keywords.lower().split())
                                    curr_w = set(clean_keywords.lower().split())
                                    inter = prev_w.intersection(curr_w)
                                    if len(inter) >= 2 or (len(curr_w) > 0 and len(inter) / len(curr_w) > 0.5):
                                        is_new = False
                                        
                                if is_new:
                                    self._last_web_keywords = clean_keywords
                                    snippet = fetch_web_preview(text_q)
                                    brand_title = clean_keywords.upper()
                                    
                                    self.current_brand_info = {
                                        "brand_name": brand_title,
                                        "summary": snippet,
                                        "category": "Live Product AI Search"
                                    }
                                    
                                    # Override object display label with the Tavily AI identified brand title!
                                    target_obj["display_label"] = brand_title
                        
                        threading.Thread(target=run_bg_search, args=(ocr_t, obj), daemon=True).start()
                        break
            
            # Reset brand info ONLY when no objects are present in scene
            if len(objects_list) == 0:
                self.current_brand_info = None

            # 6. Person Profile Characteristics Analysis
            person_profile = self._analyze_person_characteristics(
                frame_to_process, 
                person_box, 
                detected_gender, 
                detected_hair_prompt, 
                detected_glasses
            )

            if len(self.track_memory) > 60:
                self.track_memory.clear()
                self.ocr.clear_cache()

            parsed = {
                "objects": objects_list,
                "person_profile": person_profile,
                "brand_enrichment": self.current_brand_info,
                "mode": self.engine_mode
            }

            with self.result_lock:
                self.latest_results = parsed

    def _is_valid_bounding_box(self, box, frame_area, is_person=False):
        """Precision sanity check to filter out degenerate boxes & false positive background noise."""
        bw = box[2] - box[0]
        bh = box[3] - box[1]
        area = bw * bh
        
        if area < MIN_BOX_AREA:
            return False
        
        if not is_person and (area / frame_area) > MAX_BOX_COVERAGE:
            return False

        aspect_ratio = bw / max(1.0, bh)
        if aspect_ratio > 6.0 or aspect_ratio < 0.15:
            return False

        return True

    def _on_brand_info_fetched(self, query_text, info_dict):
        self.current_brand_info = info_dict

    def _perform_ocr_on_object(self, frame, box, label, track_id, polygon=None):
        if not self.ocr.enabled:
            return None

        h, w, _ = frame.shape
        
        # Use full MobileSAM polygon bounds if available for OCR
        if polygon is not None and len(polygon) > 0:
            poly_np = np.array(polygon, dtype=np.int32)
            px, py, pw, ph = cv2.boundingRect(poly_np)
            x1, y1 = max(0, px), max(0, py)
            x2, y2 = min(w, px + pw), min(h, py + ph)
        else:
            x1, y1, x2, y2 = map(int, box)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
        
        if (x2 - x1) < 20 or (y2 - y1) < 20:
            return self.ocr.get_cached_text(track_id) if track_id is not None else None

        # Check if 4.0 seconds have elapsed since last read for this tracked object (prevents fast refresh)
        if track_id is not None and not self.ocr.should_reread(track_id, interval=4.0):
            return self.ocr.get_cached_text(track_id)

        crop = frame[y1:y2, x1:x2].copy()
        text = self.ocr.read_text_from_crop(crop)
        
        if text and len(text.strip()) >= 2:
            if track_id is not None:
                self.ocr.set_cached_text(track_id, text)
                print(f"[OCR 4s Refresh] Track #{track_id} '{label}': \"{text}\"")
                return text
            else:
                print(f"[OCR 4s Refresh] '{label}': \"{text}\"")
                return text

        return self.ocr.get_cached_text(track_id) if track_id is not None else None

    def _get_stable_label_vote(self, t_id, detected_label, conf):
        if t_id is None:
            return detected_label, conf
        
        self.track_vote_history[t_id].append(detected_label)
        self.track_conf_history[t_id].append(conf)
        
        majority_label = Counter(self.track_vote_history[t_id]).most_common(1)[0][0]
        avg_conf = float(np.mean(self.track_conf_history[t_id]))
        
        return majority_label, avg_conf

    def _apply_ema_smoothing(self, t_id, raw_box):
        if t_id is not None:
            if t_id in self.track_memory:
                prev_box = self.track_memory[t_id]
                smooth_box = [self.alpha * raw_box[j] + (1 - self.alpha) * prev_box[j] for j in range(4)]
            else:
                smooth_box = raw_box
            self.track_memory[t_id] = smooth_box
            return smooth_box
        return raw_box

    def _is_box_overlapping(self, new_box, existing_boxes, iou_thresh=DUPLICATE_IOU_THRESHOLD):
        for e_box in existing_boxes:
            xi1 = max(new_box[0], e_box[0])
            yi1 = max(new_box[1], e_box[1])
            xi2 = min(new_box[2], e_box[2])
            yi2 = min(new_box[3], e_box[3])
            
            inter_w = max(0, xi2 - xi1)
            inter_h = max(0, yi2 - yi1)
            inter_area = inter_w * inter_h
            
            area1 = (new_box[2] - new_box[0]) * (new_box[3] - new_box[1])
            area2 = (e_box[2] - e_box[0]) * (e_box[3] - e_box[1])
            union_area = area1 + area2 - inter_area
            
            if union_area > 0 and (inter_area / union_area) > iou_thresh:
                return True
        return False

    def _extract_fallback_perimeter(self, frame, box):
        h, w, _ = frame.shape
        x1, y1, x2, y2 = map(int, box)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 <= x1 + 4 or y2 <= y1 + 4:
            return None

        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.Canny(blurred, 30, 120)
        combined = cv2.bitwise_or(thresh, edges)

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest_cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_cnt) < 15:
            return None

        epsilon = 0.010 * cv2.arcLength(largest_cnt, True)
        approx = cv2.approxPolyDP(largest_cnt, epsilon, True)

        polygon_points = approx.reshape(-1, 2)
        polygon_points[:, 0] += x1
        polygon_points[:, 1] += y1

        return polygon_points.astype(np.int32)

    def _analyze_person_characteristics(self, frame, person_box, gender_prompt, hair_prompt, glasses_detected):
        if person_box is None:
            return {
                "gender": gender_prompt if gender_prompt else "Adult",
                "hair": hair_prompt if hair_prompt else "Short Hair",
                "upper_wear": "Casual Attire",
                "lower_wear": "N/A",
                "eyewear": "Glasses Detected" if glasses_detected else "None"
            }

        h_img, w_img, _ = frame.shape
        px1, py1, px2, py2 = map(int, person_box)
        px1, py1 = max(0, px1), max(0, py1)
        px2, py2 = min(w_img, px2), min(h_img, py2)

        crop = frame[py1:py2, px1:px2]
        ch, cw, _ = crop.shape
        if ch < 30 or cw < 30:
            return {
                "gender": gender_prompt if gender_prompt else "Adult",
                "hair": hair_prompt if hair_prompt else "Short Hair",
                "upper_wear": "Casual Shirt",
                "lower_wear": "Pants",
                "eyewear": "Glasses" if glasses_detected else "None"
            }

        hair_roi = crop[0:int(ch * 0.25), :]
        hair_color = self._get_dominant_color_name(hair_roi, is_hair=True)
        hair_str = f"{hair_color} Hair" if not hair_prompt else hair_prompt

        upper_roi = crop[int(ch * 0.22):int(ch * 0.60), :]
        upper_color = self._get_dominant_color_name(upper_roi)

        lower_roi = crop[int(ch * 0.60):ch, :]
        lower_color = self._get_dominant_color_name(lower_roi)

        return {
            "gender": gender_prompt if gender_prompt else "Adult",
            "hair": hair_str,
            "upper_wear": f"{upper_color} Top",
            "lower_wear": f"{lower_color} Bottom",
            "eyewear": "Glasses / Eyewear" if glasses_detected else "Clear View"
        }

    def _get_dominant_color_name(self, roi, is_hair=False):
        if roi.size == 0:
            return "Dark" if is_hair else "Casual"

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        
        avg_v = np.mean(v)
        avg_s = np.mean(s)
        avg_h = np.mean(h)

        if is_hair:
            if avg_v < 60:
                return "Black"
            elif avg_v > 170 and avg_s < 60:
                return "Grey/Blonde"
            elif avg_h < 25 and avg_s > 80:
                return "Blonde/Brown"
            elif avg_h < 15:
                return "Auburn/Red"
            else:
                return "Brown"

        if avg_v < 50:
            return "Black"
        elif avg_v > 200 and avg_s < 40:
            return "White"
        elif avg_s < 40:
            return "Grey"
        elif avg_h < 10 or avg_h > 170:
            return "Red"
        elif 10 <= avg_h < 25:
            return "Orange/Brown"
        elif 25 <= avg_h < 35:
            return "Yellow"
        elif 35 <= avg_h < 85:
            return "Green"
        elif 85 <= avg_h < 130:
            return "Blue"
        elif 130 <= avg_h < 160:
            return "Purple/Pink"
        
        return "Colored"

    def get_latest_results(self):
        with self.result_lock:
            return self.latest_results.copy()

    def stop(self):
        self.running = False
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
