#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GLM-4.5V Vision-Language Model client.

Extracted from the duplicated code in look_around.py and capture_at_handover.py.
"""

import io
import os
import base64
import json
import re
import requests
from PIL import Image
from termcolor import cprint
from pathlib import Path


class VLMClient:
    """Thin wrapper around the GLM-4.5V chat-completion API."""

    DEFAULT_API_URL = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
    DEFAULT_MODEL = "glm-4.5v"
    ENV_TOKEN = "GLM_API_TOKEN"

    def __init__(self, api_url=None, api_token=None, model=None):
        self.api_url = api_url or self.DEFAULT_API_URL
        local_token = self._read_local_token()
        self.api_token = (
            api_token
            or local_token
            or os.environ.get(self.ENV_TOKEN)
        )
        self.model = model or self.DEFAULT_MODEL

    @classmethod
    def _read_local_token(cls):
        """Read a git-ignored local .env token without adding dependencies."""
        env_path = Path(__file__).resolve().parents[1] / ".env"
        try:
            for line in env_path.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                if key.strip() == cls.ENV_TOKEN:
                    return value.strip().strip('"').strip("'")
        except OSError:
            pass
        return None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def ground_object(self, rgb_image, object_name, max_prompts=6):
        """Ground one requested object instance in an RGB image.

        The returned box is used only to restrict existing grasp candidates;
        the VLM never produces robot poses or directly authorizes motion.
        Coordinates are in the original RGB image pixel frame.
        """
        original = str(object_name or "").strip()
        if not original:
            return {"prompts": [], "box": None, "found": False}
        if not self.api_token:
            return {"prompts": [original], "box": None, "found": False}

        height, width = rgb_image.shape[:2]
        prompt = (
            "你是机器人视觉目标定位模块。请在图片中定位用户指定的唯一目标实例。\n"
            f"用户目标：{original[:120]}\n"
            f"图片尺寸：宽 {width} 像素，高 {height} 像素\n\n"
            "要求：\n"
            "1. 只返回 JSON 对象，不要 Markdown 或解释。\n"
            "2. prompts 是给开放词汇检测器使用的 2-6 个短语，必须包含用户目标。\n"
            "3. box 只填写最符合用户目标的一个实例框 [x1,y1,x2,y2]，坐标使用 0-1000 归一化格式。\n"
            "4. 不要把其他外观相近但不是目标的物体一起放入 box。\n"
            "5. 如果目标不可见，返回 found=false、box=null。\n"
            '返回格式：{"found":true,"box_format":"normalized_1000","prompts":["..."],"box":[x1,y1,x2,y2]}。'
        )
        response = self.analyze(
            rgb_image,
            prompt=prompt,
            max_tokens=384,
            temperature=0.1,
            thinking_type="disabled",
        )
        value = self._parse_json_object(response)
        prompts = value.get("prompts", []) if isinstance(value, dict) else []
        if not isinstance(prompts, list):
            prompts = []
        prompts = [str(item).strip() for item in prompts if str(item).strip()]
        if original not in prompts:
            prompts.insert(0, original)

        box = value.get("box") if isinstance(value, dict) else None
        try:
            if not isinstance(box, (list, tuple)) or len(box) != 4:
                raise ValueError
            box = [float(item) for item in box]
            x1, y1, x2, y2 = box
            if not all(map(lambda item: item == item and abs(item) != float("inf"), box)):
                raise ValueError
            if x2 <= x1 or y2 <= y1:
                raise ValueError
            # GLM grounding commonly uses a 0..1000 coordinate frame even
            # when the prompt contains the camera resolution.  Convert it to
            # the actual RGB pixel frame before clipping.
            box_format = value.get("box_format") if isinstance(value, dict) else None
            if box_format == "normalized_1000" or (
                max(box) <= 1000 and (x2 > width or y2 > height)
            ):
                x1, x2 = x1 * width / 1000.0, x2 * width / 1000.0
                y1, y2 = y1 * height / 1000.0, y2 * height / 1000.0
            box = [
                max(0.0, min(float(width), x1)),
                max(0.0, min(float(height), y1)),
                max(0.0, min(float(width), x2)),
                max(0.0, min(float(height), y2)),
            ]
            if box[2] <= box[0] or box[3] <= box[1]:
                raise ValueError
        except (TypeError, ValueError):
            box = None

        return {
            "prompts": prompts[:max_prompts],
            "box": box,
            "found": bool(value.get("found", box is not None)) if isinstance(value, dict) else False,
        }

    def expand_object_prompts(self, rgb_image, object_name, max_prompts=6):
        """Return detector-friendly prompts for a user-described object.

        The VLM is used only for semantic prompt expansion.  It does not
        produce robot poses or authorize motion.  Returning the original
        object name on any failure keeps the existing grasp path available.
        """
        return self.ground_object(rgb_image, object_name, max_prompts)["prompts"]

    @staticmethod
    def _parse_json_object(response):
        """Parse a tolerant JSON-object response from the VLM."""
        if not response:
            return {}
        text = str(response).strip()
        text = re.sub(r"^```(?:json)?\s*|\s*```$", "", text, flags=re.I)
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            match = re.search(r"\{[\s\S]*\}", text)
            if not match:
                return {}
            try:
                value = json.loads(match.group(0))
            except json.JSONDecodeError:
                return {}
        return value if isinstance(value, dict) else {}

    @staticmethod
    def _parse_prompt_list(response):
        """Parse a tolerant JSON-list response from the VLM."""
        if not response:
            return []
        text = str(response).strip()
        text = re.sub(r"^```(?:json)?\s*|\s*```$", "", text, flags=re.I)
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            match = re.search(r"\[[\s\S]*?\]", text)
            if not match:
                return []
            try:
                value = json.loads(match.group(0))
            except json.JSONDecodeError:
                return []
        if not isinstance(value, list):
            return []
        result = []
        seen = set()
        for item in value:
            if not isinstance(item, str):
                continue
            item = " ".join(item.split()).strip(" ,，")
            if item and item.lower() not in seen:
                seen.add(item.lower())
                result.append(item)
        return result

    def analyze(
        self,
        rgb_image,
        prompt=None,
        max_tokens=1024,
        temperature=0.7,
        thinking_type="disabled",
    ):
        """
        Send an RGB image to GLM-4.5V and return the text response.

        Args:
            rgb_image: numpy RGB array (H, W, 3) uint8.
            prompt:    text prompt.  If *None* a default scene-analysis prompt
                       is used.
            max_tokens: maximum tokens in the response.
            temperature: sampling temperature.
            thinking_type: optional GLM thinking mode (``enabled`` or
                ``disabled``).  Defaults to ``disabled`` so answers go into
                ``content`` (reasoning otherwise consumes the token budget).

        Returns:
            str or None: the model's text response, or None on error.
        """
        if prompt is None:
            prompt = (
                "请分析图片中的物品及其空间关系，按以下格式回答：\n\n"
                "【物品列表】\n"
                "1. 物品名称 - 位于图片的(左上/右上/左下/右下/中间)位置\n\n"
                "【空间关系】\n"
                "是否有物品被某些容器装着？如果有，请列出，"
                '如"粉红色桃子 在 粉色盘子 里面"\n'
                "...\n\n"
                '备注：你看到的"红色水果"是粉红色桃子'
            )

        if not self.api_token:
            cprint(
                f"[VLM] Missing API token. Set ${self.ENV_TOKEN} env var "
                f"or pass api_token=... to VLMClient().",
                "red",
            )
            return None

        try:
            pil_image = Image.fromarray(rgb_image)

            # Resize if too large (keep aspect ratio)
            max_size = 1024
            if max(pil_image.size) > max_size:
                ratio = max_size / max(pil_image.size)
                new_size = (
                    int(pil_image.size[0] * ratio),
                    int(pil_image.size[1] * ratio),
                )
                pil_image = pil_image.resize(new_size, Image.LANCZOS)

            # JPEG compress
            buffer = io.BytesIO()
            pil_image.save(buffer, format="JPEG", quality=85)
            image_base64 = base64.b64encode(buffer.getvalue()).decode("utf-8")

            cprint(
                f"[VLM] Calling {self.model} (image {pil_image.size})...",
                "cyan",
            )

            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_token}",
            }

            payload = {
                "model": self.model,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_base64}"
                                },
                            },
                            {"type": "text", "text": prompt},
                        ],
                    }
                ],
                "max_tokens": max_tokens,
                "temperature": temperature,
            }
            if thinking_type is not None:
                payload["thinking"] = {"type": thinking_type}

            response = requests.post(
                self.api_url, headers=headers, json=payload, timeout=60
            )

            if response.status_code == 200:
                result = response.json()
                msg = result.get("choices", [{}])[0].get("message", {})
                content = msg.get("content", "")
                # Fallback: if the model returned reasoning but empty
                # content (thinking mode on), use the reasoning text.
                if not content:
                    content = msg.get("reasoning_content", "")
                return content
            else:
                cprint(
                    f"[VLM] API error {response.status_code}: {response.text}",
                    "red",
                )
                return None

        except Exception as e:
            cprint(f"[VLM] Error: {e}", "red")
            return None
