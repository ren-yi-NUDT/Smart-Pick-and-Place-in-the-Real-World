#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GLM-4.5V Vision-Language Model client.

Extracted from the duplicated code in look_around.py and capture_at_handover.py.
"""

import io
import os
import base64
import requests
from PIL import Image
from termcolor import cprint


class VLMClient:
    """Thin wrapper around the GLM-4.5V chat-completion API."""

    DEFAULT_API_URL = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
    DEFAULT_MODEL = "glm-4.5v"
    ENV_TOKEN = "GLM_API_TOKEN"

    def __init__(self, api_url=None, api_token=None, model=None):
        self.api_url = api_url or self.DEFAULT_API_URL
        self.api_token = api_token or os.environ.get(self.ENV_TOKEN)
        self.model = model or self.DEFAULT_MODEL

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def analyze(self, rgb_image, prompt=None, max_tokens=1024, temperature=0.7):
        """
        Send an RGB image to GLM-4.5V and return the text response.

        Args:
            rgb_image: numpy RGB array (H, W, 3) uint8.
            prompt:    text prompt.  If *None* a default scene-analysis prompt
                       is used.
            max_tokens: maximum tokens in the response.
            temperature: sampling temperature.

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

            response = requests.post(
                self.api_url, headers=headers, json=payload, timeout=60
            )

            if response.status_code == 200:
                result = response.json()
                content = (
                    result.get("choices", [{}])[0]
                    .get("message", {})
                    .get("content", "")
                )
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
