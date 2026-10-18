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

from core.abc import BaseVLM


class VLMClient(BaseVLM):
    """Thin wrapper around the GLM-4.5V chat-completion API."""

    # 交接门用的 prompt: 必须允许"无", 否则模型会顺着预设编一个物品出来。
    # 强制一行输出 + temperature=0 —— 这是分类不是创作, 带随机性会翻。
    HAND_PROMPT = (
        "画面里有没有一只手正拿着物品？"
        "拿着的就回答「有:物品名」，没拿着就回答「无」。"
        "只回答这一行, 不要描述画面、不要猜测、不确定就回答「无」。"
    )

    DEFAULT_API_URL = "https://ark.cn-beijing.volces.com/api/plan/v3"
    DEFAULT_MODEL = "ark-code-latest"
    # 密钥只从环境变量读 (GLM_API_TOKEN=... 在 ~/.bashrc, 不进 git)
    DEFAULT_TOKEN = os.environ.get("GLM_API_TOKEN", "")

    def __init__(self, api_url=None, api_token=None, model=None, use_env_proxy=False):
        # 允许传基地址 (…/v1) 或完整端点 (…/v1/chat/completions), 两种都补全
        url = (api_url or self.DEFAULT_API_URL).rstrip("/")
        self.api_url = url if url.endswith("/chat/completions") else url + "/chat/completions"
        self.api_token = api_token or self.DEFAULT_TOKEN
        self.model = model or self.DEFAULT_MODEL
        # 默认直连: 国内中转端点经系统代理会 SSL EOF (RemoteDisconnected)。
        # 需要走代理时传 use_env_proxy=True。
        self._session = requests.Session()
        self._session.trust_env = use_env_proxy

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def detect_hand_object(self, rgb_image):
        """
        判断画面里的手有没有拿着物品 —— 交接流程的"空手就跳过"门。

        Returns:
            (True,  物品名)  手里有东西
            (False, None)    手是空的 / 没看到手
            (None,  None)    调用失败 —— 调用方自己决定, 别当成空手
        """
        # max_tokens 要给够: 实测 1024 时思维链被截断, 会答出错误的"无"
        ans = self.analyze(rgb_image, prompt=self.HAND_PROMPT,
                           max_tokens=8192, temperature=0.0)
        if ans is None:
            return None, None
        ans = ans.strip()
        if not ans.startswith("有"):
            return False, None
        name = ans.split(":", 1)[-1].split("：", 1)[-1].strip()
        return True, (name or None)

    def analyze(self, rgb_image, prompt=None, max_tokens=8192, temperature=0.7):
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

            response = self._session.post(
                self.api_url, headers=headers, json=payload, timeout=60
            )

            if response.status_code == 200:
                result = response.json()
                choice = result.get("choices", [{}])[0]
                content = (choice.get("message", {}) or {}).get("content")
                if not content:
                    # glm-5.3-flash 是推理模型: 思维链占满 max_tokens 时
                    # content 为 null 且 finish_reason=length。别静默返回 None,
                    # 否则上层会把"截断"当成"模型说没有"。
                    cprint(
                        f"[VLM] 空回复 (finish_reason={choice.get('finish_reason')}, "
                        f"max_tokens={max_tokens}) — 推理模型需要更大的 max_tokens",
                        "red",
                    )
                    return None
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
