#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂查看用户手中物品拍照模块
===================
移动到look_over_what_in_user_hand_pose位置，拍摄RGB-D图像并保存。

使用方式:
    # 命令行直接运行
    python3 capture_at_handover.py

    # 作为模块导入
    from capture_at_handover import CaptureAtHandover
    capturer = CaptureAtHandover()
    rgb, depth, rgb_path, depth_path = capturer.capture()
"""

import os
import sys
import json
import socket
import time
import base64
import requests
import numpy as np
from termcolor import cprint
from camera import RealSenseCapture
from armcontroller import ArmController
from datetime import datetime
from PIL import Image


class CaptureAtHandover:
    """机械臂查看用户手中物品拍照类"""

    # 默认配置
    DEFAULT_ROBOT_CONFIG = "./robot_config.json"
    DEFAULT_SAVE_PATH = "./log/handover_capture"
    ARM_HOST = "127.0.0.1"
    ARM_PORT = 8010

    # GLM-4.5V API 配置
    GLM_API_URL = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
    GLM_API_TOKEN = "b8b434c4bc27407e83b76a5bec46fa12.UJWcvdOgEnGb4cBJ"
    GLM_MODEL = "glm-4.5v"

    def __init__(self, robot_config_path=None, save_path=None):
        """
        初始化拍照模块

        Args:
            robot_config_path: 机器人配置文件路径
            save_path: 图片保存路径
        """
        # 配置路径
        self.robot_config_path = robot_config_path or self.DEFAULT_ROBOT_CONFIG
        self.save_path = save_path or self.DEFAULT_SAVE_PATH

        # 确保保存目录存在
        os.makedirs(self.save_path, exist_ok=True)

        # 加载机器人配置
        self.robot_config = json.load(open(self.robot_config_path, "r"))
        self.look_pose = self.robot_config.get("look_over_what_in_user_hand_pose")

        if self.look_pose is None:
            raise ValueError("robot_config.json 中未定义 look_over_what_in_user_hand_pose")

        cprint(f"[CaptureAtHandover] 已加载 look_over_what_in_user_hand_pose", "cyan")

        # 初始化相机
        self.cam = RealSenseCapture(width=640, height=480, fps=30, save_path=self.save_path)

        # 初始化机械臂控制器
        self.arm_controller = ArmController()
        self.arm_client = None

    def connect_arm(self):
        """连接机械臂控制服务"""
        try:
            self.arm_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.arm_client.connect((self.ARM_HOST, self.ARM_PORT))
            cprint(f"[CaptureAtHandover] 已连接机械臂服务 {self.ARM_HOST}:{self.ARM_PORT}", "green")
            return True
        except Exception as e:
            cprint(f"[CaptureAtHandover] 连接机械臂服务失败: {e}", "red")
            return False

    def disconnect_arm(self):
        """断开机械臂连接（已禁用）"""
        # 已禁用，保持连接
        # if self.arm_client:
        #     self.arm_client.close()
        #     self.arm_client = None
        #     cprint("[CaptureAtHandover] 已断开机械臂连接", "yellow")
        pass

    def analyze_image_with_glm(self, rgb_image, prompt=None):
        """
        使用 GLM-4.5V 分析图片

        Args:
            rgb_image: RGB 图像 (numpy array)
            prompt: 自定义提示词

        Returns:
            str: 模型返回的分析结果
        """
        if prompt is None:
            prompt = "画面中有一只手，告诉我手里拿的是什么。请简洁回答，只说物品名称。"

        try:
            import io

            # 将图片转为 PIL Image
            pil_image = Image.fromarray(rgb_image)

            # 缩放图片到合适大小（保持宽高比）
            max_size = 1024
            if max(pil_image.size) > max_size:
                ratio = max_size / max(pil_image.size)
                new_size = (int(pil_image.size[0] * ratio), int(pil_image.size[1] * ratio))
                pil_image = pil_image.resize(new_size, Image.LANCZOS)

            # 使用 JPEG 格式压缩（质量85%）
            buffer = io.BytesIO()
            pil_image.save(buffer, format="JPEG", quality=85)
            image_base64 = base64.b64encode(buffer.getvalue()).decode("utf-8")

            cprint(f"[CaptureAtHandover] 正在调用 GLM-4.5V 分析图片 (尺寸: {pil_image.size})...", "cyan")

            # 智谱 API 格式 (OpenAI 兼容)
            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.GLM_API_TOKEN}"
            }

            payload = {
                "model": self.GLM_MODEL,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_base64}"
                                }
                            },
                            {
                                "type": "text",
                                "text": prompt
                            }
                        ]
                    }
                ],
                "max_tokens": 256,
                "temperature": 0.7
            }

            response = requests.post(
                self.GLM_API_URL,
                headers=headers,
                json=payload,
                timeout=60
            )

            if response.status_code == 200:
                result = response.json()
                content = result.get('choices', [{}])[0].get('message', {}).get('content', '')
                return content
            else:
                cprint(f"[CaptureAtHandover] GLM API 调用失败: {response.status_code} - {response.text}", "red")
                return None

        except Exception as e:
            cprint(f"[CaptureAtHandover] GLM 分析出错: {e}", "red")
            return None

    def move_to_look_pose(self, speed=15):
        """
        移动到look_over_what_in_user_hand_pose位置

        Args:
            speed: 移动速度

        Returns:
            bool: 是否成功
        """
        if not self.arm_client:
            cprint("[CaptureAtHandover] 机械臂未连接", "red")
            return False

        try:
            # 转换为轨迹格式
            joint_angles = [
                self.look_pose["J1"],
                self.look_pose["J2"],
                self.look_pose["J3"],
                self.look_pose["J4"],
                self.look_pose["J5"],
                self.look_pose["J6"],
                self.look_pose["J7"]
            ]

            self.arm_controller.start_cmd()
            # 使用trajectory模式发送单个位置
            self.arm_controller.add_js_cmd(
                {
                    'J1': joint_angles[0], 'J2': joint_angles[1], 'J3': joint_angles[2],
                    'J4': joint_angles[3], 'J5': joint_angles[4], 'J6': joint_angles[5], 'J7': joint_angles[6]
                },
                speed=speed,
                block=True
            )
            self.arm_controller.send_cmds(self.arm_client)
            self.arm_controller.reset_cmd()

            cprint("[CaptureAtHandover] 已移动到 look_over_what_in_user_hand_pose", "green")
            return True
        except Exception as e:
            cprint(f"[CaptureAtHandover] 移动失败: {e}", "red")
            return False

    def capture_image(self, save_with_timestamp=True):
        """
        在当前位置拍摄图像

        Args:
            save_with_timestamp: 是否在文件名中包含时间戳

        Returns:
            tuple: (rgb, depth, rgb_path, depth_path)
        """
        try:
            # 等待机械臂稳定
            time.sleep(0.5)

            # 拍摄图像
            rgb, depth = self.cam.get_rgbd()

            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            if save_with_timestamp:
                rgb_filename = f"handover_rgb_{timestamp}.png"
                depth_filename = f"handover_depth_{timestamp}.png"
            else:
                rgb_filename = "handover_rgb.png"
                depth_filename = "handover_depth.png"

            rgb_path = os.path.join(self.save_path, rgb_filename)
            depth_path = os.path.join(self.save_path, depth_filename)

            # 保存图像
            Image.fromarray(rgb).save(rgb_path)
            Image.fromarray(depth).save(depth_path)

            cprint(f"[CaptureAtHandover] 已保存图像:", "green")
            cprint(f"  RGB:   {rgb_path}", "cyan")
            cprint(f"  Depth: {depth_path}", "cyan")

            return rgb, depth, rgb_path, depth_path

        except Exception as e:
            cprint(f"[CaptureAtHandover] 拍照失败: {e}", "red")
            return None, None, None, None

    def capture(self, speed=15, return_to_safe=True, analyze=True):
        """
        移动到look_over_what_in_user_hand_pose并拍照，可选调用GLM-4.5V分析

        Args:
            speed: 移动速度
            return_to_safe: 完成后是否返回安全位置 (grasp1)
            analyze: 是否调用GLM-4.5V分析手中物品

        Returns:
            tuple: (rgb, depth, rgb_path, depth_path, glm_analysis)
        """
        rgb, depth, rgb_path, depth_path, glm_analysis = None, None, None, None, None

        # 连接机械臂
        if not self.connect_arm():
            return rgb, depth, rgb_path, depth_path, glm_analysis

        try:
            # 移动到观察位置
            if not self.move_to_look_pose(speed):
                return rgb, depth, rgb_path, depth_path, glm_analysis

            # 拍照
            rgb, depth, rgb_path, depth_path = self.capture_image()

            # 调用 GLM-4.5V 分析手中物品
            if analyze and rgb is not None:
                glm_analysis = self.analyze_image_with_glm(rgb)
                if glm_analysis:
                    cprint(f"\n[CaptureAtHandover] GLM-4.5V 识别结果: {glm_analysis}\n", "green")

            # 返回安全位置
            if return_to_safe:
                self.return_to_safe_position()

        finally:
            # 保持连接，不主动断开
            pass

        return rgb, depth, rgb_path, depth_path, glm_analysis

    def return_to_safe_position(self, speed=30):
        """返回安全位置 (grasp1)"""
        if not self.arm_client:
            return False

        try:
            default_traj_js = self.robot_config.get("default_traj_js", {})
            grasp1 = default_traj_js.get("grasp1")

            if grasp1 is None:
                cprint("[CaptureAtHandover] 未找到 grasp1 位置", "yellow")
                return False

            self.arm_controller.start_cmd()
            self.arm_controller.add_js_cmd(grasp1, speed=speed, block=True)
            self.arm_controller.send_cmds(self.arm_client)
            self.arm_controller.reset_cmd()

            cprint("[CaptureAtHandover] 已返回安全位置", "green")
            return True
        except Exception as e:
            cprint(f"[CaptureAtHandover] 返回失败: {e}", "red")
            return False

    def get_latest_image(self):
        """
        获取最新拍摄的图像路径

        Returns:
            dict: {"rgb_path": ..., "depth_path": ...} 或 None
        """
        rgb_files = sorted([
            f for f in os.listdir(self.save_path)
            if f.startswith("handover_rgb_") and f.endswith(".png")
        ], reverse=True)

        depth_files = sorted([
            f for f in os.listdir(self.save_path)
            if f.startswith("handover_depth_") and f.endswith(".png")
        ], reverse=True)

        if rgb_files and depth_files:
            return {
                "rgb_path": os.path.join(self.save_path, rgb_files[0]),
                "depth_path": os.path.join(self.save_path, depth_files[0])
            }

        return None


def main():
    """命令行入口"""
    import argparse

    parser = argparse.ArgumentParser(description="机械臂在handover位置拍照")
    parser.add_argument("--config", default="./robot_config.json", help="机器人配置文件路径")
    parser.add_argument("--save-path", default="./log/handover_capture", help="图片保存路径")
    parser.add_argument("--speed", type=int, default=15, help="移动速度")
    parser.add_argument("--no-return", action="store_true", help="完成后不返回安全位置")
    parser.add_argument("--no-analyze", action="store_true", help="不调用GLM-4.5V分析")

    args = parser.parse_args()

    # 创建拍照实例
    capturer = CaptureAtHandover(
        robot_config_path=args.config,
        save_path=args.save_path
    )

    # 执行拍照
    rgb, depth, rgb_path, depth_path, glm_analysis = capturer.capture(
        speed=args.speed,
        return_to_safe=not args.no_return,
        analyze=not args.no_analyze
    )

    if rgb is not None:
        print(f"\n========== 拍照成功 ==========")
        print(f"  RGB:   {rgb_path}")
        print(f"  Depth: {depth_path}")
        if glm_analysis:
            print(f"\n========== GLM-4.5V 识别结果 ==========")
            print(f"  手中物品: {glm_analysis}")
    else:
        print("拍照失败")
        sys.exit(1)


if __name__ == "__main__":
    main()
