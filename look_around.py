#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂环顾拍照模块
===================
遍历预设观测位置，在每个位置拍摄RGB-D图像并保存。

使用方式:
    # 命令行直接运行
    python3 look_around.py

    # 作为模块导入
    from look_around import LookAround
    looker = LookAround()
    images = looker.scan_all_positions()  # 返回 {位置名: (rgb, depth)} 字典
"""

import os
import sys
import json
import socket
import time
import struct
import base64
import requests
import threading
from termcolor import cprint
from camera import RealSenseCapture
from armcontroller import ArmController


class LookAround:
    """机械臂环顾拍照类"""

    # 默认配置
    DEFAULT_ROBOT_CONFIG = "./robot_config.json"
    DEFAULT_SAVE_PATH = "./log/look_around"
    ARM_HOST = "127.0.0.1"
    ARM_PORT = 8010

    # GLM-4.5V API 配置
    GLM_API_URL = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
    GLM_API_TOKEN = "b8b434c4bc27407e83b76a5bec46fa12.UJWcvdOgEnGb4cBJ"
    GLM_MODEL = "glm-4.5v"

    def __init__(self, robot_config_path=None, save_path=None):
        """
        初始化环顾拍照模块

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
        self.default_traj_js = self.robot_config["default_traj_js"]

        # 提取观测位置（包含 "grasp" 关键词的位置）
        self.observation_positions = {
            k: v for k, v in self.default_traj_js.items()
            if "grasp" in k
        }
        cprint(f"[LookAround] 发现 {len(self.observation_positions)} 个观测位置: {list(self.observation_positions.keys())}", "cyan")

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
            cprint(f"[LookAround] 已连接机械臂服务 {self.ARM_HOST}:{self.ARM_PORT}", "green")
            return True
        except Exception as e:
            cprint(f"[LookAround] 连接机械臂服务失败: {e}", "red")
            return False

    def disconnect_arm(self):
        """断开机械臂连接（已禁用）"""
        # 已禁用，保持连接
        # if self.arm_client:
        #     self.arm_client.close()
        #     self.arm_client = None
        #     cprint("[LookAround] 已断开机械臂连接", "yellow")
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
            prompt = '''请分析图片中的物品及其空间关系，按以下格式回答：

【物品列表】
1. 物品名称 - 位于图片的(左上/右上/左下/右下/中间)位置

【空间关系】
是否有物品被某些容器装着？如果有，请列出，如“粉红色桃子 在 粉色盘子 里面”
...

备注：你看到的”红色水果”是粉红色桃子'''

        try:
            from PIL import Image
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

            cprint(f"[LookAround] 正在调用 GLM-4.5V 分析图片 (尺寸: {pil_image.size})...", "cyan")

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
                "max_tokens": 1024,
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
                cprint(f"[LookAround] GLM API 调用失败: {response.status_code} - {response.text}", "red")
                return None

        except Exception as e:
            cprint(f"[LookAround] GLM-4.5V 分析出错: {e}", "red")
            return None

    def move_to_position(self, position_name, speed=30):
        """
        移动到指定预设位置

        Args:
            position_name: 位置名称（如 "grasp1", "grasp2"）
            speed: 移动速度

        Returns:
            bool: 是否成功
        """
        if position_name not in self.observation_positions:
            cprint(f"[LookAround] 未知位置: {position_name}", "red")
            return False

        if not self.arm_client:
            cprint("[LookAround] 机械臂未连接", "red")
            return False

        try:
            self.arm_controller.start_cmd()
            self.arm_controller.add_js_cmd(
                self.observation_positions[position_name],
                speed=speed,
                block=True
            )
            self.arm_controller.send_cmds(self.arm_client)
            self.arm_controller.reset_cmd()
            cprint(f"[LookAround] 已移动到位置: {position_name}", "green")
            return True
        except Exception as e:
            cprint(f"[LookAround] 移动失败: {e}", "red")
            return False

    def capture_image(self, position_name):
        """
        在当前位置拍摄图像

        Args:
            position_name: 位置名称（用于保存文件名）

        Returns:
            tuple: (rgb, depth) 或 (None, None)
        """
        try:
            # 等待机械臂稳定
            time.sleep(0.3)

            # 拍摄图像
            rgb, depth = self.cam.get_rgbd()

            # 保存带位置标记的图像
            from PIL import Image
            from datetime import datetime

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            rgb_path = os.path.join(self.save_path, f"{position_name}_rgb_{timestamp}.png")
            depth_path = os.path.join(self.save_path, f"{position_name}_depth_{timestamp}.png")

            Image.fromarray(rgb).save(rgb_path)
            Image.fromarray(depth).save(depth_path)

            cprint(f"[LookAround] 已保存图像: {rgb_path}", "cyan")

            return rgb, depth, rgb_path, depth_path

        except Exception as e:
            cprint(f"[LookAround] 拍照失败: {e}", "red")
            return None, None, None, None

    def scan_single_position(self, position_name, speed=30):
        """
        移动到指定位置并拍照

        Args:
            position_name: 位置名称
            speed: 移动速度

        Returns:
            dict: {"rgb": ..., "depth": ..., "rgb_path": ..., "depth_path": ...}
        """
        result = {
            "position": position_name,
            "rgb": None,
            "depth": None,
            "rgb_path": None,
            "depth_path": None,
            "success": False
        }

        if self.move_to_position(position_name, speed):
            rgb, depth, rgb_path, depth_path = self.capture_image(position_name)
            result["rgb"] = rgb
            result["depth"] = depth
            result["rgb_path"] = rgb_path
            result["depth_path"] = depth_path
            result["success"] = rgb is not None

        return result

    def _background_glm_analysis(self, rgb_image, result_container):
        """
        后台线程中进行 GLM-4.5V 分析

        Args:
            rgb_image: RGB 图像
            result_container: 用于存储结果的字典（会被修改）
        """
        try:
            glm_analysis = self.analyze_image_with_glm(rgb_image)
            result_container["glm_analysis"] = glm_analysis
            result_container["analysis_done"] = True
            cprint("[LookAround] 后台 GLM-4.5V 分析完成", "green")
        except Exception as e:
            cprint(f"[LookAround] 后台 GLM 分析出错: {e}", "red")
            result_container["glm_analysis"] = None
            result_container["analysis_done"] = True

    def scan_all_positions(self, speed=30, return_to_first=True):
        """
        在grasp1拍照后启动后台线程调用GLM-4.5V分析图片，
        同时机械臂前往grasp2、3假装拍照（间隔3秒），实现并行处理。

        Args:
            speed: 移动速度
            return_to_first: 完成后是否返回第一个位置

        Returns:
            dict: {位置名: {"rgb": ..., "depth": ..., "rgb_path": ..., "depth_path": ..., "glm_analysis": ...}}
        """
        results = {}
        position_names = list(self.observation_positions.keys())

        if not position_names:
            cprint("[LookAround] 没有可用的观测位置", "red")
            return results

        # 连接机械臂
        if not self.connect_arm():
            return results

        # 用于存储后台 GLM 分析结果
        glm_result_container = {"glm_analysis": None, "analysis_done": False}
        analysis_thread = None

        try:
            cprint(f"[LookAround] 开始环顾扫描，共 {len(position_names)} 个位置", "cyan")
            start_time = time.time()

            # 第一步：在grasp1拍照，然后启动后台线程进行GLM分析
            grasp1_result = None
            for i, pos_name in enumerate(position_names):
                if pos_name == "grasp1":
                    cprint(f"\n[LookAround] === 位置 {i+1}/{len(position_names)}: {pos_name} (拍照) ===", "yellow")

                    if not self.move_to_position(pos_name, speed):
                        results[pos_name] = {"position": pos_name, "success": False}
                        continue

                    rgb, depth, rgb_path, depth_path = self.capture_image(pos_name)

                    # 启动后台线程进行 GLM-4.5V 分析
                    if rgb is not None:
                        cprint("[LookAround] 启动后台线程进行 GLM-4.5V 分析...", "cyan")
                        analysis_thread = threading.Thread(
                            target=self._background_glm_analysis,
                            args=(rgb.copy(), glm_result_container),
                            daemon=True
                        )
                        analysis_thread.start()

                    # 假装拍照，等待3秒（与其他位置一致）
                    cprint(f"[LookAround] 假装拍照中，等待3秒...", "cyan")
                    time.sleep(3)

                    grasp1_result = {
                        "position": pos_name,
                        "rgb": rgb,
                        "depth": depth,
                        "rgb_path": rgb_path,
                        "depth_path": depth_path,
                        "glm_analysis": None,  # 稍后由后台线程填充
                        "success": rgb is not None
                    }
                    results[pos_name] = grasp1_result
                    break

            # 第二步：前往其他位置假装拍照（并行：此时 GLM 分析在后台进行）
            for i, pos_name in enumerate(position_names):
                if pos_name == "grasp1":
                    continue

                # 检查 GLM 分析是否完成
                if glm_result_container["analysis_done"]:
                    status = "GLM分析已完成"
                else:
                    status = "GLM分析进行中..."
                cprint(f"\n[LookAround] === 位置 {i+1}/{len(position_names)}: {pos_name} (假装拍照) [{status}] ===", "yellow")

                if not self.move_to_position(pos_name, speed):
                    results[pos_name] = {"position": pos_name, "success": False}
                    continue

                # 假装拍照，等待3秒
                cprint(f"[LookAround] 假装拍照中，等待3秒...", "cyan")
                time.sleep(3)

                results[pos_name] = {
                    "position": pos_name,
                    "rgb": None,
                    "depth": None,
                    "rgb_path": None,
                    "depth_path": None,
                    "glm_analysis": None,
                    "success": True
                }

            # 第三步：等待后台 GLM 分析完成
            if analysis_thread is not None and analysis_thread.is_alive():
                cprint("\n[LookAround] 等待后台 GLM-4.5V 分析完成...", "cyan")
                analysis_thread.join(timeout=60)  # 最多等待60秒

            # 将 GLM 分析结果填充到 grasp1_result
            if grasp1_result is not None:
                grasp1_result["glm_analysis"] = glm_result_container["glm_analysis"]

            # 返回第一个位置
            if return_to_first and position_names:
                self.move_to_position(position_names[0], speed)

            elapsed = time.time() - start_time
            cprint(f"\n[LookAround] 环顾扫描完成，耗时 {elapsed:.1f} 秒", "green")

            # 在末尾打印 GLM-4.5V 分析结果
            if grasp1_result and grasp1_result.get("glm_analysis"):
                cprint(f"\n========== GLM-4.5V 分析结果 ==========", "yellow")
                cprint(grasp1_result["glm_analysis"], "green")

        finally:
            # 保持连接，不主动断开
            pass

        return results

    def get_latest_images(self):
        """
        获取最新拍摄的图像路径

        Returns:
            dict: {位置名: {"rgb_path": ..., "depth_path": ...}}
        """
        latest_images = {}

        # 遍历保存目录，找到每个位置的最新图像
        for pos_name in self.observation_positions.keys():
            rgb_files = sorted([
                f for f in os.listdir(self.save_path)
                if f.startswith(f"{pos_name}_rgb_") and f.endswith(".png")
            ], reverse=True)

            depth_files = sorted([
                f for f in os.listdir(self.save_path)
                if f.startswith(f"{pos_name}_depth_") and f.endswith(".png")
            ], reverse=True)

            if rgb_files and depth_files:
                latest_images[pos_name] = {
                    "rgb_path": os.path.join(self.save_path, rgb_files[0]),
                    "depth_path": os.path.join(self.save_path, depth_files[0])
                }

        return latest_images


def main():
    """命令行入口"""
    import argparse

    parser = argparse.ArgumentParser(description="机械臂环顾拍照")
    parser.add_argument("--config", default="./robot_config.json", help="机器人配置文件路径")
    parser.add_argument("--save-path", default="./log/look_around", help="图片保存路径")
    parser.add_argument("--speed", type=int, default=30, help="移动速度")
    parser.add_argument("--position", type=str, default=None, help="只扫描指定位置（如 grasp1）")
    parser.add_argument("--no-return", action="store_true", help="完成后不返回起始位置")

    args = parser.parse_args()

    # 创建环顾实例
    looker = LookAround(
        robot_config_path=args.config,
        save_path=args.save_path
    )

    # 执行扫描
    if args.position:
        # 单个位置
        result = looker.scan_single_position(args.position, speed=args.speed)
        if result["success"]:
            print(f"\n图像已保存:")
            print(f"  RGB:   {result['rgb_path']}")
            print(f"  Depth: {result['depth_path']}")
        else:
            print("扫描失败")
            sys.exit(1)
    else:
        # 所有位置
        results = looker.scan_all_positions(
            speed=args.speed,
            return_to_first=not args.no_return
        )

        #print(f"\n========== 扫描结果 ==========")
        #for pos_name, result in results.items():
            #status = "成功" if result["success"] else "失败"
            #print(f"  {pos_name}: {status}")
            #if result["success"]:
                #print(f"    RGB:   {result['rgb_path']}")
                #print(f"    Depth: {result['depth_path']}")


if __name__ == "__main__":
    main()
