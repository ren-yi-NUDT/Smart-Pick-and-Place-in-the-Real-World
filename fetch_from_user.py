import os, sys
curr_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(curr_dir)
import struct
import json, copy, rospy
import socket, time
import numpy as np
import matplotlib.pyplot as plt
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from armcontroller import ArmController
from datetime import datetime
from transformation import TransformationUtil
from camera import RealSenseCapture
from json_input import JsonInputParser
from termcolor import cprint
from PIL import Image
from ultralytics import YOLOWorld
from scipy.spatial.transform import Rotation as R
from utils import pixel_to_camera_point2


class PlannerFetchFromUser:
    """
    从用户手中接收物品并放置到指定容器的规划器

    流程:
    1. 读取JSON输入 (container字段)
    2. 移动到handover_pose接收位置
    3. 张开手等待用户放入物品
    4. 拍照识别用户手中的物品 (可选)
    5. 固定延时等待用户放置
    6. 关闭手并检测是否成功抓取物品
    7. 移动到容器位置并放置
    """

    def __init__(self, yolo_model_path="", robot_config_path="", save_path=""):
        self.save_path = save_path
        self.cam = RealSenseCapture(width=640, height=480, fps=30, save_path=save_path)
        self.json_parser = JsonInputParser()
        self.yolo_model = YOLOWorld(yolo_model_path)
        self.transform = TransformationUtil()
        self.arm_controller = ArmController()

        # Socket connections
        HOST = '127.0.0.1'
        HAND_PORT = 8000
        self.hand_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.hand_client.connect((HOST, HAND_PORT))

        ARM_PORT = 8010
        self.arm_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.arm_client.connect((HOST, ARM_PORT))

        TWIN_PORT = 8020
        self.twin_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.twin_client.connect((HOST, TWIN_PORT))

        self.tf_broadcaster = StaticTransformBroadcaster()

        # Load robot config
        self.robot_config = json.load(open(robot_config_path, "r"))
        self.default_traj_js = self.robot_config["default_traj_js"]
        self.default_traj_js_rad = [data / 180 * np.pi for data in self.default_traj_js["grasp1"].values()]

        # Hand configuration
        self.hand_config = {
            "close": [0, 0, 0, 460, 0, 0],
            "open": [1000, 1000, 1000, 1000, 1000, 0]
        }

        # Coordinate transforms
        self.T_base_to_cam = None
        self.T_hand_effector_to_arm_endlink = None

        # Detected object name
        self.detected_object = None

    def send_cmd(self, sock, data):
        """发送命令到socket并接收响应"""
        msg = json.dumps(data).encode('utf-8')
        sock.sendall(msg)
        resp = json.loads(sock.recv(1024).decode('utf-8'))
        cprint(f"Response: {resp}", "red")
        return resp

    def send_cmd_twin(self, sock, data):
        """发送命令到Twin服务 (带4字节长度前缀)"""
        msg = json.dumps(data).encode('utf-8')
        sock.sendall(msg)

        # 读取4字节长度前缀
        length_bytes = b''
        while len(length_bytes) < 4:
            chunk = sock.recv(4 - len(length_bytes))
            if not chunk:
                raise ConnectionError("Connection closed")
            length_bytes += chunk

        data_length = struct.unpack('>I', length_bytes)[0]

        # 读取数据
        data_bytes = b''
        while len(data_bytes) < data_length:
            chunk = sock.recv(min(4096, data_length - len(data_bytes)))
            if not chunk:
                raise ConnectionError("Connection closed")
            data_bytes += chunk

        resp = json.loads(data_bytes.decode('utf-8'))
        cprint(f"Twin response: {resp}", "red")
        return resp

    def control_hand(self, cmd_type="close"):
        """控制灵巧手"""
        if cmd_type == "close":
            cmd = {"src": "/left_hand/movement_control", "type": "set", "cmd": self.hand_config["close"]}
            self.send_cmd(self.hand_client, cmd)
        elif cmd_type == "open":
            cmd = {"src": "/left_hand/movement_control", "type": "set", "cmd": self.hand_config["open"]}
            self.send_cmd(self.hand_client, cmd)
        elif cmd_type == "get_state":
            cmd = {"src": "/left_hand/movement_control", "type": "get"}
            resp = self.send_cmd(self.hand_client, cmd)
            return resp

    def check_grasping_object(self):
        """
        检测手里是否成功抓取物品
        原理：关闭手后获取手指位置，与完全关闭状态比较
        如果差异 > 20，说明有物品挡住（手没完全合上）
        """
        time.sleep(0.7)
        self.control_hand(cmd_type="close")
        value = self.control_hand(cmd_type="get_state")["value"]
        diff = np.array(value) - np.array(list(self.hand_config["close"]))
        if abs(diff.sum()) > 20:
            return True
        else:
            return False

    def get_camera_obs(self):
        """获取相机观测"""
        rgb, depth = self.cam.get_rgbd()
        return rgb, depth

    def get_json_input(self):
        """从stdin读取JSON命令"""
        return self.json_parser.get_command()

    def parse_input(self, json_data):
        """解析JSON数据，提取容器信息"""
        if json_data is None:
            return None
        container = json_data.get('container')
        return container

    def save_current_transformation(self):
        """保存当前坐标变换"""
        transform_from_frame = self.robot_config["base_link_name"]
        transform_to_frame = self.robot_config["camera_link_name"]
        self.T_base_to_cam, _, _ = self.transform.get_transform_from_frame_to_frame(
            transform_from_frame, transform_to_frame
        )
        grasping_from_frame = self.robot_config["hand_effector_name"]
        grasping_to_frame = self.robot_config["arm_end_link_name"]
        self.T_hand_effector_to_arm_endlink, _, _ = self.transform.get_transform_from_frame_to_frame(
            grasping_from_frame, grasping_to_frame
        )

    def detect_object_in_hand(self, image, conf=0.25):
        """
        使用YOLO-World检测用户手中的物品
        返回检测到的物品类别名称
        """
        # 使用通用的物品类别进行检测
        common_objects = ["object", "item", "fruit", "food", "bottle", "cup", "phone", "box",
                          "apple", "orange", "banana", "lemon", "pear", "carrot"]
        self.yolo_model.set_classes(common_objects)
        results = self.yolo_model.predict(source=Image.fromarray(image), conf=conf)

        if len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            detections = results[0].boxes.data.tolist()
            if detections:
                # 获取置信度最高的检测结果
                best_det = max(detections, key=lambda x: x[4])
                class_id = int(best_det[5])
                class_name = results[0].names[class_id]
                cprint(f"Detected object: {class_name} (confidence: {best_det[4]:.2f})", "green")
                return class_name

        cprint("No object detected in user's hand", "yellow")
        return None

    def control_arm(self, pose_type=None, trajectory=None, speed=20):
        """控制机械臂"""
        try:
            self.arm_controller.start_cmd()
            if pose_type is not None:
                self.arm_controller.add_js_cmd(self.default_traj_js[pose_type], speed=speed, block=True)
            elif trajectory is not None:
                for i in range(len(trajectory)):
                    self.arm_controller.add_js_cmd({
                        'J1': trajectory[i][0], 'J2': trajectory[i][1], 'J3': trajectory[i][2],
                        'J4': trajectory[i][3], 'J5': trajectory[i][4], 'J6': trajectory[i][5], 'J7': trajectory[i][6]
                    }, speed=speed, block=True)
            self.arm_controller.send_cmds(self.arm_client)
            self.arm_controller.reset_cmd()
            return True
        except Exception as e:
            cprint(f"Arm control error: {e}", "red")
            return False

    def move_to_receive_pose(self):
        """移动到接收物品的位置 (handover_pose)"""
        handover_pose = self.robot_config.get("handover_pose")
        if handover_pose is None:
            cprint("Error: handover_pose not defined in robot_config.json", "red")
            return False

        cprint("=================== Moving to receive pose (handover) ===================", "cyan")

        joint_angles = [
            handover_pose["J1"], handover_pose["J2"], handover_pose["J3"],
            handover_pose["J4"], handover_pose["J5"], handover_pose["J6"], handover_pose["J7"]
        ]

        trajectory = np.array([joint_angles])
        success = self.control_arm(trajectory=trajectory, speed=15)

        if success:
            cprint("=================== Reached receive pose ===================", "green")
        return success

    def wait_for_user_to_place(self, wait_time=3.0):
        """
        等待用户将物品放入机械臂手中

        Args:
            wait_time: 等待时间(秒)
        """
        cprint(f"=================== Waiting {wait_time}s for user to place item ===================", "cyan")
        time.sleep(wait_time)
        cprint("=================== Wait complete, closing hand ===================", "green")

    def get_placing_position(self, class_name=None, image="", vis=False):
        """获取放置位置 (容器中心点)"""
        self.yolo_model.set_classes([class_name])
        results = self.yolo_model.predict(source=Image.fromarray(image), conf=0.25)

        if len(results) == 0 or results[0].boxes is None or len(results[0].boxes) == 0:
            detections = []
        else:
            detections = results[0].boxes.data.tolist()

        try:
            if len(detections):
                det = detections[0][:4]
                # 如果检测框在图像下半部分且有多个检测，取第二个
                if det[1] >= 400 and det[3] <= 480 and len(detections) > 1:
                    det = detections[1][:4]

                x1, y1, x2, y2 = [int(coord) for coord in det]

                H, W = self.depth.shape
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(W, x2), min(H, y2)

                depth_sub_image_mm = self.depth[y1:y2, x1:x2]
                valid_depths_mm = depth_sub_image_mm[depth_sub_image_mm > 0]

                if len(valid_depths_mm) > 0:
                    mean_depth_mm = np.median(valid_depths_mm)
                else:
                    cprint("Warning: No valid depth values in bounding box", "yellow")
                    return []

                mean_depth_m = mean_depth_mm * 1e-3

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_cam_point = pixel_to_camera_point2(
                    np.array([center_x, center_y]).reshape(-1, 2), mean_depth_m
                )
                center_cam_point = center_cam_point.flatten()

                placing_pos_world = self.transform_pose_to_world(center_cam_point)

                if vis:
                    plt.figure()
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    plt.imshow(image)
                    plt.gca().add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, color='red'))
                    plt.gca().text(x1, y1 - 5, f'{class_name}', color='red', fontsize=10, backgroundcolor='none')
                    plt.savefig(f"./log/placement_rgb_{timestamp}.png")
                    plt.close()

                return placing_pos_world
        except Exception as e:
            cprint(f"Error getting placing position: {e}", "red")
            return []

    def transform_pose_to_world(self, pose_cam_point):
        """将相机坐标系下的位置转换到世界坐标系"""
        placing_translation = pose_cam_point.flatten()
        T_cam_point = np.eye(4, 4)
        T_cam_point[:3, :3] = R.from_quat([-0.210, 0.016, -0.056, 0.976]).as_matrix()
        T_cam_point[:3, 3] = placing_translation
        T_world_point = self.T_base_to_cam @ T_cam_point
        T_world_pose = T_world_point.copy()
        return T_world_pose

    def create_send_config_3(self, placing_pos_position, placing_pos_orn, current_js_pose=None):
        """创建放置轨迹配置"""
        if current_js_pose is None:
            self.config = {
                "target_pose": [[placing_pos_position[0], placing_pos_position[1], placing_pos_position[2],
                        placing_pos_orn[0], placing_pos_orn[1], placing_pos_orn[2], placing_pos_orn[3]]
                        ],
                "current_js": [self.default_traj_js_rad[0], self.default_traj_js_rad[1], self.default_traj_js_rad[2],
                                self.default_traj_js_rad[3], self.default_traj_js_rad[4], self.default_traj_js_rad[5], self.default_traj_js_rad[6]],
                "struct": "left_arm"
            }
        else:
            self.config = {
                "target_pose": [[placing_pos_position[0], placing_pos_position[1], placing_pos_position[2],
                        placing_pos_orn[0], placing_pos_orn[1], placing_pos_orn[2], placing_pos_orn[3]]
                        ],
                "current_js": [current_js_pose[0], current_js_pose[1], current_js_pose[2],
                                current_js_pose[3], current_js_pose[4], current_js_pose[5], current_js_pose[6]],
                "struct": "left_arm"
            }

    def create_twin_service(self, type=None, cnfg=None):
        """调用Twin服务"""
        cmd = {"srv": "twin_inference", "type": type, "cnfg": cnfg}
        resp = self.send_cmd_twin(self.twin_client, cmd)
        return resp

    def execute_placement_js(self, placement_pos_world):
        """执行放置动作 (使用Twin服务生成轨迹)"""
        # 抬高放置位置
        placement_pos_world[2, 3] += 0.15
        placement_pos_arm = placement_pos_world @ self.T_hand_effector_to_arm_endlink
        placement_pos_arm_pos_position = placement_pos_arm[:3, 3]
        placement_pos_arm_pos_orientation = R.from_matrix(placement_pos_arm[:3, :3]).as_quat()

        self.create_send_config_3(placement_pos_arm_pos_position, placement_pos_arm_pos_orientation)
        rsp = self.create_twin_service(type="trajectory_generation2", cnfg=self.config)
        state = rsp["value"]

        if state:
            trajectory_placing = rsp["info"]["trajectory"]
            trajectory_placing = np.array(copy.deepcopy(trajectory_placing)) / np.pi * 180
            self.control_arm(trajectory=trajectory_placing, speed=20)
            cprint("=================== Reach placing pose =============", "green")
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            cprint("=================== Released item =============", "green")
            return True
        else:
            cprint("********************* The placing pose is not reachable !! *********************", "yellow")
            return False

    def execute_throw_to_trash_js(self):
        """执行丢垃圾操作：移动到预设的throw_to_trash_pose位置并松开手"""
        throw_pose = self.robot_config.get("throw_to_trash_pose")
        if throw_pose is None:
            cprint("Error: throw_to_trash_pose not defined in robot_config.json", "red")
            return False

        cprint("=================== Moving to trash pose =============", "cyan")

        # 转换为关节角度列表 [J1, J2, J3, J4, J5, J6, J7]
        joint_angles = [
            throw_pose["J1"], throw_pose["J2"], throw_pose["J3"],
            throw_pose["J4"], throw_pose["J5"], throw_pose["J6"], throw_pose["J7"]
        ]

        # 使用control_arm的trajectory模式直接发送关节角度
        trajectory = np.array([joint_angles])
        self.control_arm(trajectory=trajectory, speed=15)

        cprint("=================== Reached trash pose =============", "green")
        time.sleep(0.5)

        # 松开手丢掉物品
        self.control_hand(cmd_type="open")
        cprint("=================== Opened hand to drop trash =============", "green")
        time.sleep(1)

        # 返回到安全位置
        self.control_arm(pose_type="grasp1", speed=30)
        cprint("=================== Returned to safe pose =============", "cyan")

        return True

    def execute_desk_placement_js(self):
        """执行桌面放置操作：从desk_pose_1/2/3中随机选择一个位置放置物品"""
        import random

        desk_poses = ["desk_pose_1", "desk_pose_2", "desk_pose_3"]
        selected_pose_key = random.choice(desk_poses)
        desk_pose = self.robot_config.get(selected_pose_key)

        if desk_pose is None:
            cprint(f"Error: {selected_pose_key} not defined in robot_config.json", "red")
            return False

        cprint(f"=================== Moving to desk pose ({selected_pose_key}) =============", "cyan")

        # 转换为关节角度列表 [J1, J2, J3, J4, J5, J6, J7]
        joint_angles = [
            desk_pose["J1"], desk_pose["J2"], desk_pose["J3"],
            desk_pose["J4"], desk_pose["J5"], desk_pose["J6"], desk_pose["J7"]
        ]

        # 使用control_arm的trajectory模式直接发送关节角度
        trajectory = np.array([joint_angles])
        self.control_arm(trajectory=trajectory, speed=15)

        cprint(f"=================== Reached desk pose ({selected_pose_key}) =============", "green")
        time.sleep(0.5)

        # 松开手放置物品
        self.control_hand(cmd_type="open")
        cprint("=================== Opened hand to place on desk =============", "green")
        time.sleep(1)

        # 返回到安全位置
        self.control_arm(pose_type="grasp1", speed=30)
        cprint("=================== Returned to safe pose =============", "cyan")

        return True

    def run_pipeline(self):
        """
        主流程：从用户手中接收物品并放置到指定容器

        流程:
        1. 读取JSON输入获取容器信息
        2. 移动到handover_pose
        3. 张开手等待
        4. 拍照识别物品 (可选)
        5. 等待用户放入物品
        6. 关闭手并检测是否成功抓取
        7. 移动到容器位置并放置
        8. 返回安全位置
        """
        # ============ Step 1: 获取JSON输入 ============
        json_data = self.get_json_input()
        if json_data is None:
            cprint("No valid JSON input received", "red")
            return False

        cprint(f"=================== 1. Get JSON input: {json_data} ===================", "cyan")

        container = self.parse_input(json_data)
        if container is None:
            cprint("JSON input missing required field: container", "red")
            return False

        cprint(f"=================== 2. Target container: {container} ===================", "cyan")

        # ============ Step 2: 移动到接收位置 ============
        if not self.move_to_receive_pose():
            cprint("Failed to move to receive pose", "red")
            return False

        # ============ Step 3: 张开手等待 ============
        self.control_hand(cmd_type="open")
        cprint("=================== 3. Hand opened, ready to receive item ===================", "green")

        # ============ Step 4: 等待用户放入物品 ============
        self.wait_for_user_to_place(wait_time=1.0)

        # ============ Step 5: 关闭手并检测是否成功抓取 ============
        has_object = self.check_grasping_object()

        if not has_object:
            cprint("=================== 4. No object detected in hand! Please place the item. ===================", "red")
            # 可以选择重试或返回失败
            # 这里我们再给用户一次机会
            cprint("Retrying... Please place the item in the hand now!", "yellow")
            self.control_hand(cmd_type="open")
            self.wait_for_user_to_place(wait_time=3.0)
            has_object = self.check_grasping_object()

            if not has_object:
                cprint("=================== Still no object! Task failed. ===================", "red")
                self.control_arm(pose_type="grasp1", speed=30)
                return False

        cprint("=================== 4. Successfully grabbed object! ===================", "green")
        time.sleep(0.5)

        # ============ Step 6: 移动到容器位置并放置 ============
        # 检测是否是垃圾桶模式
        if container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            cprint("=================== Trash mode detected: throwing to trash ===================", "cyan")
            check = self.execute_throw_to_trash_js()
            if check:
                cprint("=================== 7. Task completed successfully! ===================", "green")
            else:
                cprint("=================== Trash task failed ===================", "red")
            return check

        # 检测是否是桌面放置模式
        if container.lower() in ["desk", "桌子", "table"]:
            cprint("=================== Desk placement mode detected: placing on desk ===================", "cyan")
            check = self.execute_desk_placement_js()
            if check:
                cprint("=================== 7. Task completed successfully! ===================", "green")
            else:
                cprint("=================== Desk placement task failed ===================", "red")
            return check

        # 普通放置模式：寻找容器并放置
        check = False
        for key in self.default_traj_js.keys():
            if "grasp" in key:
                self.control_arm(pose_type=key, speed=30)
                self.rgb, self.depth = self.get_camera_obs()
                self.save_current_transformation()

                cprint(f"=================== 5. Looking for container: {container} ===================", "cyan")
                placing_pos_world = self.get_placing_position(
                    class_name=container, image=self.rgb, vis=False
                )

                if len(placing_pos_world) == 0:
                    cprint(f"Container not found from pose {key}, trying next...", "yellow")
                    continue

                cprint(f"=================== 6. Container found, executing placement ===================", "cyan")
                check = self.execute_placement_js(placing_pos_world)

                if check:
                    break

        if not check:
            cprint("=================== Placement failed! ===================", "red")
            self.control_arm(pose_type="grasp1", speed=30)
            return False

        # ============ Step 7: 返回安全位置 ============
        self.control_arm(pose_type="grasp1", speed=30)
        cprint("=================== 7. Task completed successfully! ===================", "green")
        return True


if __name__ == '__main__':
    planner = PlannerFetchFromUser(
        yolo_model_path="/home/zz/ros_proj/erdaiji_ws/src/anygrasp_ros/src/yolo_world/yolov8x-worldv2.pt",
        robot_config_path="./robot_config.json",
        save_path="./log"
    )
    success = planner.run_pipeline()
    if success:
        print("任务执行成功")
    else:
        print("任务执行失败")
