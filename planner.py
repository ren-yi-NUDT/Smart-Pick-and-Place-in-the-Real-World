import os, sys
curr_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(curr_dir)
import struct
import json, os, copy, rospy
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
from anygrasp_sdk.grasp_detection.anygrasp_get_poses import anygrasp_get_poses
from scipy.spatial.transform import Rotation as R
from utils import graspcam2pixel, self_rotation_np, rpy_to_vector, transform_world_to_camera, self_rotation_inv, visualization, pixel_to_camera_point, pixel_to_camera_point2

class Planner:
    def __init__(self, yolo_model_path="", anygrasp_model_path="", robot_config_path="", save_path=""):
        self.save_path = save_path
        self.checkpoint_path = anygrasp_model_path
        self.cam = RealSenseCapture(width=640, height=480, fps=30, save_path=save_path)
        self.json_parser = JsonInputParser()
        self.yolo_model = YOLOWorld(yolo_model_path)
        self.transform = TransformationUtil()
        self.arm_controller = ArmController()
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
        self.robot_config = json.load(open(robot_config_path, "r"))
        self.default_traj_js = self.robot_config["default_traj_js"]
        
    def send_cmd(self, sock, data):
        msg = json.dumps(data).encode('utf-8')
        sock.sendall(msg)
        resp = json.loads(sock.recv(1024).decode('utf-8'))
        cprint(f"Control hand response: {resp}", "red")
        return resp

    def send_cmd_twin(self, sock, data):
        msg = json.dumps(data).encode('utf-8')
        sock.sendall(msg)

        length_bytes = b''
        while len(length_bytes) < 4:
            chunk = sock.recv(4 - len(length_bytes))
            if not chunk:
                raise ConnectionError("Connection closed")
            length_bytes += chunk
        
        data_length = struct.unpack('>I', length_bytes)[0]
        
        data_bytes = b''
        while len(data_bytes) < data_length:
            chunk = sock.recv(min(4096, data_length - len(data_bytes)))
            if not chunk:
                raise ConnectionError("Connection closed")
            data_bytes += chunk
        
        resp = json.loads(data_bytes.decode('utf-8'))
        cprint(f"Control hand response: {resp}", "red")
        return resp

    def control_hand(self, cmd_type="close"):
        self.hand_config = {
            "close": [0, 0, 0, 460, 0, 0],
            "open": [1000, 1000, 1000, 1000, 1000, 0]
        }
        if cmd_type == "close":
            cmd = {"src": "/left_hand/movement_control", "type": "set", "cmd": self.hand_config["close"]}
            resp = self.send_cmd(self.hand_client, cmd)
        elif cmd_type == "open":
            cmd = {"src": "/left_hand/movement_control", "type": "set", "cmd": self.hand_config["open"]}
            resp = self.send_cmd(self.hand_client, cmd)
        elif cmd_type == "get_state":
            cmd = {"src": "/left_hand/movement_control", "type": "get"}
            resp = self.send_cmd(self.hand_client, cmd)
            return resp

    def get_camera_obs(self):
        rgb, depth = self.cam.get_rgbd()
        return rgb, depth

    def get_json_input(self):
        """从stdin读取JSON命令输入"""
        result = self.json_parser.get_command()
        return result

    def parse_input(self, json_data):
        """解析JSON数据，提取物体和容器信息"""
        if json_data is None:
            return None, None
        obj = json_data.get('object')
        container = json_data.get('container')
        return obj, container

    def save_current_transformation(self,):
        transform_from_frame = self.robot_config["base_link_name"]
        transform_to_frame = self.robot_config["camera_link_name"]
        self.T_base_to_cam, _, _ = self.transform.get_transform_from_frame_to_frame(transform_from_frame, transform_to_frame)
        grasping_from_frame = self.robot_config["hand_effector_name"]
        grasping_to_frame = self.robot_config["arm_end_link_name"]
        self.T_hand_effector_to_arm_endlink, _, _ = self.transform.get_transform_from_frame_to_frame(grasping_from_frame, grasping_to_frame)

    def filtering_pose(self, anygrasp_pose, class_name="", image="", return_label=False, vis=True):
        class_name = [cls for cls in class_name.split(',')]
        self.yolo_model.set_classes(class_name)
        results = self.yolo_model.predict(source=Image.fromarray(image), conf=0.2)
        if len(results) == 0 or results[0].boxes is None or len(results[0].boxes) == 0:
            detections = []
        else:
            detections = results[0].boxes.data.tolist()

        grasp_points, grasp_pose_cam = graspcam2pixel(anygrasp_pose)
        valid_indices = set()
        final_grasps = []
        valid_boxes = []
        ans = False
        if len(detections):
            det = detections[0][:4] # detections[0][2]
            x1, y1, x2, y2 = det
            valid_boxes.append((x1, y1, x2, y2))
            for i, grasp_p in enumerate(grasp_points):
                if grasp_p[0] > x1 - 20 and grasp_p[0] < x2 + 20 and grasp_p[1] > y1 - 20 and grasp_p[1] < y2 + 20:
                    valid_indices.add(i)

            if len(valid_indices):
                sorted_indices = sorted(list(valid_indices))
                for i in sorted_indices:
                    g_pose = grasp_pose_cam[i]
                    if return_label:
                        if isinstance(g_pose, dict):
                            g_pose["label"] = class_name
                    final_grasps.append(g_pose)
                cprint(f"*********** Class name {class_name} ****************** Grasp pose number: {len(final_grasps)} ******************", "red")
                ans = True
            else:
                cprint(f"Found objects ({class_name}) but NO grasp points inside them.", "yellow")
        else:
            cprint(f"No object detected for class: {class_name}", "yellow")

        if vis:
            plt.figure(figsize=(10, 8))
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plt.imshow(image)
            ax = plt.gca()
            
            for box in valid_boxes:
                x1, y1, x2, y2 = box
                ax.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, color='red', linewidth=2))
                ax.text(x1, y1 - 5, class_name, color='red', fontsize=10, weight='bold')

            if final_grasps:
                for idx in valid_indices:
                    plt.plot(grasp_points[idx][0], grasp_points[idx][1], 'g*', markersize=8, label='Valid')
            else:
                if len(grasp_points) > 0:
                    plt.plot(grasp_points[:, 0], grasp_points[:, 1], 'b.', markersize=6, alpha=0.5)
            plt.title(f'{class_name}: {len(valid_boxes)} objects, {len(final_grasps)} grasps')
            plt.axis('off')
            save_filename = f"filtered_rgb_{timestamp}_{class_name}.png"
            plt.savefig(os.path.join(self.save_path, save_filename))
            plt.close()

        return final_grasps if ans else []

    def get_placing_position(self, class_name=None, image="", vis=False):
        self.yolo_model.set_classes([class_name])
        results = self.yolo_model.predict(source=Image.fromarray(image), conf=0.25)
        if len(results) == 0 or results[0].boxes is None or len(results[0].boxes) == 0:
            detections = []
        else:
            detections = results[0].boxes.data.tolist()

        try:
            if len(results):
                det = detections[0][:4] # detections[0][2]
                if det[1] >= 400 and det[3] <= 480 and len(detections) > 1:
                    det = detections[1][:4]
                x1, y1, x2, y2 = [int(coord) for coord in det]

                H, W = self.depth.shape
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(W, x2)
                y2 = min(H, y2)
                
                depth_sub_image_mm = self.depth[y1:y2, x1:x2]
                
                valid_depths_mm = depth_sub_image_mm[depth_sub_image_mm > 0]

                if len(valid_depths_mm) > 0:
                    mean_depth_mm = np.median(valid_depths_mm)
                else:
                    print("Warning: No valid depth values found in the bounding box.")
                
                mean_depth_m = mean_depth_mm * 1e-3

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_cam_point = pixel_to_camera_point2(np.array([center_x, center_y]).reshape(-1, 2), mean_depth_m)
                center_cam_point = center_cam_point.flatten()
                placing_pos_world = self.transform_pose_to_world(center_cam_point)

                if vis:
                    plt.figure()
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    plt.imshow(image)
                    plt.gca().add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, color='red'))
                    plt.gca().text(x1, y1 - 5, f'{class_name}', color='red', fontsize=10, backgroundcolor='none')
                    plt.savefig(f"./log/placement_rgb_{timestamp}.png")

                return placing_pos_world
        except:
            return []

    def transform_pose_to_world(self, pose_cam_point):
        placing_translation = pose_cam_point.flatten()
        T_cam_point = np.eye(4, 4)
        T_cam_point[:3, :3] = R.from_quat([-0.210, 0.016, -0.056, 0.976]).as_matrix()
        T_cam_point[:3, 3] = placing_translation
        T_world_point = self.T_base_to_cam @ T_cam_point
        T_world_pose = T_world_point.copy()
        return T_world_pose

    def transform_y_axis(self, transformed_pose_world):
        r, p, y = R.from_matrix(transformed_pose_world[:3, :3]).as_euler('xyz', degrees=False)
        y_axis_rotated = rpy_to_vector(r, p, y, axis=[0, 1, 0])
        z_axis_world = np.array([0, 0, 1])
        cos_theta = np.dot(y_axis_rotated, z_axis_world) / (
            np.linalg.norm(y_axis_rotated) * np.linalg.norm(z_axis_world)
        )
        if cos_theta >= 0:
            transformed_pose_world = transformed_pose_world @ np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        return transformed_pose_world

    def transform_x_axis(self, transformed_pose_world):
        r, p, y = R.from_matrix(transformed_pose_world[:3, :3]).as_euler('xyz', degrees=False)
        x_axis_rotated = rpy_to_vector(r, p, y, axis=[1, 0, 0])
        y_axis_world = np.array([0, 1, 0])
        cos_theta = np.dot(x_axis_rotated, y_axis_world) / (
            np.linalg.norm(x_axis_rotated) * np.linalg.norm(y_axis_world)
        )
        if cos_theta > 0:
            transformed_pose_world = transformed_pose_world @ np.array([
                [-1, 0, 0, 0], 
                [0, -1, 0, 0], 
                [0, 0, 1, 0], 
                [0, 0, 0, 1]
            ])
        return transformed_pose_world

    def publish_single_pose(self, transform_matrix: np.ndarray, child_frame_id: str, parent_frame_id: str = "base_link"):
        if transform_matrix.shape != (4, 4):
            rospy.logerr("输入矩阵必须是 4x4 的齐次变换矩阵。")
            return

        translation = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        try:
            r = R.from_matrix(rotation_matrix)
            quat = r.as_quat() 
        except Exception as e:
            rospy.logerr(f"无法从旋转矩阵转换四元数：{e}")
            return

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform([t])
        rospy.loginfo(f"已成功发布位姿到 RViz：{child_frame_id}，Parent: {parent_frame_id}")

    def publish_grasp_transforms(self, parent_frame="base_link"):
        transforms_list = []
        for i, (original_id, item) in enumerate(self.final_grasp_pose_data[:]):
            pose_matrix = item["transformed_pose_world"]
            translation = pose_matrix[:3, 3]
            rotation_matrix = pose_matrix[:3, :3]
            r = R.from_matrix(rotation_matrix)
            quat = r.as_quat()

            t = TransformStamped()
            t.header.stamp = rospy.Time(0)
            t.header.frame_id = parent_frame
            t.child_frame_id = f"grasp_pose_hand_endeffector_{i}"
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            transforms_list.append(t)

        if transforms_list:
            self.tf_broadcaster.sendTransform(transforms_list)

    def transform_anygrasp_pose(self, anygrasp_pose, _visualization=True, return_labels=False):
        try:
            eval_score = {}
            for id, data in enumerate(anygrasp_pose[:]):
                grasp_translation = data['trans']
                grasp_rotation_matrix = data['rotation_matrix']

                grasp_transformation_matrix = np.eye(4, 4)
                grasp_transformation_matrix[:3, :3] = grasp_rotation_matrix
                grasp_transformation_matrix[:3, 3] = grasp_translation

                # self_pose_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                self_pose_matrix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                self.self_pose_matrix = self_rotation_np(self_pose_matrix)
                
                # self.publish_single_pose(grasp_transformation_matrix, "before", parent_frame_id="base_link")
                transformed_pose_camera = grasp_transformation_matrix @ self.self_pose_matrix
                # self.publish_single_pose(transformed_pose_camera, "after", parent_frame_id="base_link")
                transformed_pose_world = self.T_base_to_cam @ transformed_pose_camera
                # self.publish_single_pose(transformed_pose_world, "world", parent_frame_id="base_link")
                transformed_pose_world = self.transform_x_axis(transformed_pose_world)

                eval_score[id] = {}
                eval_score[id]["score"] = data["score"] # * self.evaluate_pose_mask(transformed_pose_world, from_frame="base_link", to_frame="base_link_r")
                eval_score[id]["transformed_pose_world"] = transformed_pose_world 
                eval_score[id]["original_pose"] = anygrasp_pose[id]
                if return_labels:
                    eval_score[id]["label"] = data["label"]

            self.final_grasp_pose_data = sorted(eval_score.items(), key=lambda x: x[1]["score"], reverse=True)

            self.publish_grasp_transforms(parent_frame="base_link")

            if return_labels:
                return [self.final_grasp_pose_data[i][1]["transformed_pose_world"] for i in range(len(self.final_grasp_pose_data))], [self.final_grasp_pose_data[i][1]["label"] for i in range(len(self.final_grasp_pose_data))]
            else:
                return [self.final_grasp_pose_data[i][1]["transformed_pose_world"] for i in range(len(self.final_grasp_pose_data))]
        except:
            return [], None

    def get_pose_and_save(self, rgb, depth):
        try:
            anygrasp_pose, self.point_cloud = anygrasp_get_poses(self.checkpoint_path, rgb, depth)
            # with open(os.path.join(self.save_path, "result.json"), "w") as f:
            #     json.dump(anygrasp_pose, f, indent=5)
            # f.close()
            return anygrasp_pose
        except:
            return False

    def control_arm(self, pose_type=None, trajectory=None, speed=20):
        try:
            self.arm_controller.start_cmd()
            if pose_type is not None:
                self.default_traj_js_rad = [data / 180 * np.pi for data in self.default_traj_js[pose_type].values()]
                self.arm_controller.add_js_cmd(self.default_traj_js[pose_type], speed=speed, block=True)
            elif trajectory is not None:
                for i in range(len(trajectory)):
                    self.arm_controller.add_js_cmd({'J1': trajectory[i][0], 'J2': trajectory[i][1], 'J3': trajectory[i][2],
                                                    'J4': trajectory[i][3], 'J5': trajectory[i][4], 'J6': trajectory[i][5], 'J7': trajectory[i][6]}, speed=speed, block=True)
            self.arm_controller.send_cmds(self.arm_client)
            self.arm_controller.reset_cmd()
            return True
        except:
            return False

    def create_send_config_2(self, preparasion_grasping_pos_position, preparasion_grasping_pos_orn, grasping_pos_position, grasping_pos_orn, current_js_pose=None, struct="left_arm"):
        if current_js_pose is None:
            self.config = {
                "target_pose": [
                        [preparasion_grasping_pos_position[0], preparasion_grasping_pos_position[1], preparasion_grasping_pos_position[2],
                        preparasion_grasping_pos_orn[0], preparasion_grasping_pos_orn[1], preparasion_grasping_pos_orn[2], preparasion_grasping_pos_orn[3]],
                        [grasping_pos_position[0], grasping_pos_position[1], grasping_pos_position[2],
                        grasping_pos_orn[0], grasping_pos_orn[1], grasping_pos_orn[2], grasping_pos_orn[3]]
                        ],
                "current_js": [self.default_traj_js_rad[0], self.default_traj_js_rad[1], self.default_traj_js_rad[2],
                                self.default_traj_js_rad[3], self.default_traj_js_rad[4], self.default_traj_js_rad[5], self.default_traj_js_rad[6]],
                "struct": struct
            }
        else:
            self.config = {
                "target_pose": [
                        [preparasion_grasping_pos_position[0], preparasion_grasping_pos_position[1], preparasion_grasping_pos_position[2],
                        preparasion_grasping_pos_orn[0], preparasion_grasping_pos_orn[1], preparasion_grasping_pos_orn[2], preparasion_grasping_pos_orn[3]],
                        [grasping_pos_position[0], grasping_pos_position[1], grasping_pos_position[2],
                        grasping_pos_orn[0], grasping_pos_orn[1], grasping_pos_orn[2], grasping_pos_orn[3]]
                        ],
                "current_js": [current_js_pose[0], current_js_pose[1], current_js_pose[2],
                                current_js_pose[3], current_js_pose[4], current_js_pose[5], current_js_pose[6]],
                "struct": struct
            }

    def create_send_config_3(self, placing_pos_position, placing_pos_orn, current_js_pose=None):
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
        cmd = {"srv": "twin_inference", "type": type, "cnfg": cnfg}
        resp = self.send_cmd_twin(self.twin_client, cmd)
        return resp

    def visualization_3d_grasping_pose(self, grasping_pose_world, translation_matrix2=None):
        pose_matrix = grasping_pose_world
        print(f"Pose of grasping in root frame: {pose_matrix} \n \
                        {R.from_matrix(pose_matrix[:3, :3]).as_euler('xyz', degrees=False)}")
        
        pose_c = transform_world_to_camera(pose_matrix, self.T_base_to_cam)
        self_rot_inv = self_rotation_inv(self.self_pose_matrix)
        rot_g = np.dot(pose_c[:3, :3], self_rot_inv)

        translation_matrix = np.eye(4, 4)
        translation_matrix[0:3, 0:3] = rot_g
        translation_matrix[0:3, 3] = pose_c[:3, 3]
        if translation_matrix2 is None:
            visualization(self.point_cloud, translation_matrix)
        else:
            visualization(self.point_cloud, np.concatenate((translation_matrix, translation_matrix2), axis=0))

    def check_grasping_object(self):
        time.sleep(0.7)
        self.control_hand(cmd_type="close")
        value = self.control_hand(cmd_type="get_state")["value"]
        diff = np.array(value) - np.array(list(self.hand_config["close"]))
        if abs(diff.sum()) > 20:
            return True
        else:
            return False

    def execute_grasping_twin_js_2(self, grasping_pose_world_hand, idx=None):
        self.control_hand(cmd_type="open")
        basic_mat = np.eye(4)
        basic_mat[2, 3] -= 0.02
        preparasion_grasping_pos = grasping_pose_world_hand @ basic_mat
        preparasion_grasping_pos[2, 3] += 0.02
        preparasion_grasping_pos = preparasion_grasping_pos @ self.T_hand_effector_to_arm_endlink
        preparasion_grasping_pos_position = preparasion_grasping_pos[:3, 3]
        preparasion_grasping_pos_orientation = R.from_matrix(preparasion_grasping_pos[:3, :3]).as_quat()
        # self.publish_pose_as_tf(preparasion_grasping_pos, "preparasion_grasping_pos_arm_end")
        # self.publish_grasping_pose(preparasion_grasping_pos, f"preparasion_grasping_pos_arm_end")

        basic_mat = np.eye(4)
        execution_grasping_pos = grasping_pose_world_hand @ basic_mat
        print("=-============================", execution_grasping_pos[2, 3])
        execution_grasping_pos[2, 3] = max(execution_grasping_pos[2, 3], 0.042)
        # self.publish_grasping_pose(execution_grasping_pos, f"execution_grasping_pos_hand_end")
        execution_grasping_pos = execution_grasping_pos @ self.T_hand_effector_to_arm_endlink
        execution_grasping_pos_position = execution_grasping_pos[:3, 3]
        execution_grasping_pos_orientation = R.from_matrix(execution_grasping_pos[:3, :3]).as_quat()

        self.create_send_config_2(preparasion_grasping_pos_position, preparasion_grasping_pos_orientation, execution_grasping_pos_position, execution_grasping_pos_orientation)
        rsp = self.create_twin_service(type="trajectory_generation2", cnfg=self.config)
        state = rsp["value"]

        if state:
            trajectory_grasping = rsp["info"]["trajectory"]
            trajectory_grasping = np.array(copy.deepcopy(trajectory_grasping)) / np.pi * 180
            self.control_arm(trajectory=trajectory_grasping, speed=20)
            cprint(f"=============== Reach grasping pose =============")
            self.control_hand(cmd_type="close")
            # stop_event = threading.Event()
            # grasping_task = threading.Thread(target=self.control_hand_continuing, kwargs={"type": "grasping"})
            # grasping_task.start()
            cprint("=============== Close hand =============")
            time.sleep(0.5)
            # post_trajectory = copy.deepcopy(list(trajectory_grasping))[::-1]
            # self.control_arm(trajectory=post_trajectory, speed=20, use_block = False)
            self.control_arm(pose_type=idx, speed=30)
            cprint(f"=============== Reach post grasping pose =============")
            return True
        else:
            cprint(f"********************* The preparasion pose is not reachable !! *********************")
            return False

    def execute_placement_js(self, placement_pos_world):
        placement_pos_world[2, 3] += 0.15
        placement_pos_arm = placement_pos_world @ self.T_hand_effector_to_arm_endlink
        placement_pos_arm_pos_position = placement_pos_arm[:3, 3]
        placement_pos_arm_pos_orientation = R.from_matrix(placement_pos_arm[:3, :3]).as_quat()
        self.publish_single_pose(placement_pos_world, "execution_placing_pos_hand_end", parent_frame_id="base_link")
        
        self.create_send_config_3(placement_pos_arm_pos_position, placement_pos_arm_pos_orientation)
        rsp = self.create_twin_service(type="trajectory_generation2", cnfg=self.config)
        state = rsp["value"]

        if state:
            trajectory_placing = rsp["info"]["trajectory"]
            trajectory_placing = np.array(copy.deepcopy(trajectory_placing)) / np.pi * 180
            self.control_arm(trajectory=trajectory_placing, speed=20)
            cprint(f"=============== Reach placing pose =============")
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            # post_trajectory = copy.deepcopy(list(trajectory_placing))[::-1]
            # self.control_arm(trajectory=post_trajectory, speed=20, use_block = False)
            # self.control_arm(pose_type="place1", speed=30)
            cprint(f"=============== Reach post placing pose =============")
            return True
        else:
            cprint(f"********************* The placing pose is not reachable !! *********************", "yellow")
            return False

    def execute_handover_js(self):
        """执行递送操作：平滑移动经过中间姿态到handover位置并松开手

        轨迹: 当前位置 → get_ready_to_handover_1st → get_ready_to_handover_2nd → handover_pose
        """
        # 获取所有需要的位姿
        pose_1st = self.robot_config.get("get_ready_to_handover_1st")
        pose_2nd = self.robot_config.get("get_ready_to_handover_2nd")
        handover_pose = self.robot_config.get("handover_pose")

        if pose_1st is None or pose_2nd is None or handover_pose is None:
            cprint("错误: robot_config.json 中缺少 handover 相关位姿定义", "red")
            return False

        cprint(f"=============== Moving to handover pose (smooth trajectory) =============", "cyan")

        # 提取关节角度的辅助函数
        def extract_joints(pose_dict):
            return [
                pose_dict["J1"], pose_dict["J2"], pose_dict["J3"],
                pose_dict["J4"], pose_dict["J5"], pose_dict["J6"], pose_dict["J7"]
            ]

        # 构建平滑轨迹：1st → 2nd → handover
        waypoint_joints = [
            extract_joints(pose_1st),
            extract_joints(pose_2nd),
            extract_joints(handover_pose)
        ]

        # 生成插值轨迹
        trajectory = self._interpolate_joint_trajectory(waypoint_joints, steps_per_segment=20)

        # 执行平滑轨迹
        self.control_arm(trajectory=trajectory, speed=15)

        cprint(f"=============== Reached handover pose =============", "green")
        time.sleep(0.5)

        # 松开手递送物品
        self.control_hand(cmd_type="open")
        cprint(f"=============== Opened hand for handover =============", "green")
        time.sleep(1)

        # 返回到安全位置
        self.control_arm(pose_type="grasp1", speed=30)
        cprint(f"=============== Returned to safe pose =============", "cyan")

        return True

    def _interpolate_joint_trajectory(self, waypoint_joints, steps_per_segment=20):
        """
        在多个关节空间waypoint之间生成平滑插值轨迹

        Args:
            waypoint_joints: list of joint angle lists [[J1,J2...J7], [J1,J2...J7], ...]
            steps_per_segment: 每段轨迹的插值点数

        Returns:
            trajectory: numpy array of shape (N, 7)
        """
        trajectory = []
        for i in range(len(waypoint_joints) - 1):
            start_joints = np.array(waypoint_joints[i])
            end_joints = np.array(waypoint_joints[i + 1])

            for step in range(steps_per_segment):
                t = step / steps_per_segment
                interpolated = start_joints + t * (end_joints - start_joints)
                trajectory.append(interpolated)

        # 添加最后一个waypoint
        trajectory.append(np.array(waypoint_joints[-1]))

        return np.array(trajectory)

    def execute_throw_to_trash_js(self):
        """执行丢垃圾操作：移动到预设的throw_to_trash_pose位置并松开手"""
        throw_pose = self.robot_config.get("throw_to_trash_pose")
        if throw_pose is None:
            cprint("错误: robot_config.json 中没有定义 throw_to_trash_pose", "red")
            return False

        cprint(f"=============== Moving to trash pose =============", "cyan")

        # 转换为关节角度列表 [J1, J2, J3, J4, J5, J6, J7]
        joint_angles = [
            throw_pose["J1"],
            throw_pose["J2"],
            throw_pose["J3"],
            throw_pose["J4"],
            throw_pose["J5"],
            throw_pose["J6"],
            throw_pose["J7"]
        ]

        # 使用control_arm的trajectory模式直接发送关节角度
        trajectory = np.array([joint_angles])
        self.control_arm(trajectory=trajectory, speed=15)

        cprint(f"=============== Reached trash pose =============", "green")
        time.sleep(0.5)

        # 松开手丢掉物品
        self.control_hand(cmd_type="open")
        cprint(f"=============== Opened hand to drop trash =============", "green")
        time.sleep(1)

        # 返回到安全位置
        self.control_arm(pose_type="grasp1", speed=30)
        cprint(f"=============== Returned to safe pose =============", "cyan")

        return True

    def execute_desk_placement_js(self):
        """执行桌面放置操作：从desk_pose_1/2/3中随机选择一个位置放置物品"""
        import random

        desk_poses = ["desk_pose_1", "desk_pose_2", "desk_pose_3"]
        selected_pose_key = random.choice(desk_poses)
        desk_pose = self.robot_config.get(selected_pose_key)

        if desk_pose is None:
            cprint(f"错误: robot_config.json 中没有定义 {selected_pose_key}", "red")
            return False

        cprint(f"=============== Moving to desk pose ({selected_pose_key}) =============", "cyan")

        # 转换为关节角度列表 [J1, J2, J3, J4, J5, J6, J7]
        joint_angles = [
            desk_pose["J1"],
            desk_pose["J2"],
            desk_pose["J3"],
            desk_pose["J4"],
            desk_pose["J5"],
            desk_pose["J6"],
            desk_pose["J7"]
        ]

        # 使用control_arm的trajectory模式直接发送关节角度
        trajectory = np.array([joint_angles])
        self.control_arm(trajectory=trajectory, speed=15)

        cprint(f"=============== Reached desk pose ({selected_pose_key}) =============", "green")
        time.sleep(0.5)

        # 松开手放置物品
        self.control_hand(cmd_type="open")
        cprint(f"=============== Opened hand to place on desk =============", "green")
        time.sleep(1)

        # 返回到安全位置
        self.control_arm(pose_type="grasp1", speed=30)
        cprint(f"=============== Returned to safe pose =============", "cyan")

        return True

    def run_pipeline(self):
        """主流程：从stdin读取JSON命令，执行抓取和放置任务（单次执行）"""
        json_data = self.get_json_input()

        if json_data is None:
            cprint("未收到有效的JSON输入", "red")
            return False

        cprint(f"=================== 1. Get JSON input: {json_data} ===================", "cyan")

        obj, container = self.parse_input(json_data)
        if obj is None or container is None:
            cprint("JSON输入缺少必需字段 (object 或 container)", "red")
            return False

        cprint(f"=================== 2. Parse input: Grasp {obj} and place it in the {container} ===================", "cyan")
        self.control_hand(cmd_type="close")

        # grasp phase
        check = False  # 初始化抓取成功标志
        check_object = False
        for key, value in self.default_traj_js.items():
            if "grasp" in key:
                self.control_arm(pose_type=key, speed=30)

                self.rgb, self.depth = self.get_camera_obs()
                cprint(f"G=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")
                anygrasp_pose = self.get_pose_and_save(self.rgb, self.depth)
                cprint(f"G=================== 4. Generate the grasping pose and save it in file: {self.save_path}/result.json ===================", "cyan")
                self.save_current_transformation()
                if not anygrasp_pose:
                    continue

                if obj is not None:
                    filtering_grasping_pose = self.filtering_pose(anygrasp_pose, class_name=obj, image=self.rgb)

                if not filtering_grasping_pose:
                    continue

                grasping_pose_world = self.transform_anygrasp_pose(filtering_grasping_pose, _visualization=False)

                if not len(grasping_pose_world):
                    continue

                for i in range(len(grasping_pose_world)):
                    cprint(f"=================== Checking pose: {i+1} / {len(grasping_pose_world)} ===================", "yellow")
                    check = self.execute_grasping_twin_js_2(grasping_pose_world[i], idx=key)

                    if check:
                        break

                if not check:
                    continue
                else:
                    break

        if not check:
            cprint("G=================== Grasping task failed ===================", "red")
            return False

        cprint(f"G=================== 5. Successfully completed the grasping task ===================", "green")

        # placement phase - check if handing over to a person
        if container.lower() == "person":
            cprint(f"H=================== Handover mode detected: delivering to person ===================", "cyan")
            check = self.execute_handover_js()
            if check:
                cprint(f"H=================== 5. Successfully completed the handover task ===================", "green")
                return True
            else:
                cprint(f"H=================== Handover task failed ===================", "red")
                return False

        # placement phase - check if throwing to trash
        if container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            cprint(f"T=================== Trash mode detected: throwing to trash ===================", "cyan")
            check = self.execute_throw_to_trash_js()
            if check:
                cprint(f"T=================== 5. Successfully completed the trash task ===================", "green")
                return True
            else:
                cprint(f"T=================== Trash task failed ===================", "red")
                return False

        # placement phase - check if placing on desk
        if container.lower() in ["desk", "桌子", "table"]:
            cprint(f"D=================== Desk placement mode detected: placing on desk ===================", "cyan")
            check = self.execute_desk_placement_js()
            if check:
                cprint(f"D=================== 5. Successfully completed the desk placement task ===================", "green")
                return True
            else:
                cprint(f"D=================== Desk placement task failed ===================", "red")
                return False

        # normal placement phase
        for key, value in self.default_traj_js.items():
            if "grasp" in key:
                self.control_arm(pose_type=key, speed=30)
                self.rgb, self.depth = self.get_camera_obs()
                cprint(f"P=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")
                self.save_current_transformation()
                placing_pos_world = self.get_placing_position(class_name=container, image=self.rgb, vis=False)
                cprint(f"P=================== 4. Generate the placing pose ===================", "cyan")

                if not len(placing_pos_world):
                    continue

                check = self.execute_placement_js(placing_pos_world)

                if check:
                    break

        self.control_arm(pose_type="grasp1", speed=30)
        cprint(f"P=================== 5. Successfully completed the placement task ===================", "green")
        return True

if __name__ == '__main__':
    planner = Planner(yolo_model_path="/home/zz/ros_proj/erdaiji_ws/src/anygrasp_ros/src/yolo_world/yolov8x-worldv2.pt",
                      anygrasp_model_path="/home/zz/ros_proj/erdaiji_ws/src/anygrasp_ros/src/anygrasp_sdk/checkpoint_detection.tar",
                      robot_config_path="./robot_config.json",
                      save_path="./log")
    success = planner.run_pipeline()
    if success:
        print("任务执行成功")
    else:
        print("任务执行失败")