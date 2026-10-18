#!/home/zz/anaconda3/envs/anygrasp/bin/python3.9

import os, sys
sys.path.append(os.path.dirname(__file__))
import rospy
import numpy as np
import json
import time
import pybullet as p
import pybullet_data
import threading
import copy
import socket, struct

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from termcolor import cprint

from robot import ErdaijiRobot
import tqdm
from sim_world import World
from utils import save_dict_to_json, slerp, draw_axis, calcul_transformation_matrix
# from simulation_bringup.srv import StringRequest, StringRequestResponse
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
# pynput 只给 __main__ 里的"按 ` 退出"用, 放在 __main__ 里导入,
# 免得 socket 服务/单元测试因为缺这个包就起不来。

SIMULATION_STEP_DELAY = 1 / 240.
ROUND_NUMBER = 3

def visualize_pose(pos, orn, d = 1):
    T = calcul_transformation_matrix(pos, orn)
    draw_axis(T, duration=d)


SUPPORTED_TWIN_SRV_TYPE = ["reachability_check", "collision_check", "IK_calculation", "trajectory_generation" , "trajectory_generation2"]

class TwinTest2(World):
    def __init__(self, vis=True):
        self.vis = vis
        self.camera_refresh_freq = 50
        urdf_dir = os.path.join(os.path.dirname(__file__), "../smart_pick_and_place_ws/src/rm_description/urdf/SingleArm")
        self.robot_path = os.path.join(urdf_dir, "rm65.urdf")
        self.robot_config_path = os.path.join(urdf_dir, "robot_config.json")
        self.robot = ErdaijiRobot((0.0, 0.0, 0.0), (0, 0, 0), robot_path = self.robot_path, config_path = self.robot_config_path, fixed_robot = True, vis=self.vis)        

        self.srv_name = "twin_inference"
        self.server_ip = '0.0.0.0'
        self.server_port = 8020
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(5)

        self.socket_thread = threading.Thread(target=self.socket_server_worker)
        self.socket_thread.daemon = True
        # 一次只跑一个推理: PyBullet 仿真状态非线程安全, 并发请求会互相污染。
        self.exec_lock = threading.Lock()
        self.socket_thread.start()

        self.setup()

    def socket_server_worker(self):
        """Socket 监听循环"""
        while not rospy.is_shutdown():
            try:
                self.server_socket.settimeout(1.0)
                try:
                    conn, addr = self.server_socket.accept()
                    # 每连接一线程: 同 8010, 一个空闲连接会让后续客户端全饿死。
                    threading.Thread(target=self.handle_client, args=(conn,),
                                     daemon=True).start()
                except socket.timeout:
                    continue
            except Exception as e:
                print(f"Socket Server Error: {e}")
                rospy.sleep(1)

    @staticmethod
    def _recv_exact(conn, n):
        """循环 recv 直到收满 n 字节; 连接关闭返回 None。"""
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def handle_client(self, conn):
        """处理单个客户端请求 (请求/响应都是 4 字节大端长度前缀 + JSON)。

        请求必须按前缀整包收: 裸 recv(1024) 在请求超过 1024 字节时被截断,
        json.loads 直接失败。trajectory_generation2 带 3 个目标位姿就 ~740 字节。
        """
        with conn:
            while not rospy.is_shutdown():
                try:
                    header = self._recv_exact(conn, 4)
                    if not header:
                        break
                    length = struct.unpack('>I', header)[0]
                    if length > 10 * 1024 * 1024:
                        print(f"Client handler error: request length {length} too large")
                        break
                    payload = self._recv_exact(conn, length)
                    if payload is None:
                        break

                    msg_str = payload.decode('utf-8')
                    try:
                        with self.exec_lock:
                            data_bytes = self.twin_callback(msg_str).encode('utf-8')
                    except Exception as e:
                        # 规划抛异常也必须回包: 客户端收不到响应会一直阻塞在 recv 上
                        rospy.logerr(f"twin_callback error: {e}")
                        data_bytes = json.dumps({"srv": "twin_inference", "value": 0,
                                                 "info": {"error": str(e)}}).encode('utf-8')
                    conn.sendall(struct.pack('>I', len(data_bytes)))
                    conn.sendall(data_bytes)
                except Exception as e:
                    print(f"Client handler error: {e}")
                    break

    def twin_callback(self, req):
        data = json.loads(req)
        srv_name = data["srv"] if "srv" in data else None
        srv_type = data["type"] if "type" in data else None
        srv_config = data["cnfg"] if "cnfg" in data else None
        if srv_type not in SUPPORTED_TWIN_SRV_TYPE or srv_config is None or srv_name!= self.srv_name:
            cprint(f"WRONG REQUEST TO TWIN INFERENCER, req:{data}\n", "red")
            return_dict = {}
            return_dict["srv"] = self.srv_name
            return_dict["value"] = 0
            return_dict["info"] = {}
            return_dict_str = json.dumps(return_dict)
            return return_dict_str
        
        try:
            struct_name = srv_config["struct"]
            target_pose = srv_config.get("target_pose")
            curr_js = srv_config["current_js"]
            cprint(f"struct_name::{struct_name}")
            cprint(f"target_pose::{target_pose}")
            cprint(f"curr_js::{curr_js}")
        except:
            cprint(f"NEED MORE CONFIG INFORMATION FOR TWIN INFERENCER, req:{data}\n")
            return_dict = {}
            return_dict["srv"] = self.srv_name
            return_dict["value"] = 0
            return_dict["info"] = {}
            return_dict_str = json.dumps(return_dict)
            return return_dict_str
        
        if struct_name != "rm65" or len(curr_js) != 6:
            return json.dumps({"srv": self.srv_name, "value": 0, "info": {}})

        if srv_type == "reachability_check":
            check, info = self.reachability_check(srv_config)
        elif srv_type == "collision_check":
            check, info = self.reachability_check(srv_config)
        elif srv_type == "IK_calculation":
            check, info = self.reachability_check(srv_config)
        elif srv_type == "trajectory_generation":
            check, traj, traj_ee, infos = self.linear_traj_generation(srv_config)
            info = {}
            info["trajectory"] = traj
            info["trajectory_ee"] = traj_ee
            info["infos"] = infos
        elif srv_type == "trajectory_generation2":
            check, traj, traj_ee, infos = self.linear_traj_generation2(srv_config)
            info = {}
            info["trajectory"] = traj
            info["trajectory_ee"] = traj_ee
            info["infos"] = infos
        else:
            cprint(f"TWIN INFERENCER DOES NOT  SUPPORT CURRENT TYPE REQUEST: {data}\n")
            check = False
            info = {}
        
        # reset robot back to current pose
        self.robot.robot_structs[struct_name].reset_by_joint_states(curr_js)
        self.robot.robot_structs[struct_name].move_joint(curr_js)
        self.step()    
            
        return_dict = {}
        return_dict["srv"] = srv_name
        return_dict["value"] = int(check)
        return_dict["info"] = info
        return_dict_str = json.dumps(return_dict)
        return return_dict_str
    
    def reachability_check(self, config):
    
        struct_name = config["struct"]
        target_pose = config["target_pose"]
        curr_js = config["current_js"]
        if struct_name != "rm65":
            rospy.logwarn(f"TWIN INFERENCER 'reachaibility' DOES NOT  SUPPORT CURRENT STRUCT: {struct_name}")
            return 0, {}
        
        # reset robot to target pose
        pos, orn = target_pose[:3], target_pose[3:]
        if self.vis:
            visualize_pose(pos, orn)

        arm = self.robot.robot_structs[struct_name]
        arm.reset_by_ee_pose_self(pos, orn, js = curr_js)

        # 同 linear_traj_generation2: 检查与取关节角都在 step 之前 (step 会把臂甩离 IK 解)
        target_js = arm.get_joint_pose()
        is_reach, delta_xyz, delta_rpy = arm.check_reach(pos, orn, xyz_threshold =0.01, rpy_threshold = 0.01)

        self.step()
        
        is_collide, collide_id = self.robot.check_collision()
        
        # update info
        check = is_reach and (not is_collide)
        info ={}
        info["is_reached"] = int(is_reach)
        info["delta_xyz"] = delta_xyz
        info["delta_rpy"] = delta_rpy
        info["target_js"] = list(target_js)
        info["is_collided"] = int(is_collide)
        info["collide_body"] = collide_id
        
        print("_________________________________________\n")
        rospy.loginfo(f"REACHABILITY CHECK RESULT:")
        for k, v in info.items():
            rospy.loginfo(f" {k}:  {v}")
        print("_________________________________________\n")
        return check,info


    def linear_traj_generation(self, config):
                
        struct_name = config["struct"]
        target_pose = config["target_pose"]
        curr_js = config["current_js"]
        interval_threshold = config["interval_threshold"] if "interval_threshold" in config else 0.05
        if struct_name != "rm65":
            rospy.logwarn(f"TWIN INFERENCER 'trajectory generation' DOES NOT  SUPPORT CURRENT STRUCT: {struct_name}")
            return 0, {}
        
        # get target
        target_pos, target_orn = target_pose[:3], target_pose[3:]
        if self.vis:
            visualize_pose(target_pos, target_orn)
                    
        # reset robot back to current pose
        self.robot.robot_structs[struct_name].reset_by_joint_states(curr_js)
        self.robot.robot_structs[struct_name].move_joint(curr_js)
        self.step()
        
        # get curr
        curr_pos, curr_orn = self.robot.robot_structs[struct_name].get_end_link_pose_self()
        # curr_R = R.from_quat(curr_orn)        
        
        delta_xyz = np.linalg.norm(target_pos - curr_pos)
        loop_num  = max(int(np.floor(delta_xyz/interval_threshold)), 1)
        intermediate_js = copy.copy(curr_js)
        trajectory  = []
        trajectory.append(curr_js)
        infos = []
        intermediate_orn_list = slerp(curr_orn, target_orn, loop_num+1)
        trajectory_ee = []
        
        for l in range(loop_num):
             # get and visualize intermediate pose
            intermediate_pos= curr_pos + (target_pos - curr_pos)*((l+1)/loop_num)
            intermediate_orn = intermediate_orn_list[l + 1]#R.slerp((l+1)/20, curr_R, target_R).as_quat() 
            if self.vis:
                visualize_pose(intermediate_pos, intermediate_orn, d = 100)
            
            # reset robot to interval pose
            arm = self.robot.robot_structs[struct_name]
            arm.reset_by_ee_pose_self(intermediate_pos, intermediate_orn, js = intermediate_js)

            # 先读构型再 step: trajectory 里存的是 intermediate_js, step 后读会被
            # 重力甩离 IK 解 (见 linear_traj_generation2 里的说明)
            intermediate_js = arm.get_joint_pose()

            is_reach, delta_xyz, delta_rpy = arm.check_reach(intermediate_pos, intermediate_orn
                                                             , xyz_threshold =0.01, rpy_threshold = 0.01)

            is_collide, collide_id = self.robot.check_collision()

            self.step()
            
            trajectory.append(list(intermediate_js))
            trajectory_ee.append(list(intermediate_pos) + list(intermediate_orn))

            info = {}
            info["intermediate_pose"] = list(intermediate_pos) + list(intermediate_orn)
            info["is_reached"] = int(is_reach)
            info["delta_xyz"] = delta_xyz
            info["delta_rpy"] = delta_rpy
            info["is_collided"] = int(is_collide)
            info["collide_body"] = collide_id
            infos.append(info)
            if is_collide:
                rospy.logwarn(f"COLLISION AT pos:{intermediate_pos}, {intermediate_orn}, CANOT GENERATE TRAJECTORY FOR TARGET:{target_pose}")
                return 0, trajectory, trajectory_ee, infos

        rospy.loginfo(f"TRAJECTORY GENERATED FOR CURENT POSE:{[curr_pos, curr_orn]} TO TARGET POSE:{[target_pos, target_orn]}")
        return 1, trajectory, trajectory_ee, infos
    
    def check_arm_joints_z_height(self, z_min_threshold=0.02):
        """
        检查机械臂各关节 link 的 Z 高度是否低于阈值 (原版实现)

        Args:
            z_min_threshold: Z高度最小阈值，默认0.02m

        Returns:
            is_safe: 是否安全 (所有关节Z > z_min_threshold)
            unsafe_links: 低于阈值的关节名称列表 [[link_name, z_value], ...]
            min_z: 最低的Z值
        """
        arm_struct = self.robot.robot_structs["rm65"]
        # 获取机械臂关节对应的link ID
        # 排除 base_link 和 arm_end_link，只检查 Link1-Link6
        arm_link_ids = arm_struct.link_ids
        arm_link_names = arm_struct.link_names

        # 过滤掉 base_link 和 end_link，只检查实际的机械臂关节 link
        links_to_skip = ["base_link", "arm_end_link", "robot_base_link"]
        check_link_ids = []
        check_link_names = []
        for i, name in enumerate(arm_link_names):
            if name not in links_to_skip and "base" not in name.lower():
                check_link_ids.append(arm_link_ids[i])
                check_link_names.append(name)

        unsafe_links = []
        min_z = float('inf')

        for link_id, link_name in zip(check_link_ids, check_link_names):
            link_state = p.getLinkState(self.robot.id, link_id)
            link_pos = link_state[0]  # 世界坐标系中的位置 [x, y, z]
            link_z = link_pos[2]
            min_z = min(min_z, link_z)

            if link_z < z_min_threshold:
                unsafe_links.append([link_name, round(link_z, 4)])

        is_safe = len(unsafe_links) == 0
        return is_safe, unsafe_links, round(min_z, 4)

    def linear_traj_generation2(self, config):

        struct_name = config["struct"]
        target_pose_list = config["target_pose"]
        curr_js = config["current_js"]
        interval_threshold = config["interval_threshold"] if "interval_threshold" in config else 0.05
        rotation_interval_rad = config.get("rotation_interval_rad", 0.15)
        joint_step_limit_rad = config.get("joint_step_limit_rad", 0.12)
        loose_constraint = config["loose_constraint"] if "loose_constraint" in config else 0
        xyz_threshold = config.get("xyz_threshold", 0.015)
        rpy_threshold = config.get("rpy_threshold", 0.05)
        z_min_threshold = config["z_min_threshold"] if "z_min_threshold" in config else 0.02  # 关节最低高度阈值
        if struct_name != "rm65":
            rospy.logwarn(f"TWIN INFERENCER 'trajectory generation' DOES NOT  SUPPORT CURRENT STRUCT: {struct_name}")
            return 0, {}
        
        # get target
        target_pos_list = []
        target_orn_list = []
        for target_pose in target_pose_list:
            target_pos, target_orn = target_pose[:3], target_pose[3:]
            target_pos_list.append(target_pos)
            target_orn_list.append(target_orn)
            if self.vis:
                visualize_pose(target_pos, target_orn)
                    
        # reset robot back to current pose
        self.robot.robot_structs[struct_name].reset_by_joint_states(curr_js)
        self.robot.robot_structs[struct_name].move_joint(curr_js)
        self.step()
        # reset robot to target_poses
        infos = []
        reaches = []
        collides = []
        z_safes = []
        js_fake = copy.deepcopy(curr_js)
        for target_pos, target_orn in zip(target_pos_list, target_orn_list):
            arm = self.robot.robot_structs[struct_name]
            arm.reset_by_ee_pose_self(target_pos, target_orn, js = js_fake)

            # 所有检查都必须在 step 之前做。move_joint 用的是 POSITION_CONTROL 的默认
            # positionGain=0.1, 撑不住重力: 实测一步 stepSimulation (4ms) 就能让 J4 走
            # 0.42rad (正好 maxvel*dt), 末端偏离 IK 解 40mm。在 step 之后再读 check_reach,
            # 读到的是"被重力甩开之后的位姿", 于是明明 IK 命中 0.1mm 的抓取候选全判不可达。
            # 可达性问的是"IK 解这个构型能不能到位", 就该在解上问, 不该问重力把臂拖到哪了。
            # z 高度检查同理 —— 漂移是往下塌的, 会把本来贴桌面的夹爪报成"插进桌面"。
            js_fake = arm.get_joint_pose()
            is_reach, delta_xyz, delta_rpy = arm.check_reach(target_pos, target_orn
                                                             , xyz_threshold =xyz_threshold, rpy_threshold = rpy_threshold)
            is_collide, collide_id = self.robot.check_collision()
            # 检查关节Z高度是否安全
            is_z_safe, unsafe_links, min_z = self.check_arm_joints_z_height(z_min_threshold)

            self.step()
            info = {}
            info["is_reached"] = int(is_reach)
            info["delta_xyz"] = delta_xyz
            info["delta_rpy"] = delta_rpy
            info["is_collided"] = int(is_collide)
            info["collide_body"] = collide_id
            info["is_z_safe"] = int(is_z_safe)
            info["unsafe_links"] = unsafe_links
            info["min_z"] = min_z
            info["js_fake"] = list(js_fake)
            info["ee_fake"] = target_pos + target_orn
            if self.vis:
                print(info)
                print("-"*20)
            infos.append(info)
            reaches.append(int(is_reach))
            collides.append(is_collide)
            z_safes.append(is_z_safe)

            # Z高度不安全则直接失败
            if not is_z_safe:
                rospy.logwarn(f"Z_HEIGHT_UNSAFE: Links {unsafe_links} below {z_min_threshold}m at target pose {target_pos}")
                return 0, [], [], infos

        if loose_constraint:
            if not reaches[-1]: # or any(is_collide == 1 for is_collide in collides):
                return 0, [], [], infos
        else:
            if any(is_reach == 0 for is_reach in reaches): # or any(is_collide == 1 for is_collide in collides):
                return 0, [], [], infos
        
        # reset robot back to current pose
        self.robot.robot_structs[struct_name].reset_by_joint_states(curr_js)
        self.robot.robot_structs[struct_name].move_joint(curr_js)
        self.step()
        
        # get curr
        curr_pos, curr_orn = self.robot.robot_structs[struct_name].get_end_link_pose_self()

        trajectory  = []
        trajectory_ee = []
        infos = []
        trajectory.append(curr_js)
        trajectory_ee.append(list(curr_pos)+list(curr_orn))
        
        fake_pos = copy.copy(curr_pos)
        fake_orn = copy.copy(curr_orn)
        fake_js = copy.copy(curr_js)
        
        for target_pos, target_orn in zip(target_pos_list, target_orn_list):
            # generate traj from fake pose to target
            target_pos_arr, target_orn_arr = np.array(target_pos), np.array(target_orn)
            delta_xyz = np.linalg.norm(target_pos_arr - fake_pos)
            # Position-only subdivision can leave a large wrist rotation in
            # one command when the grasp orientation differs from the
            # observation pose. Subdivide by both translation and the
            # shortest quaternion angle so the controller receives a gradual
            # pose change.
            quat_dot = float(np.dot(np.asarray(fake_orn), target_orn_arr))
            rotation_angle = 2.0 * np.arccos(
                np.clip(abs(quat_dot), -1.0, 1.0)
            )
            loop_num = max(
                int(np.ceil(delta_xyz / max(interval_threshold, 1e-6))),
                int(np.ceil(rotation_angle / max(rotation_interval_rad, 1e-6))),
                1,
            )
            
            intermediate_pos_list = [fake_pos + (target_pos_arr - fake_pos)*((l+1)/loop_num) for l in range(loop_num)]
            intermediate_orn_list = slerp(fake_orn, target_orn_arr, loop_num+1)[1:]
            intermediate_js = copy.copy(fake_js)
            
            for l in range(loop_num):
                # get and visualize intermediate pose
                intermediate_pos = intermediate_pos_list[l]
                intermediate_orn = intermediate_orn_list[l]
                if self.vis:
                    visualize_pose(intermediate_pos, intermediate_orn, d = 10)
                    time.sleep(0.1)
                
                # reset robot to interval pose
                joint_start_js = np.asarray(intermediate_js, dtype=float).copy()
                arm = self.robot.robot_structs[struct_name]
                arm.reset_by_ee_pose_self(intermediate_pos, intermediate_orn, js = intermediate_js)

                # 先读构型再 step。这里更要紧 —— trajectory 里存的是 intermediate_js,
                # 而这条轨迹是要发给真机的。step 之后取, 取到的是被重力甩开 0.4rad 的
                # 关节角, 等于把错的目标点发给机械臂。
                intermediate_js = arm.get_joint_pose()

                is_reach, delta_xyz, delta_rpy = arm.check_reach(intermediate_pos, intermediate_orn
                                                                 , xyz_threshold =0.01, rpy_threshold = 0.01)
                is_collide, collide_id = self.robot.check_collision()
                # 检查关节Z高度
                is_z_safe, unsafe_links, min_z = self.check_arm_joints_z_height(z_min_threshold)

                # IK can occasionally switch solution branches even when the
                # Cartesian step is small. Insert joint-space waypoints and
                # validate every inserted state before sending the path out.
                joint_delta = np.max(
                    np.abs(np.asarray(intermediate_js) - joint_start_js)
                )
                joint_steps = max(
                    int(np.ceil(joint_delta / max(joint_step_limit_rad, 1e-6))),
                    1,
                )
                for joint_step in range(1, joint_steps):
                    ratio = joint_step / float(joint_steps)
                    safe_js = (
                        joint_start_js
                        + ratio * (
                            np.asarray(intermediate_js, dtype=float)
                            - joint_start_js
                        )
                    ).tolist()
                    self.robot.robot_structs[struct_name].reset_by_joint_states(
                        safe_js
                    )
                    self.step()
                    safe_collided, safe_collide_id = self.robot.check_collision()
                    safe_z, safe_unsafe_links, safe_min_z = (
                        self.check_arm_joints_z_height(z_min_threshold)
                    )
                    if safe_collided or not safe_z:
                        rospy.logwarn(
                            "UNSAFE JOINT INTERPOLATION: "
                            f"collision={safe_collided}, z_safe={safe_z}, "
                            f"collide_body={safe_collide_id}"
                        )
                        return 0, trajectory, trajectory_ee, infos
                    trajectory.append(list(safe_js))
                    safe_pos, safe_orn = (
                        self.robot.robot_structs[struct_name]
                        .get_end_link_pose_self()
                    )
                    trajectory_ee.append(list(safe_pos) + list(safe_orn))

                self.step()


                trajectory.append(list(intermediate_js))
                trajectory_ee.append(list(intermediate_pos) + list(intermediate_orn))

                info = {}
                info["intermediate_pose"] = list(intermediate_pos) + list(intermediate_orn)
                info["is_reached"] = int(is_reach)
                info["delta_xyz"] = delta_xyz
                info["delta_rpy"] = delta_rpy
                info["is_collided"] = int(is_collide)
                info["collide_body"] = collide_id
                info["is_z_safe"] = int(is_z_safe)
                info["unsafe_links"] = unsafe_links
                info["min_z"] = min_z
                if self.vis:
                    print(info)
                    print("-"*20)
                infos.append(info)

                print(info)

                if is_collide:
                    rospy.logwarn(f"COLLISION AT pos:{intermediate_pos}, {intermediate_orn}, CANOT GENERATE TRAJECTORY FOR TARGET:{target_pose}")
                    return 0, trajectory, trajectory_ee, infos

                if not is_z_safe:
                    rospy.logwarn(f"Z_HEIGHT_UNSAFE: Links {unsafe_links} below {z_min_threshold}m at pos:{intermediate_pos}")
                    return 0, trajectory, trajectory_ee, infos
            
            fake_pos = np.array(target_pos)
            fake_orn = np.array(target_orn)
            fake_js = copy.copy(intermediate_js)
            

        rospy.loginfo(f"TRAJECTORY GENERATED FOR CURENT POSE:{[curr_pos, curr_orn]} TO TARGET POSE:{[target_pos, target_orn]}")
        return 1, trajectory, trajectory_ee, infos

    def test_client(self):
        
        data = {
            "srv":self.srv_name,
            "type":"trajectory_generation2",
            "cnfg":{
                "target_pose":[
                    [0.6130359164338649, -0.26390629817473416, 0.9572628420278162, 0.5423872870142016, 0.7882424147151591, 0.21071606926629194, 0.20022153900705178],
                    [0.6130359164338649, -0.18390629817473416, 0.9572628420278162, 0.5423872870142016, 0.7882424147151591, 0.21071606926629194, 0.20022153900705178]        
                               ],
                "current_js":[-1.204, -8.815, 57.494, -3.33, 113.799, -0.012],
                "struct":"rm65"
            }
        }
        req = json.dumps(data)
        rsp = self.node_list.client_service_request(self.srv_name, StringRequest, req)
        rsp = json.loads(rsp.response)
        import pprint
        pprint.pprint(rsp)
        return True
    def start_test(self):
        self.test_thread = threading.Thread(target=self.test_client)
        self.test_thread.start()
        return self.test_thread



def on_press(key):
    global kill_program
    try:
        if key.char == '`':  # Check if 's' is pressed
            print("PROGRAM KILLED BY HUMAN")
            kill_program = True  # Set flag to stop the loop
    except AttributeError:
        pass




if __name__ == '__main__':
    from pynput import keyboard

    world = TwinTest2(vis=True)
    world.reset()

    key_board_listener = keyboard.Listener(on_press=on_press)
    key_board_listener.start()
    
    kill_program = False
    count = 0
    while not kill_program:
        count+= 1
        robot_state = world.step()
         
        
        # print(count)
        # if count == 50:
        #     world.start_test()
        
        # if count == 100:
        #     time.sleep(5)
        #     break