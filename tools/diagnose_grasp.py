#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取为什么没夹住
================
一次抓取失败, 能怪的东西有三个, 而且现场长得一模一样 —— 夹爪在果子旁边空合一下:

  1. **地图错了**   地图记的世界坐标不在物体上
  2. **候选错了**   AnyGrasp 选中的候选不落在物体上 (框过滤只保证"落在框里",
                    框比物体大, 果子长条形的时候框里一半是桌面)
  3. **执行错了**   候选是对的, 但候选 -> 世界坐标 -> 孪生轨迹 -> 关节 这一串丢了精度

这个工具在**同一个观察位姿**上把三段分开量, 全画在一张图上:

  * 洋红十字   = 地图记的那个世界坐标投影回画面
  * 灰点       = AnyGrasp 的全部候选
  * 蓝圈       = 通过 YOLO 框过滤的候选
  * 红圈       = 其中分数最高、真正被执行的那一个

十字压在果子上而红圈在桌面 -> 候选错了。红圈也压在果子上 -> 执行那一串丢了精度。
两个都不在果子上 -> 地图错了。

用法:
    conda activate anygrasp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib

    python3 tools/diagnose_grasp.py cucumber                 # 默认 grasp2
    python3 tools/diagnose_grasp.py cucumber --pose grasp3
    python3 tools/diagnose_grasp.py cucumber --no-move       # 臂已经在那个位姿上了
"""

import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dependence"))   # anygrasp_sdk 在这儿

from core import scene as scene_map                                    # noqa: E402
from core.transforms import _get_intrinsics, transform_world_to_camera  # noqa: E402


def project(world_xyz, T_base_to_cam):
    """世界坐标 -> 相机像素。返回 (u, v, 相机系坐标)。"""
    p = np.eye(4)
    p[:3, 3] = world_xyz
    pc = transform_world_to_camera(p, T_base_to_cam)[:3, 3]
    if pc[2] <= 1e-6:
        return None, None, pc
    fx, fy, cx, cy = _get_intrinsics("right")
    return int(round(fx * pc[0] / pc[2] + cx)), int(round(fy * pc[1] / pc[2] + cy)), pc


def main():
    ap = argparse.ArgumentParser(description="拆开量一次抓取失败")
    ap.add_argument("object", help="目标物体名 (地图里的名字)")
    ap.add_argument("--pose", default="grasp2", help="观察位姿 (默认 grasp2)")
    ap.add_argument("--no-move", action="store_true", help="不动臂, 假设它已经在该位姿")
    ap.add_argument("--go", action="store_true", help="真的走到抓取位姿再拍一张 (不合夹爪)")
    ap.add_argument("--ttl", type=float, default=1e9, help="地图过期秒数 (默认不过期)")
    args = ap.parse_args()

    import cv2
    from skills.pick_and_place import PickAndPlaceSkill

    scene = scene_map.load_fresh(args.ttl)
    if scene is None:
        print("[map] 没有可用的场景地图")
        return 1
    tgt = scene_map.find(scene, args.object)
    if tgt is None:
        print(f"[map] 地图里没有 {args.object}; 有: "
              + ", ".join(f"{o['color']} {o['name']}" for o in scene["objects"]))
        return 1
    map_world = np.array(tgt["world"])
    print(f"[map] {tgt['color']} {tgt['name']} conf={tgt['conf']:.2f} "
          f"world=({map_world[0]:+.3f},{map_world[1]:+.3f},{map_world[2]:+.3f}) "
          f"扫描于 {scene['scanned_at']} seen_from={tgt['seen_from']}")

    s = PickAndPlaceSkill()
    if not args.no_move:
        s.control_arm(pose_type="grasp1", speed=30)
        s.control_arm(pose_type=args.pose, speed=30)
    rgb, depth = s.get_camera_obs()
    s.save_current_transformation()

    # ---- 1. 地图点回到画面 ----
    u, v, pc = project(map_world, s.T_base_to_cam)
    print(f"[map] 相机系=({pc[0]:+.3f},{pc[1]:+.3f},{pc[2]:+.3f})m  像素=({u},{v})"
          f"  深度图上该点深度={depth[v, u] if u is not None and 0 <= v < depth.shape[0] and 0 <= u < depth.shape[1] else -1}")

    # ---- 2. AnyGrasp 候选 ----
    anygrasp_pose = s.perception.detect_grasps(
        rgb, depth, class_name=args.object,
        raw_depth=getattr(s.camera, "raw_depth", None))
    if not anygrasp_pose:
        print("[anygrasp] 没有候选")
        return 1
    filt = s.perception.filter_grasps_by_detection(
        anygrasp_pose, rgb, class_name=args.object, vis=False)
    print(f"[anygrasp] 候选 {len(anygrasp_pose)} 个, 框内 {len(filt)} 个")
    if not filt:
        return 1

    # 抓取管线同款换算: 相机系候选 -> 世界系手爪位姿 (含左臂夹爪约定)。
    cands = s.grasp_pipeline._build_grasp_candidates(filt)
    cand = s.grasp_pipeline._score_grasp_candidates(cands)[0]
    top_world = cand["pose"][:3, 3]
    d = np.linalg.norm(top_world - map_world)
    print(f"[exec] 最高分候选世界坐标=({top_world[0]:+.3f},{top_world[1]:+.3f},{top_world[2]:+.3f})"
          f"  离地图点 {d * 1000:.0f} mm")

    # ---- 3. 真正发给孪生的那个点 (管线同款三段目标: 高预抓取/低预抓取/执行) ----
    cnfg = s.grasp_pipeline._grasp_target_config(
        cand, s.config.get_pose(args.pose))
    tip = np.array(cnfg["target_pose"][-1][:3])
    print(f"[exec] 发给孪生的末端坐标=({tip[0]:+.3f},{tip[1]:+.3f},{tip[2]:+.3f})"
          f"  离地图点 {np.linalg.norm(tip - map_world) * 1000:.0f} mm")
    print(f"[exec] 工具偏置 手爪->末端 = {np.linalg.norm(s.T_hand_effector_to_arm_endlink[:3, 3]) * 1000:.0f} mm")

    # ---- 4. 孪生拿这个点算出什么 ----
    # 孪生报的末端坐标如果等于上面那个 tip, 说明"候选 -> 世界 -> 孪生"这一段没丢精度,
    # 臂是真的被指到果子上了; 不等就是这一段的问题。
    rsp = s.send_cmd_twin(s.twin, {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
    info = rsp.get("info") or {}
    ee = np.array(info.get("trajectory_ee", []))
    print(f"[twin] value={rsp.get('value')} is_z_safe={info.get('is_z_safe')}")
    if len(ee):
        last = ee[-1]
        print(f"[twin] 轨迹末端=({last[0]:+.3f},{last[1]:+.3f},{last[2]:+.3f})"
              f"  离请求 {np.linalg.norm(np.array(last[:3]) - tip) * 1000:.0f} mm")

    # ---- 画图 ----
    img = np.ascontiguousarray(rgb.copy())
    fx, fy, cx, cy = _get_intrinsics("right")
    for i, g in enumerate(anygrasp_pose):
        t = np.array(g["trans"], dtype=float)
        if t[2] <= 1e-6:
            continue
        gu, gv = int(fx * t[0] / t[2] + cx), int(fy * t[1] / t[2] + cy)
        cv2.circle(img, (gu, gv), 3, (140, 140, 140), -1)
    for g in filt:
        t = np.array(g["trans"], dtype=float)
        gu, gv = int(fx * t[0] / t[2] + cx), int(fy * t[1] / t[2] + cy)
        cv2.circle(img, (gu, gv), 9, (255, 128, 0), 2)
    t = np.array(filt[int(np.argmax([g["score"] for g in filt]))]["trans"], dtype=float)
    gu, gv = int(fx * t[0] / t[2] + cx), int(fy * t[1] / t[2] + cy)
    cv2.circle(img, (gu, gv), 13, (0, 0, 255), 3)
    if u is not None:
        cv2.drawMarker(img, (u, v), (255, 0, 255), cv2.MARKER_CROSS, 34, 3)
    out = os.path.join(ROOT, "log", f"diagnose_{args.object}_{args.pose}.png")
    cv2.imwrite(out, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    print(f"[img] {out}  洋红=地图点 蓝=框内候选 红=执行的那个 灰=全部候选")

    # ---- 5. 可选: 真的走到抓取位姿上拍一张 (不合夹爪) ----
    # 这是唯一能直接看见"夹爪到底停在果子哪儿"的办法: 相机跟着手走, 到了抓取位姿,
    # 画面下半部分就是两根手指。地图点也投影上去, 看它有没有落在两指中间。
    if args.go and len(ee):
        traj = np.array(info["trajectory"]) / np.pi * 180
        print(f"[go] 走 {len(traj)} 个路点到最后一位 (抓取位姿), 不合夹爪")
        s.control_arm(trajectory=traj, speed=20)
        rgb2, _ = s.get_camera_obs()
        s.save_current_transformation()
        img2 = np.ascontiguousarray(rgb2.copy())
        u2, v2, pc2 = project(map_world, s.T_base_to_cam)
        if u2 is not None and 0 <= v2 < img2.shape[0] and 0 <= u2 < img2.shape[1]:
            cv2.drawMarker(img2, (u2, v2), (255, 0, 255), cv2.MARKER_CROSS, 34, 3)
            print(f"[go] 抓取位姿上地图点投影=({u2},{v2}) 相机系=({pc2[0]:+.3f},{pc2[1]:+.3f},{pc2[2]:+.3f})")
        else:
            print(f"[go] 抓取位姿上地图点不在画面里 (投影=({u2},{v2})) 相机系={np.round(pc2,3)}")
        out2 = os.path.join(ROOT, "log", f"diagnose_{args.object}_{args.pose}_atgrasp.png")
        cv2.imwrite(out2, cv2.cvtColor(img2, cv2.COLOR_RGB2BGR))
        print(f"[img] {out2}")

    if not args.no_move:
        s.control_arm(pose_type="grasp1", speed=30)
    return 0


if __name__ == "__main__":
    sys.exit(main())
