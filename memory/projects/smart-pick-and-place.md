# Smart Pick and Place 项目

## 基本信息
- **路径**: `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World`
- **类型**: 语音控制智能抓取系统
- **状态**: ✅ 运行测试通过 (2026-03-09)

## 快速启动
```bash
bash start.bash  # 自动在2个终端启动 ROS服务(8000,8010) + Twin IK(8020)

# 执行技能
conda activate anygrasp && python run_skill.py <skill_name>
```

## 架构
- ROS层 (8000/8010) → Twin层 (8020) → 应用层 (planner.py)
- 详见: `代码库/openclaw/EXECUTION_GUIDE.md`

## 最近日志
- 2026-03-09: 完成首次完整运行测试，成功抓取橘子放置到粉色盘子
