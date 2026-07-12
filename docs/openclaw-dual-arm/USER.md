# USER.md - About Your Human

_Learn about the person you're helping. Update this as you go._

- **Name:**: 任奕
- **What to call them:**: 
- **Pronouns:** _(optional)_
- **Timezone:** Asia/Shanghai
- **Notes:**

## Context

任奕在做大创项目"基于虚实结合双重推理架构的桌面级智能机械臂平台"。当前在 `Double-arm-on-desk` 分支开发**双臂交接**能力——把左臂（灵巧手）抓的物体交给右臂（夹爪），或反向。已实测可用的双臂 4 步序列在 MEMORY 2026-07-05。

### Preferences

- 用户要求：机械臂完成动作序列的安排后想一下有什么动作是有优先级顺序的，什么动作是无必要的，尝试优化动作序列
- 每次用户说"你好"时，问候用户并操控机械臂对用户挥手（执行下面的指令）
```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && conda activate anygrasp && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && echo '{"sequence": [{"hand": "open", "delay": 0.3},{"arm": "palm_toward_user", "delay": 0.1},{"arm": "palm_toward_user_right", "delay": 0.1},{"arm": "palm_toward_user", "delay": 0.1},{"arm": "palm_toward_user_right", "delay": 0.1},{"arm": "home", "delay": 0.5},{"arm": "grasp-ready", "delay": 0.5}]}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pose_execute
```
- **不要尝试自己创作脚本去操控机械臂**（除非得到用户允许），使用代码库里已有的 skill 组合实现需求
- **代码库即字典**：用户提需求时，先查 `skills/` + `COMMANDS.md` + `CLAUDE.md` + `recorded_*` 看现有 primitives 能拼出什么，**不要先入为主说"做不到"**
- 成功解决的新 skill 组合配方，追加到 `skills/skill-extraction/SKILL.md` 的"组合配方"节
- 用户很没有耐心，每次要做耗时的操作要尽可能的通过输出中间结果给用户看等方式分散用户注意力，让他的等待变得有趣而不是让他觉得烦

---

The more you know, the better you can help. But remember — you're learning about a person, not building a dossier. Respect the difference.
