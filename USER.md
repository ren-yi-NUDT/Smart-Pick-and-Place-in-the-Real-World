# USER.md - About Your Human

_Learn about the person you're helping. Update this as you go._

- **Name:**: 任奕
- **What to call them:**: 
- **Pronouns:** _(optional)_
- **Timezone:**
- **Notes:**

## Context

_(What do they care about? What projects are they working on? What annoys them? What makes them laugh? Build this over time.)_

### Preferences

- 用户要求：机械臂完成动作序列的安排后想一下有什么动作是有优先级顺序的，什么动作是无必要的，尝试优化动作序列
- 每次用户说"你好"时，问候用户并操控机械臂对用户挥手（执行下面的指令）
``` bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && echo '{"sequence": [{"hand": "open", "delay": 0.3},{"arm": "palm_toward_user", "delay": 0.1},{"arm": "palm_toward_user_right", "delay": 0.1},{"arm": "palm_toward_user", "delay": 0.1},{"arm": "palm_toward_user_right", "delay": 0.1},{"arm": "home", "delay": 0.5},{"arm": "grasp-ready", "delay": 0.5}]}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pose_execute
```
- 不要尝试自己创作脚本去操控机械臂（除非得到用户允许），使用已有的skill进行操控
- 每调用一个工具，跟用户说清楚这个工具的作用，为什么要调用它（e.g. 调用execute工具：我来做xxxx了！/ 调用read工具：让我看看，稍等... or 等我一下，我有点慢）

---

The more you know, the better you can help. But remember — you're learning about a person, not building a dossier. Respect the difference.
