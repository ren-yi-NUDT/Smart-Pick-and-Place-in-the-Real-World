# 本rep为大创项目 `基于虚实结合双重推理架构的桌面级智能机械臂平台` pick-and-place代码仓库


1.ping 192.168.1.19   机械臂的IP
2.ping 192.168.11.210  灵巧手的IP
3.激活anygrasp环境： conda activate anygrasp
4.运行 start.bash (这个脚本是安全的，放心执行)


主要看scrip3窗口

## JSON输入格式

程序从标准输入(stdin)读取JSON命令，格式如下：

```json
{"object": "orange", "container": "pink_plate", "direction": "left"}
```

**必填字段：**
- `object`: 要抓取的物体类型
- `container`: 放置的容器类型

**可选字段：**
- `direction`: 方位 (left/right/middle/front/back)

**支持的物体类型：**
- orange, lemon, pear, bottle, carambola, starfruit, bitter_gourd, carrot, eggplant, peach

**支持的容器类型：**
- pink_plate, green_bowl, white_box, bowl, plate, box

**使用示例：**
```bash
# 方式1：管道输入
echo '{"object": "orange", "container": "pink_plate"}' | python3 planner.py

# 方式2：文件输入
cat command.json | python3 planner.py

# 方式3：交互式输入
python3 planner.py
# 然后输入JSON并回车
```
