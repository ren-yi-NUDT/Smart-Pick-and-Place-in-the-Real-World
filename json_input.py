import json
import sys
from typing import Optional, Dict, Any

"""
JSON 输入格式说明
================

输入通过 stdin 传递，每行一个 JSON 命令。

必填字段:
    - object: 要抓取的物体名称，可以是任意 YOLO-World 支持的类别名称
              支持逗号分隔的多类别（如 "orange,lemon"）
    - container: 放置的容器名称，可以是任意 YOLO-World 支持的类别名称

可选字段:
    - direction: 空间方向提示 (left/right/middle/front/back)
                 注：当前版本暂未实现空间选择功能

示例:
    {"object": "orange", "container": "pink plate"}
    {"object": "apple,fruit", "container": "bowl"}
    {"object": "bottle", "container": "white box", "direction": "left"}

注意:
    - object 和 container 的值会直接传给 YOLO-World 进行检测
    - 建议使用 YOLO-World 能识别的类别名称以获得最佳效果
"""


class JsonInputParser:
    """从stdin读取JSON命令并解析"""

    def __init__(self):
        pass

    def read_from_stdin(self) -> Optional[Dict[str, Any]]:
        """从stdin读取一行JSON输入"""
        try:
            line = sys.stdin.readline()
            if not line:
                return None
            line = line.strip()
            if not line:
                return None
            return json.loads(line)
        except json.JSONDecodeError as e:
            print(f"JSON解析错误: {e}")
            return None
        except Exception as e:
            print(f"读取输入错误: {e}")
            return None

    def parse(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        解析JSON数据，直接提取物体和容器信息（不做词汇表映射）

        输入格式:
        {
            "object": "orange",        # 必填: 抓取的物体名称
            "container": "pink plate", # 必填: 放置的容器名称
            "direction": "left"        # 可选: 方位（暂未实现）
        }

        输出格式:
        {
            "object": "orange",        # 物体名称（直接使用输入值）
            "container": "pink plate", # 容器名称（直接使用输入值）
            "direction": "left",       # 方位（直接使用输入值）
            "original": {...}          # 原始输入
        }
        """
        return {
            "object": data.get("object"),
            "container": data.get("container"),
            "direction": data.get("direction"),
            "original": data
        }

    def get_command(self) -> Optional[Dict[str, Any]]:
        """读取并解析一条命令"""
        data = self.read_from_stdin()
        if data is None:
            return None
        return self.parse(data)


if __name__ == "__main__":
    # 测试代码
    parser = JsonInputParser()

    print("请输入JSON命令 (Ctrl+D 结束):")
    print('示例: {"object": "orange", "container": "pink plate"}')
    print('      {"object": "apple,fruit", "container": "bowl", "direction": "left"}')

    while True:
        try:
            result = parser.get_command()
            if result is None:
                break
            print(f"解析结果: {result}")
        except KeyboardInterrupt:
            break

    print("\n结束")
