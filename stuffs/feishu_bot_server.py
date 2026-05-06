#!/usr/bin/env python3
import http.server
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class FeishuBotHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length)
        
        try:
            data = json.loads(post_data.decode('utf-8'))
            logger.info(f"收到飞书消息: {json.dumps(data, indent=2)}")
            
            # 处理消息类型
            if data.get('type') == 'url_verification':
                # 飞书验证
                challenge = data.get('challenge')
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"challenge": challenge}
                self.wfile.write(json.dumps(response).encode())
                logger.info(f"URL验证成功: {challenge}")
                
            elif data.get('type') == 'event':
                # 处理实际事件
                event = data.get('event', {})
                event_type = event.get('type')
                logger.info(f"事件类型: {event_type}")
                
                # 如果是消息事件，回复
                if event_type == 'im.message.receive_v1':
                    message = event.get('message', {})
                    text_content = message.get('text', '')
                    msg_id = message.get('message_id')
                    chat_id = message.get('chat_id')
                    sender_id = message.get('sender', {}).get('sender_id', {}).get('open_id')
                    
                    logger.info(f"收到消息: {text_content} (来自: {sender_id}, 聊天ID: {chat_id})")
                    
                    # 构建回复内容
                    reply_content = {
                        "msg_type": "text",
                        "content": json.dumps({"text": f"🤖 机器人收到你的消息: {text_content}\n\n现在可以响应机器人了！你可以发送命令来控制机械臂。"})
                    }
                    
                    # 发送回复
                    self.send_reply(chat_id, reply_content)
                    
            else:
                logger.info(f"未知事件类型: {data}")
                
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"code": 0}')
            
        except Exception as e:
            logger.error(f"处理消息时出错: {e}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"code": 500, "error": "Internal error"}')

    def send_reply(self, chat_id, content):
        """发送回复消息"""
        token = "t-g10439gFLEESJXUIOYDTI3MKQIZS3U6LTXJUA5HA"
        
        # 飞书消息发送API (使用正确的端点)
        url = "https://open.feishu.cn/open-apis/im/v1/messages"
        
        # 构建请求参数
        params = {
            "receive_id": chat_id,
            "receive_id_type": "chat_id"
        }
        
        # 构建请求体
        payload = {
            "msg_type": content["msg_type"],
            "content": content["content"]
        }
        
        try:
            import subprocess
            # 构建带查询参数的URL
            full_url = f"{url}?receive_id={chat_id}&receive_id_type=chat_id"
            curl_cmd = f'''curl -X POST "{full_url}" -H "Content-Type: application/json" -H "Authorization: Bearer {token}" -d '{json.dumps(payload)}' '''
            result = subprocess.run(curl_cmd, shell=True, capture_output=True, text=True)
            logger.info(f"发送回复结果: {result.stdout}")
            if result.stderr:
                logger.error(f"发送回复错误: {result.stderr}")
        except Exception as e:
            logger.error(f"发送回复失败: {e}")

    def do_GET(self):
        if self.path == '/health':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"status": "ok", "message": "Feishu bot server is running"}
            self.wfile.write(json.dumps(response).encode())
        else:
            self.send_response(404)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'Not Found')

if __name__ == "__main__":
    print("=== 飞书机器人服务器启动 ===")
    print("服务器地址: http://81.28.13.22:8080")
    print("本地地址: http://127.0.0.1:8080")
    print("健康检查: http://127.0.0.1:8080/health")
    print()
    print("请按照以下步骤配置机器人:")
    print("1. 登录飞书开放平台: https://open.feishu.cn")
    print("2. 进入应用管理 -> 你的应用 -> 事件订阅")
    print("3. 添加事件订阅:")
    print("   - 事件类型: im.message.receive_v1")
    print("   - 推送地址: http://81.28.13.22:8080")
    print("4. 保存配置")
    print()
    
    # 启动服务器
    server = HTTPServer(('0.0.0.0', 8080), FeishuBotHandler)
    logger.info("HTTP服务器启动在 0.0.0.0:8080")
    
    try:
        logger.info("开始监听消息...")
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("服务器停止")
        server.shutdown()