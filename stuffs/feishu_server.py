#!/usr/bin/env python3
import http.server
import json
import threading
import time
import logging
from http.server import BaseHTTPRequestHandler, HTTPServer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class FeishuBotHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length)
        
        try:
            data = json.loads(post_data.decode('utf-8'))
            logger.info(f"收到消息: {data}")
            
            # 处理消息类型
            if data.get('type') == 'url_verification':
                # 飞书验证
                challenge = data.get('challenge')
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"challenge": challenge}
                self.wfile.write(json.dumps(response).encode())
                logger.info("已验证URL")
                
            elif data.get('type') == 'event':
                # 处理实际事件
                event_type = data.get('event', {}).get('type')
                logger.info(f"事件类型: {event_type}")
                
                # 如果是消息事件，回复
                if event_type == 'im.message.receive_v1':
                    message = data.get('event', {}).get('message', {})
                    text_content = message.get('text', '')
                    msg_id = message.get('message_id')
                    chat_id = message.get('chat_id')
                    
                    logger.info(f"收到消息: {text_content}")
                    
                    # 简单回复
                    reply_data = {
                        "msg_type": "interactive",
                        "card": {
                            "header": {
                                "title": {
                                    "tag": "plain_text",
                                    "content": "机器人收到消息"
                                }
                            },
                            "elements": [{
                                "tag": "div",
                                "text": {
                                    "tag": "plain_text",
                                    "content": f"你发送了: {text_content}"
                                }
                            }, {
                                "tag": "action",
                                "actions": [{
                                    "tag": "button",
                                    "text": {
                                        "tag": "plain_text",
                                        "content": "测试机械臂"
                                    },
                                    "type": "primary",
                                    "url": "http://81.28.13.22:8080/test"
                                }]
                            }]
                        }
                    }
                    
                    # 发送回复
                    self.send_reply(chat_id, msg_id, reply_data)
                    
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

    def do_GET(self):
        if self.path == '/test':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html_content = """<html><head><title>机械臂测试</title></head><body><h1>机械臂控制系统</h1><p>状态: 正常运行</p><button onclick="fetch('/status')">获取状态</button></body></html>"""
            self.wfile.write(html_content.encode('utf-8'))
        else:
            self.send_response(404)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'Not Found')

    def send_reply(self, chat_id, msg_id, content):
        """发送回复消息"""
        token = "t-g10439gFLEESJXUIOYDTI3MKQIZS3U6LTXJUA5HA"
        url = f"https://open.feishu.cn/open-apis/im/v1/messages?receive_id={chat_id}&receive_type=chat"
        
        payload = {
            "msg_type": "interactive",
            "card": content["card"]
        }
        
        import subprocess
        try:
            curl_cmd = f'''curl -X POST "{url}" -H "Content-Type: application/json" -H "Authorization: Bearer {token}" -d '{json.dumps(payload)}' '''
            subprocess.run(curl_cmd, shell=True, check=True, capture_output=True)
            logger.info(f"已发送回复到 {chat_id}")
        except subprocess.CalledProcessError as e:
            logger.error(f"发送回复失败: {e}")

if __name__ == "__main__":
    server = HTTPServer(('0.0.0.0', 8080), FeishuBotHandler)
    logger.info("HTTP服务器启动在 0.0.0.0:8080")
    logger.info("公网地址: http://81.28.13.22:8080")
    server.serve_forever()