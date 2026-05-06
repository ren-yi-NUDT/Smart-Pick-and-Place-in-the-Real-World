#!/bin/bash

# 飞书机器人消息发送权限申请

APP_ID="cli_a9243de94ef91bc7"

# 需要申请的消息发送权限
PERMISSIONS="im:message:send_as_bot,im:message,im:message:send"

# 生成权限申请链接
AUTH_URL="https://open.feishu.cn/app/${APP_ID}/auth?q=${PERMISSIONS}&op_from=openapi&token_type=tenant"

echo "飞书机器人消息发送权限申请链接："
echo "$AUTH_URL"
echo ""
echo "请点击上述链接申请以下权限："
echo "- im:message:send_as_bot - 以应用身份发消息"
echo "- im:message - 消息管理"
echo "- im:message:send - 发送消息"
echo ""
echo "申请完成后，回到这里继续配置。"