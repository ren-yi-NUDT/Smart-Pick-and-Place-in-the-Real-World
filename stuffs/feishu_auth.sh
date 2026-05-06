#!/bin/bash

# 飞书机器人权限申请链接
APP_ID="cli_a9243de94ef91bc7"

# 需要申请的权限
PERMISSIONS="im:chat,im:chat:create,im:chat:create_by_user"

# 生成权限申请链接
AUTH_URL="https://open.feishu.cn/app/${APP_ID}/auth?q=${PERMISSIONS}&op_from=openapi&token_type=tenant"

echo "飞书机器人权限申请链接："
echo "$AUTH_URL"
echo ""
echo "请点击上述链接申请以下权限："
echo "- im:chat"
echo "- im:chat:create" 
echo "- im:chat:create_by_user"
echo ""
echo "申请完成后，回到这里继续配置。"