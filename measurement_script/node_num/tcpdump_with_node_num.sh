#!/bin/bash
set -x

# --- 引数のチェック ---
# 引数として、ノード数、計測モードを↓のように指定する
# $0 <ノード数>
if [ $# -lt 1 ]; then
    echo "Usage: $0 <node_num>"
    exit 1
fi
# ノード数の引数をチェック
if ! [[ $1 =~ ^[0-9]+$ ]]; then
    echo "Error: ノード数は整数で指定してください。"
    exit 1
fi
NODE_NUM="$1"


mkdir -p result

# --- 変数設定 ---
# 第1引数をNODE_NUMとして使用する
RES_FILE="result/tcpdump_${NODE_NUM}.pcap"

# 既存のファイルがあれば削除する
if [ -f "$RES_FILE" ]; then
    echo "警告: 結果ファイル '$RES_FILE' は既に存在します。上書きします。"
    rm -f "$RES_FILE"
fi

echo ">>> 設定を生成します (NODE_NUM: ${NODE_NUM})..."
python3 create_config_yaml_for_specified_node_num.py $NODE_NUM pub_sub
source ../../install/setup.bash

echo ">>> TCPダンプを開始します (ノード数: ${NODE_NUM})..."
# TCPダンプをバックグラウンドで実行
sudo echo ">>> TCPダンプを開始します (ノード数: ${NODE_NUM})..."
sudo tcpdump -i lo -w "$RES_FILE" &

# TCPダンプのプロセスIDを取得
TCPDUMP_PID=$!

sleep 1  # 少し待機してからノードを起動


echo "ROS2 launchファイルを実行します..."
# ROS2のlaunchファイルをバックグラウンドで実行
ros2 launch simple_node simple_listener_talker_launch.py node_config_file:=config.yaml &

# launchファイルのプロセスIDを取得
ROS2_PID=$!

# 30秒間待機
echo ">>> 60秒間待機します..."
sleep 60

echo ">>> TCPダンプを停止します..."
# TCPダンプを停止
sudo kill $TCPDUMP_PID

echo ">>> TCPダンプの結果を保存しました: $RES_FILE"

echo ">>> ROS2 launchファイルのプロセスを停止します..."
# ROS2のプロセスを停止
sudo kill $ROS2_PID
sleep 2
pkill -f simple_node/lib/simple_node
