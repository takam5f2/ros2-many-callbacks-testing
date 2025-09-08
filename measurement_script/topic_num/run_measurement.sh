#!/bin/bash
# set -x

# --- 引数のチェック ---
# 引数として、ノード数、計測モードを↓のように指定する
# $0 <ノード数> <ノードあたりのtopic数>
if [ $# -lt 2 ]; then
    echo "Usage: $0 <node_num> <topic_num>"
    exit 1
fi
# ノード数の引数をチェック
if ! [[ $1 =~ ^[0-9]+$ ]]; then
    echo "Error: ノード数は整数で指定してください。"
    exit 1
fi
NODE_NUM="$1"
# トピック周波数の引数をチェック
if ! [[ $2 =~ ^[0-9]+$ ]]; then
    echo "Error: トピック数は整数で指定してください。"
    exit 1
fi
TOPIC_NUM="$2"

if [ $(cat /proc/sys/kernel/perf_event_paranoid) != -1 ] || [ $(cat /proc/sys/kernel/kptr_restrict) != 0 ] || [ $(cat /proc/sys/kernel/yama/ptrace_scope) != 0 ]; then
  sudo sh -c 'echo -1 > /proc/sys/kernel/perf_event_paranoid && echo 0 > /proc/sys/kernel/kptr_restrict && echo 0 > /proc/sys/kernel/yama/ptrace_scope'
fi
ulimit -n 65536

mkdir -p result

# --- 変数設定 ---
# 第1引数をNODE_NUMとして使用する
RES_DIR="result/vtune_hs_system_wide_node_num_${NODE_NUM}_topic_num_per_node_${TOPIC_FREQUENCY}"

# 既存のディレクトリがあれば削除するか、ユーザーに確認する（オプション）
if [ -d "$RES_DIR" ]; then
    echo "警告: 結果ディレクトリ '$RES_DIR' は既に存在します。上書きします。"
    rm -rf "$RES_DIR"
fi

echo ">>> 設定を生成します (NODE_NUM: ${NODE_NUM})..."
python3 create_config_yaml_for_specified_node_num_and_topic_num.py $NODE_NUM $TOPIC_NUM

echo "ROS2 launchファイルを実行します..."
# ROS2のlaunchファイルをバックグラウンドで実行
source ../../install/setup.bash
ros2 launch simple_node simple_listener_talker_launch.py node_config_file:=config.yaml &

# launchファイルのプロセスIDを取得
ROS2_PID=$!

# 起動が完了するまで待機しつつ、CPU使用率のログを取る
python3 wait_until_cpu_usage_got_low.py > result/cpu_usage_log_node_num_${NODE_NUM}_topic_num_per_node_${TOPIC_FREQUENCY}.csv

# 起動したROS2のプロセスを終了
pkill -g ${ROS2_PID}

# simple_listenerとsimple_talkerプロセスが完全に終了するまで待機
echo "ROS2のプロセスを終了中..."
pkill -f simple_node/lib/simple_node
sleep 1
while true; do
    node_count=$(pgrep -f simple_node/lib/simple_node | wc -l)
    
    if [ $node_count -eq 0 ]; then
        echo "全てのROS2プロセスが終了しました。"
        break
    fi
    
    echo "残りのプロセス: node_count=$node_count. 再度killします..."
    pkill -f simple_node/lib/simple_node
    sleep 10
done

echo "ROS2のプロセスを終了しました。"
