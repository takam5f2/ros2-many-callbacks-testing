#!/bin/bash
# set -x

# --- 引数のチェック ---
# 引数として、ノード数、1component containerあたりのノード数を↓のように指定する
# $0 <ノード数> <ノード数/コンポーネント>
if [ $# -lt 2 ]; then
    echo "Usage: $0 <total_node_num> <node_num_per_component>"
    exit 1
fi
# ノード数の引数をチェック
if ! [[ $1 =~ ^[0-9]+$ ]]; then
    echo "Error: ノード数は整数で指定してください。"
    exit 1
fi
NODE_NUM="$1"
# ノード数/コンポーネントの引数をチェック
if ! [[ $2 =~ ^[0-9]+$ ]]; then
    echo "Error: ノード数/コンポーネントは整数で指定してください。"
    exit 1
fi
NODE_NUM_PER_COMPONENT="$2"

if [ $(cat /proc/sys/kernel/perf_event_paranoid) != -1 ] || [ $(cat /proc/sys/kernel/kptr_restrict) != 0 ] || [ $(cat /proc/sys/kernel/yama/ptrace_scope) != 0 ]; then
  sudo sh -c 'echo -1 > /proc/sys/kernel/perf_event_paranoid && echo 0 > /proc/sys/kernel/kptr_restrict && echo 0 > /proc/sys/kernel/yama/ptrace_scope'
fi
ulimit -n 65536

mkdir -p result

# --- 変数設定 ---
# 第1引数をNODE_NUMとして使用する
RES_DIR="result/vtune_hs_system_wide_total_node_num_${NODE_NUM}_per_component_${NODE_NUM_PER_COMPONENT}"

# 既存のディレクトリがあれば削除するか、ユーザーに確認する（オプション）
if [ -d "$RES_DIR" ]; then
    echo "警告: 結果ディレクトリ '$RES_DIR' は既に存在します。上書きします。"
    rm -rf "$RES_DIR"
fi

echo ">>> 設定を生成します (NODE_NUM: ${NODE_NUM})..."
python3 create_config_yaml_for_specified_node_num_per_container.py $NODE_NUM $NODE_NUM_PER_COMPONENT

echo "ROS2 launchファイルを実行します..."
# ROS2のlaunchファイルをバックグラウンドで実行
source ../../install/setup.bash
ros2 launch executors_loader many_executors_loader.launch.py executor_config_file:=config.yaml &

# launchファイルのプロセスIDを取得
ROS2_PID=$!

# 起動が完了するまで待機しつつ、CPU使用率のログを取る
python3 wait_until_cpu_usage_got_low.py > result/cpu_usage_log_total_node_num_${NODE_NUM}_per_component_${NODE_NUM_PER_COMPONENT}.csv

echo ">>> VTuneによるプロファイリングを開始します (Duration: 10s)..."
source /opt/intel/oneapi/setvars.sh
vtune -collect hotspots \
      -finalization-mode=full \
      -knob sampling-mode=hw \
      -knob enable-stack-collection=false \
      -knob stack-size=4096 \
      -knob enable-characterization-insights=false \
      -data-limit 10000 \
      -duration 10 \
      -analyze-system \
      -result-dir $RES_DIR

echo ">>> プロファイリングが完了しました。レポートを生成します..."
vtune -report hotspots \
      -result-dir $RES_DIR \
      -format=csv \
      -report-output $RES_DIR/hotspots_report.csv
vtune -report hotspots \
      -result-dir $RES_DIR \
      -format=csv \
      -group-by=process \
      -report-output $RES_DIR/process_report.csv

echo ">>> 完了: レポートは $RES_DIR/hotspots_report.csv に保存されました。"

# 起動したROS2のプロセスを終了
pkill -g ${ROS2_PID}

# simple_listenerとsimple_talkerプロセスが完全に終了するまで待機
echo "ROS2のプロセスを終了中..."
pkill -f rclcpp_components/component_container
sleep 1
while true; do
    node_count=$(pgrep -f rclcpp_components/component_container | wc -l)
    
    if [ $node_count -eq 0 ]; then
        echo "全てのROS2プロセスが終了しました。"
        break
    fi
    
    echo "残りのプロセス: node_count=$node_count. 再度killします..."
    pkill -f rclcpp_components/component_container
    sleep 10
done

echo "ROS2のプロセスを終了しました。"
