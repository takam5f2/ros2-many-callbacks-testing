"""
ROS2のノードが立ち上がるまでにノード数に応じて時間がかかる。
適切なタイミングでVTune計測を行いつつ、立ち上がるまでの時間の測定も行うためのスクリプト。
ROS2ノード立ち上げ中はCPU使用率が100%で張り付く挙動をするため、CPU使用率がしきい値以下になってから指定時間待機する。
"""
import time
import sys
import psutil  # psutilライブラリをインポート

# --- 設定項目 ---
# 閾値となるCPU使用率（この値以下でカウントダウン開始）
THRESHOLD_PERCENT = 40.0

# 閾値以下の状態が続く秒数（この秒数続いたら終了）
DURATION_SECONDS = 20
# ---

def monitor_cpu_with_psutil():
    """psutilを使用してCPU使用率を監視します。"""
    low_usage_counter = 0
    start_time = time.time()

    print("time[sec],CPU Usage[%]")

    try:
        # ループを開始
        while True:
            # psutilでCPU使用率を取得（interval=1で1秒間の平均使用率を取得）
            # これにより、time.sleep(1)が不要になります。
            cpu_percent = psutil.cpu_percent(interval=1)

            # 経過時間を計算して表示
            passed_time = time.time() - start_time
            print(f"{passed_time:.1f},{cpu_percent:.2f}")
            sys.stdout.flush()  # 出力を即座に反映

            # 閾値と比較
            if cpu_percent <= THRESHOLD_PERCENT:
                low_usage_counter += 1
            else:
                # 閾値を超えたらカウンターをリセット
                low_usage_counter = 0

            # 終了条件をチェック
            if low_usage_counter >= DURATION_SECONDS:
                sys.exit(0)

    except KeyboardInterrupt:
        print("\n🚫 スクリプトが手動で中断されました。")
        sys.exit(1)
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        sys.exit(1)

if __name__ == "__main__":
    monitor_cpu_with_psutil()
