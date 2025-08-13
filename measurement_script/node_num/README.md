# 使い方

## 前提条件
- VTuneインストール済みであること
- simple_nodeのパッケージがビルド済みであること

## 測定方法
ノード数が10個でpubsubが1対ずつある場合の結果を測定する場合
```
source ../install/local_setup.bash
./run_vtune_systemwide.sh 10 pub_sub
```
結果はresult/ディレクトリに出力されます

他の測定条件のオプションに関してはスクリプトを引数なしで実行すると確認できます。
```
measurment_scripts$ ./run_vtune_systemwide.sh
Usage: ./run_vtune_systemwide.sh <node_num> [pub_sub|pub_only|no_publisher]
```
