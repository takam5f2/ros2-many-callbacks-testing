# 使い方

## 前提条件
- VTuneインストール済みであること
- simple_nodeのパッケージがビルド済みであること

## 測定方法
ノード数が10個でpubsubが1対ずつあり、topic周波数が5Hzの結果(=callback起床回数が50回)を測定する場合
```
source ../install/local_setup.bash
./run_vtune_systemwide.sh 10 5
```
結果はresult/ディレクトリに出力されます
