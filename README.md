﻿# LittleSLAM

## 分析用に改造しています．

### ログの出力
LittleSLAM/framework/debug.h で，ログの出力コンパイル時に抑制できます．LittleSLAM/cui/main.cppのspdlog::set_level()も変更する必要があります．

|ログの出力|SPDLOG_ACTIVE_LEVEL|set_level()|備考|
|:--:|:---:|:--:|:---:|
|する|SPDLOG_LEVEL_INFO|spdlog::level::info|プロットありだがsleepは消してある|
|しない|SPDLOG_LEVEL_NONE|spdlog::level::none|プロットもしなくなる|

## Eigenの設定
eightnsetting.hで一括で設定できるようにしてあります．

## データの読み込み
iostream によるデータの読み込みが遅そうだったので，cstdioを使い書き直しました．効果は微妙です．

## 現在の問題
実行時間があまり安定していません．

## LittleSLAMについて

LittleSLAMは、SLAM学習用プログラムです。
2Dレーザスキャナのデータ（スキャン）とオドメトリデータを格納したファイルを入力し、
ロボット位置の軌跡と2D点群地図をgnuplot上に出力します。

LittleSLAMは、スキャンマッチングに基づく位置合せ、レーザスキャナとオドメトリのセンサ融合、
Graph-based SLAMに基づくループ閉じ込みなどの要素技術から構成されています。

LittleSLAMは参考書籍[1]の教材として作られたプログラムであり、
わかりやすさを優先してシンプルなアルゴリズムを採用しています。
そのため、フルスペックのSLAMプログラムと比べると性能は落ちますが、
内容の理解はしやすくなっています。


## 実行環境

LittleSLAMはプログラミング言語C++で記述されています。
動作を確認した実行環境は下記のものです。いずれも64ビット版です。

| OS | C++ |
|:--:|:---:|
| Windows 7 | Visual C++ 2013 (Visual Studio Community 2013)|
| Windows 10 | Visual C++ 2015 (Visual Studio Community 2015)|
| Linux Ubuntu 14.04 LTS | gcc 4.8.4|
| Linux Ubuntu 16.04 LTS | gcc 5.4.0|

32ビットOSでの動作確認はしていないので、必要な場合はご自分で試してください。


## 必要なソフトウェア

LittleSLAMの実行には、下記のソフトウェアが必要です。

| ソフトウェア | 内容 | バージョン |
|:------------:|:----:|:----------:|
| Boost        | C++汎用ライブラリ |1.58.0 |
| Eigen3       | 線形代数ライブラリ|3.2.4 |
| gnuplot      | グラフ描画ツール  |5.0 |
| CMake        | ビルド支援ツール  |3.2.2 |
| p2o          | Graph-based SLAMソルバ|beta |

バージョンはLittleSLAMの開発で使用したものであり、明確な条件ではありません。
これ以上のバージョンであれば通常は動作します。
これ以下のバージョンでも動作する可能性はあります。

## 使い方

- Windowsでの使い方は[こちら](doc/install-win.md)

- Linuxでの使い方は[こちら](doc/install-linux.md)

## データセット

実験用に6個のデータファイルを用意しています。下表に一覧を示します。
[ここ](https://furo.org/software/little_slam/dataset.zip)からダウンロードできます。


| ファイル名          | 内容         |
|:--------------------|:-------------|
| corridor.lsc        | 廊下（単一ループ） |
| hall.lsc            | 広間（単一ループ） |
| corridor-degene.lsc | 廊下（退化） |
| hall-degene.lsc     | 広間（退化） |
| corridor-loops.lsc  | 廊下（多重ループ） |
| hall-loops.lsc      | 広間（多重ループ） |

## カスタマイズ

LittleSLAMは学習用プログラムであり、基本形からいくつかの改良を経て
完成するようにカスタマイズできます。  
詳細は[こちら](doc/customize.md)を参照してください。

## 参考書籍

下記の書籍はSLAMの解説書です。SLAMの一般的な解説をするとともに、
具体例としてLittleSLAMを教材に用い、そのソースコードの詳細を説明しています。

[1] 友納正裕、「SLAM入門 -- ロボットの自己位置推定と地図構築の技術」、オーム社、2018年  

## ライセンス

- LittleSLAMは、MPL-2.0ライセンスにもとづいています。

