# AIを用いた二輪ロボットの能動的外観検査

## 概要
差動二輪ロボットがカメラで部品の亀裂を検査し、判断が難しい場合に別の位置へ移動して再撮影するシステムです。ROS 2とGazeboで走行・撮影を行い、画像認識と模倣学習を組み合わせました。

## 使用技術
ROS 2 Jazzy / Gazebo Harmonic / Docker / Python / RViz / rosbag2 / scikit-learn

## 実験1：再観測による検査性能
48条件で3方式を比較（144試行）。

| 方式 | 亀裂発見 | 誤正常 | 平均移動距離 | 平均検査時間 |
|---|---:|---:|---:|---:|
| 1回だけ撮影（single） | 8/24 | 3件 | 0.00 m | 0.71秒 |
| 毎回再観測（always） | 24/24 | 0件 | 3.97 m | 15.11秒 |
| 判定保留時のみ再観測（selective） | 21/24 | 3件 | 1.92 m | 7.49秒 |

selectiveはalwaysに比べ、平均移動距離を51.6%、検査時間を50.4%削減しました。ただし、亀裂を正常と誤判定した例が残っています。

## 実験2：走行の模倣学習
rosbag2で教師走行8回・1,003サンプルを収集し、目標との距離や向きの誤差から走行速度を出力する状態ベースの模倣学習（BC）を行いました。異なる8場面で従来制御と比較しました。

| 方式 | 目標到達 | 最終正判定 | 平均走行距離 | 平均時間 |
|---|---:|---:|---:|---:|
| 従来制御 | 8/8 | 8/8 | 4.104 m | 15.233秒 |
| 模倣学習（BC） | 8/8 | 8/8 | 4.122 m | 15.240秒 |

模倣学習でも到達できましたが、従来制御に対する距離・時間の改善は確認できませんでした。上記の16試行は実験1の144試行とは別のデータです。

## 主なファイル
- `world.py`：ロボット、カメラ、障害物のシミュレーション環境
- `runtime.py`：ROS 2通信、速度指令、センサデータの取得
- `runner.py`：画像認識の学習と3方式の評価
- `extension/pipeline.py`：rosbag2収集、BC学習、走行比較
- `extension/deploy.py`：学習済み走行制御での再観測
- `Dockerfile` / `compose.yaml`：実行環境
- `output/` / `extension/output/`：学習済みモデルと軽量な実験結果

追加実験の概要と操作方法は [extension/README.md](extension/README.md) を参照してください。

## 実行例
Ubuntu / WSL環境で、このフォルダを開いて実行します。

```bash
bash extension/run.sh build
docker compose run --rm research python3 verify_submission.py
bash extension/run.sh rviz
```

新しい条件での実験は既存の出力を保護するため、別コピーで実施してください。動画・PDF・全画像・生rosbag2などの大容量データはローカル成果物に保存しており、GitHubにはすべてを含めていません。

## 限界
既知の静的環境と限定された亀裂形状でのシミュレーション結果です。未知環境、実機での認識性能や安全性は未検証です。接触通知0だけでは衝突なしを保証できません。
## 提出リポジトリの取得と軽量検証

```bash
git clone https://github.com/Takafumi155/second-look-submission.git
cd second-look-submission
python -m pip install numpy
python verify_submission.py
```

Python 3.11以降を推奨します。この検証はROS・Docker不要で、保存済みのモデル、教師データ、16試行ログを読み取り専用で点検します。Gazeboを起動する動作確認とは異なります。元の `extension/verify.py` は非同梱の生rosbag2が必要です。

Docker実行にはUbuntu / WSL2、稼働中のDocker Engineが必要です。まず上記の `extension/run.sh build` でベース画像と拡張画像を順に作成してください。実験1を再実行する場合は別コピーで既存の `output/` を退避してから、コンテナ内で `bash reproduce_simulation.sh` を実行します。実験2は別コピーで `bash extension/run.sh collect` → `dataset` → `train` → `evaluate` の順です。RVizの画面表示には別途X11等のGUI接続設定が必要です。

動画・PDF・生rosbag2・全画像・実験1の全試行ログは同梱していません。動画URLとPDFは別途提出してください。元の保存結果の全検証にはこれらのローカル成果物が必要です。

出典・依存は [ATTRIBUTION.md](ATTRIBUTION.md)、公開時の検査範囲と制約は [PUBLICATION_CHECK.md](PUBLICATION_CHECK.md) を参照してください。
