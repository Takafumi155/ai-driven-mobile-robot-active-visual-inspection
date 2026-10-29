# 出典とライセンス

この提出版はSECOND LOOKの二輪ロボットによる能動的外観検査を対象にした独立スナップショットです。既存のMIT LICENSEをそのまま保持しています。モデルと教師データは、同梱コードのGazeboシミュレーションで生成された成果物として元プロジェクトに記録されています。外部のロボットメッシュ、写真、実測個人データは同梱していません。環境形状は `world.py` で生成されます。

ROS 2、Gazebo、ros_gz、RViz、NumPy、SciPy、scikit-learn、Pillowなどは外部依存であり、本リポジトリのMITライセンスで再ライセンスされません。Dockerfileはパッケージを取得しますが、その配布物やコンテナイメージ自体は同梱しません。動画・PDFの作成には別途 `requirements-artifacts.txt` の依存を使用します。フォントやライブラリを含む生成物を再配布する際は、使用した依存の条件も確認してください。

参照資料：
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo HarmonicとROSの導入](https://gazebosim.org/docs/harmonic/ros_installation/)
- [ros_gz](https://github.com/gazebosim/ros_gz)
- [scikit-learn](https://scikit-learn.org/stable/)

この確認は同梱ファイルと出典記載の点検です。著作権帰属の独立した証明や、全依存の法的審査を意味しません。
