"""Static experiment graphs and Japanese PDF from actual Gazebo measurements."""
from pathlib import Path
import json, math
from PIL import Image, ImageDraw, ImageFont
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'
NAMES={'single':'Single view','always':'Always reobserve','selective':'Uncertainty-aware'}
COLORS={'single':'#90a5bb','always':'#eebd68','selective':'#4ed7c0'}
def font(n):
    for path in ['C:/Windows/Fonts/arial.ttf','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        if Path(path).exists(): return ImageFont.truetype(path,n)
    return ImageFont.load_default()
def graphs():
    stats=json.loads((OUT/'summary.json').read_text())['overall']
    im=Image.new('RGB',(1200,760),'#0e1d2c'); d=ImageDraw.Draw(im)
    def text(x,y,t,n=24,fill='#e8f1ff'): d.text((x,y),t,font=font(n),fill=fill)
    text(40,30,'SECOND LOOK / ROS 2 + GAZEBO',22,'#4ed7c0'); text(40,75,'Measured comparison: real rendered images and wheel motion',30)
    panels=[('detection_rate','Defects detected',1,True),('mean_distance_m','Mean physical travel',6,False),('mean_seconds','Mean inspection time',30,False)]
    for j,(key,title,maxv,percent) in enumerate(panels):
        x=40+j*395; text(x,170,title,25)
        maxv=max(maxv,max(r[key] for r in stats.values())*1.15)
        for i,(p,r) in enumerate(stats.items()):
            y=235+i*115; text(x,y,NAMES[p],20,COLORS[p]); value=r[key]
            d.rounded_rectangle((x,y+38,x+330,y+62),radius=7,fill='#263c50')
            d.rounded_rectangle((x,y+38,x+max(1,330*value/maxv),y+62),radius=7,fill=COLORS[p])
            label=f'{100*value:.1f}%' if percent else f'{value:.2f}'+(' m' if key=='mean_distance_m' else ' s')
            text(x,y+72,label,22)
    a=stats['always']; s=stats['selective']; saved=100*(1-s['mean_distance_m']/max(a['mean_distance_m'],1e-12))
    text(40,625,f"Selective: {s['detected']}/{s['defects']} defects, {saved:.1f}% less travel than always",25,'#4ed7c0')
    text(40,675,f"Selective false normal: {s['defect_called_normal']}/{s['defects']}. Uncertainty alone cannot guarantee visibility.",21,'#ff9a9a')
    im.save(OUT/'gazebo_comparison.png')

def report():
    from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Image as RImage,Table,TableStyle,PageBreak
    from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.cidfonts import UnicodeCIDFont
    from reportlab.lib import colors
    pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
    st=getSampleStyleSheet(); st.add(ParagraphStyle(name='JP',fontName='HeiseiKakuGo-W5',fontSize=10,leading=16,spaceAfter=8,wordWrap='CJK'))
    st.add(ParagraphStyle(name='HJ',parent=st['JP'],fontSize=14,leading=21,textColor=colors.HexColor('#14677b'),spaceBefore=9))
    st.add(ParagraphStyle(name='TJ',parent=st['JP'],fontSize=22,leading=31,spaceAfter=14))
    story=[]
    def p(t,style='JP'): story.append(Paragraph(t,st[style]))
    def table(data,widths):
        t=Table(data,colWidths=widths,repeatRows=1); t.setStyle(TableStyle([('FONTNAME',(0,0),(-1,-1),'HeiseiKakuGo-W5'),('FONTSIZE',(0,0),(-1,-1),9),('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e0eef1')),('GRID',(0,0),(-1,-1),.4,colors.HexColor('#b3c7cc')),('TOPPADDING',(0,0),(-1,-1),8),('BOTTOMPADDING',(0,0),(-1,-1),8)])); story.append(t); story.append(Spacer(1,9))
    data=json.loads((OUT/'summary.json').read_text()); stats=data['overall']; tr=json.loads((OUT/'training.json').read_text())
    p('SECOND LOOK<br/>不確実なら、位置を変えて見直す<br/>自律点検ロボット','TJ')
    p('Physical AI応用1 最終課題 | ROS 2 Jazzy / Gazebo Harmonic / Docker | 2026年10月28日')
    p('1. 目的と工夫','HJ')
    p('遮蔽物の向こうの設備を、最初の画像だけで正常と判定してよいか。本作品は実際のGazeboカメラ画像で亀裂を認識し、不確実なときだけ別の視点へ車輪走行して再観測する。単なる巡回ではなく、画像AIの確信度がロボットの追加行動を決める閉ループを実装した。')
    story.append(RImage(str(OUT/'smoke_reobserved.png'),width=440,height=330))
    p('Gazeboが描画し、ros_gz_bridge経由で取得した実カメラ画像。黒い亀裂はシーン内の立体視覚要素であり、画像へ後から貼り付けた認識用オーバーレイではない。実機映像ではない。')
    p('初期の独自2D版とは別に、今回はROS 2の速度指令・位置フィードバック、Gazeboの物理・カメラ・接触センサーを使って実行した。旧2D版の結果をGazeboの実績として流用していない。')
    story.append(PageBreak())
    p('2. 実装・AI・従来制御の境界','HJ')
    p('Ubuntu 24.04上のDocker Engineで公式ros:jazzy-ros-baseを基に専用イメージを構築。ros-jazzy-ros-gz-sim / bridge、CPUソフトウェア描画を使用。WindowsのDocker Desktopは起動エラーだったため初期化せず、WSL Ubuntu内にDocker Engineを導入して動作させた。')
    p('差動二輪ロボットの車輪半径0.125m、左右間隔0.46m。DART系物理の接触・重力を使用し、ROS 2ノードがTwist速度指令を送り、Gazebo DiffDriveが車輪関節を駆動する。オドメトリで位置・姿勢をフィードバックし、中間点を経由して遮蔽物を回り込む。試行中にロボットの位置を直接書き換えない。学習用撮影の姿勢変更と試行間リセットだけでset_pose / world resetを使用する。')
    p('認識は位置に依存しにくい66特徴を入力にする隠れ48ユニットのMLP。中央200×200領域を64×64へ縮小し、輝度ヒストグラム32値・勾配16値・分位点12値・RGB統計6値から正常と亀裂を学習。推論は画像だけを使い、シミュレータの正解ラベルを読まない。ルールをAIと称していない。')
    p(f"Gazebo実画像{tr['original_images']}枚を取得し、姿勢インデックスで学習{tr['train_images']}枚・温度校正{tr['calibration_images']}枚・テスト{tr['test_images']}枚へ分離。学習特徴だけに小さなノイズ拡張を加え{tr['augmented_training_features']}件を学習。遮蔽・照度0.35/1.0・位置・向きを変える。同じ亀裂形状と基本シーンを共有する限定的な画像集合であり、外部の実画像検証ではない。")
    p(f"校正用画像の対数損失で温度T={tr['temperature']:.2f}を選択。テストBrier {tr['uncalibrated']['brier']:.4f}→{tr['calibrated']['brier']:.4f}、ECE {tr['uncalibrated']['ece']:.4f}→{tr['calibrated']['ece']:.4f}。保存重みのNumPy推論とscikit-learn推論の最大差は{tr['model_export_max_abs_error']:.2e}。校正が個別の正しさや安全を保証するわけではない。")
    p('画像テストの内訳は、遮蔽なし31/32件が二択で正解、完全遮蔽は16/32件、完全遮蔽32枚には閾値0.85以上の確信を持った例はなかった。一方、未使用の評価照度0.45では高確信度の誤正常が生じ、校正用条件と評価条件の違いが限界として現れた。')
    p('P(亀裂)≧0.85なら異常、≦0.15なら正常、それ以外は保留。閾値はGazebo評価前に固定。selectiveは保留時だけ最大1回再観測し、最後の画像の判定を採用する。独立画像と仮定した確率の掛け合わせはしない。')
    p('初期の画素位置依存MLPでは、走行後の斜め画像を正常と誤る失敗が生じた。26試行の開発ログを保存し、位置を集約した画像統計へ変更した。その後の評価seed 2301/2302/2303は未使用のものを選び、モデル・閾値・制御コードを固定した。最終MLPは上限250反復に達したため、最適化の収束は保証しない。')
    p('視点候補2か所を対象への距離品質と経路長で順位付けする。地図・対象位置・自己位置は既知。経路は既知の矩形障害物を避ける中間点を使い、未知環境のSLAM・Nav2や学習した期待情報利得ではない。認識MLと幾何学的経路判断の役割を区別する。')
    story.append(PageBreak())
    p('3. 実験方法','HJ')
    n=stats['single']['n']; seeds=list(data['by_seed'])
    p(f"評価seedは{' / '.join(seeds)}。遮蔽の有無、対象距離3.5/4.5m、照度0.45/1.0、正常/亀裂を組み合わせた{n}シーン。seedごとにパネル横位置を±0.055m変動させる。各シーンで3方式を実行し、計{sum(r['n'] for r in stats.values())}試行。方式間で同じシーンパラメータを使い、物理世界をリセットして比較する。")
    table([['方式','再観測条件'],['single','初回のみ。判定保留でも終了'],['always','毎回別視点へ移動し、1回再撮影'],['selective','初回のAI判定が保留のときだけ再撮影']],[110,365])
    p('発見率＝亀裂を異常とした件数／亀裂総数。亀裂を正常と誤判定した数と、保留に留まった数は別に数える。保留は成功に含めない。判定率coverageは正常・異常のいずれかを出せた割合。距離はGazebo物体位置の時系列から積算し、オドメトリ距離も別記録。時間は実際に進んだシミュレーション時刻の差で、旧2D版の定数時間モデルではない。')
    verification=json.loads((OUT/'verification.json').read_text())
    p(f"全{sum(r['n'] for r in stats.values())}試行の本体接触メッセージは{sum(r['contact_messages'] for r in stats.values())}、記録された位置での最小障害物余裕は{verification['minimum_obstacle_clearance_m']:.3f}m。保存画像{verification['recomputed_camera_predictions']}枚を独立に再推論し、実位置記録{verification['world_position_samples']}点から距離と余裕を照合してPASSした。")
    p('Gazeboの本体接触センサーのメッセージ数と、既知障害物からの幾何学的余裕を併記する。接触メッセージ数は衝突イベント数と同一ではなく、同じ接触が複数回発行される場合がある。接触ゼロは実世界の安全保証ではない。')
    p('4. 実測結果','HJ')
    table([['方式','亀裂発見','誤正常','保留','距離 / 時間'],*[[p,f"{r['detected']}/{r['defects']}",str(r['defect_called_normal']),f"{r['unresolved']}/{r['n']}",f"{r['mean_distance_m']:.2f}m / {r['mean_seconds']:.2f}s"] for p,r in stats.items()]],[85,80,70,85,155])
    a=stats['always']; b=stats['selective']; saved=100*(1-b['mean_distance_m']/max(a['mean_distance_m'],1e-12))
    p(f"selectiveは亀裂{b['defects']}件中{b['detected']}件を発見。alwaysに対して平均距離は{saved:.1f}%減った。再観測はalways {a['reobservations']}回、selective {b['reobservations']}回。全対象の判定率は{b['coverage']*100:.1f}%、保留は{b['unresolved']}件であり、これらは検査完了と扱わない。")
    story.append(PageBreak())
    p('5. 比較図と解釈','HJ')
    story.append(RImage(str(OUT/'gazebo_comparison.png'),width=475,height=300.83))
    p('学習・校正画像と評価のパネル位置seedを分けているが、単一の亀裂形状と既知地図に強く依存する。認識器の性能を一般的な設備点検AIの性能へ広げて解釈しない。暗さ・距離・遮蔽で保留が増え、再観測しても確信に達しない場合がある。')
    p(f"改良後も、selectiveで亀裂を正常とした例が{b['defect_called_normal']}件残った。低照度の遮蔽画像に過度の確信を持ち、再観測を起動しないことが原因。保留が少ないことを、安全に点検完了できた証拠とは解釈しない。別センサーや既知地図の可視性ガードによる対策が必要だが、今回のGazebo本評価では追加ガードを混ぜず3方式の比較を保った。")
    story.append(RImage(str(OUT/'condition_matrix.png'),width=475,height=329.86))
    p('条件別の発見数。各セルの亀裂は3 seedで3件に限られ、一般化性能の推定ではない。低照度・近距離の完全遮蔽で誤正常が残り、不確実性を計算するだけでは可視性を保証できないことが分かる。')
    p('比較の主眼は、同じ学習モデルでも「いつ再観測するか」で発見数と移動コストがどう変わるかである。発見が同数でも判定率・保留数・時間を併記し、速さだけで優位性を主張しない。自己位置・地図が不正確な場合や、途中に人が入る場合の回避は未検証。')
    p('6. 差別化と出典','HJ')
    p('能動知覚・不確実性・次視点選択には先行研究がある。独自性は、点検の「見えない」を認識の不確実性として扱い、物理走行を伴う追加観測の費用を3方式で揃えて比較する点。旧2D版では高確信度の誤正常が残る限界も確認済みで、現実の安全には可視性ガードや別センサーが必要になる。')
    p('課題PDFの例にある単純な二輪操作、pick-and-place、四足歩行の丸写しは行っていない。自作シーン、車体、亀裂、撮影・学習・校正・認識・走行・比較・可視化を実装。先行研究コードや外部学習重みは流用していない。')
    p('公開Webで最終課題テーマを調査し、ローカルの既存SafeSortとも別実装にした。PhysicalAI communityの最終課題チャンネルが存在することは確認したが、画面の最小化と画面操作中止により投稿一覧は読み取れていない。全受講者との非重複は保証しない。')
    p('一次資料：Gazebo公式のROS互換表（Jazzy＋Harmonic）、ros_gz_bridge公式、Gazebo DiffDrive / PosePublisher API、scikit-learn MLPClassifier公式。<br/>https://gazebosim.org/docs/harmonic/ros_installation/<br/>https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge<br/>https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html<br/>https://github.com/facebookresearch/Uncertainty-Driven-Active-Vision')
    story.append(PageBreak())
    p('7. 検証・再現・提出','HJ')
    p('ROS 2起動時の実カメラとオドメトリ取得、車輪走行、世界リセット、学習重みの推論一致を検証する。全試行について保存PNGから推論を再計算し、位置ログから距離・時間・幾何学的余裕を再計算、方式間のシーン一致、3 seedの要因網羅、保存集計を照合する。動画は記録したカメラと時刻を使い、説明用グラフ・字幕を合成する。')
    p('起動例：docker build -t second-look-ros:jazzy .<br/>docker run --rm --shm-size=512m -v [作業フォルダ]:/work second-look-ros:jazzy python3 runner.py smoke<br/>続けて collect → train → evaluate。READMEの完全な手順を参照。APIキー、GPU、外部モデル不要。初回のイメージ・パッケージ取得にはネットワークが必要。')
    p('Dockerfile、ROS 2パッケージ定義、Pythonソース、生成SDF、PNG画像、学習重み、全試行JSONL、3方式比較図、研究レポート、デモ動画を同梱。バージョン情報とイメージdigestを記録する。旧2D版の成果物は別フォルダに保存し、同じ実験と混同しない。')
    p('Codexを設計、コード作成、環境構築、実行、検証、失敗分析、図・動画・PDF作成に使用した。別のWorkセッションを実行したとは主張しない。実行していない試験や公開を完了と報告しない。')
    p('課題原本「最終課題（10_19 改訂版）.pdf」全5ページと照合。締切2026年11月2日10:00 JST、動画1〜3分、コード一式、PDF、public GitHub URL、YouTube URLが必要。PDF名・動画タイトルには個人名ではないOmniCampus IDを使用。個人情報・秘密情報・講義PDFは公開用パッケージに含めない。')
    p('8. 残る制約','HJ')
    p('単一の亀裂テンプレート、単一の障害物形状、既知地図、静的対象、少数seedに限定。実機、未知損傷、部分遮蔽、反射、地図誤差、人の動き、電力消費、通信故障時の安全停止は未実証。高い予測確率は正しさの保証ではない。Dockerの公式ベースイメージと各ライブラリの権利は各配布元に従う。')
    def footer(c,doc):
        c.setFont('HeiseiKakuGo-W5',9); c.setFillColor(colors.HexColor('#58717b')); c.drawString(45,25,'SECOND LOOK | ROS 2 + Gazebo'); c.drawRightString(550,25,str(doc.page))
    SimpleDocTemplate(str(OUT/'ros_report.pdf'),pagesize=(595.28,841.89),leftMargin=45,rightMargin=45,topMargin=40,bottomMargin=45,title='SECOND LOOK ROS 2 Gazebo inspection',author='').build(story,onFirstPage=footer,onLaterPages=footer)
if __name__=='__main__': graphs(); report()
