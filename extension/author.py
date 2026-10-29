"""Build final Japanese report and <=180s demo from verified measured artifacts."""
from pathlib import Path
import json,math
import numpy as np
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent; ROOT=HERE.parent; OUT=HERE/'output'

def report():
    from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,PageBreak,Table,TableStyle,Image as RImage
    from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.cidfonts import UnicodeCIDFont
    from reportlab.lib import colors
    from pypdf import PdfReader,PdfWriter
    pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='JP',fontName='HeiseiKakuGo-W5',fontSize=10,leading=16,spaceAfter=9,wordWrap='CJK'))
    styles.add(ParagraphStyle(name='JH',parent=styles['JP'],fontSize=16,leading=23,textColor=colors.HexColor('#126477'),spaceAfter=14))
    story=[]
    def p(s,heading=False): story.append(Paragraph(s,styles['JH' if heading else 'JP']))
    def table(rows,widths):
        converted=[[Paragraph(str(v),styles['JP']) for v in row] for row in rows]
        t=Table(converted,colWidths=widths,repeatRows=1)
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e1eff3')),
            ('GRID',(0,0),(-1,-1),.3,colors.HexColor('#b6c9cc')),('VALIGN',(0,0),(-1,-1),'TOP'),
            ('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7)]))
        story.append(t); story.append(Spacer(1,12))
    stats=json.loads((OUT/'summary.json').read_text()); train=json.loads((OUT/'training.json').read_text())
    verified=json.loads((OUT/'verification.json').read_text()); ds=json.loads((OUT/'dataset.json').read_text())
    p('追加研究：rosbag2から学ぶ再観測走行',True)
    p('SECOND LOOK / 第5回の収集・変換・学習・デプロイ・評価を、遮蔽下の能動点検へ発展。前半6ページは保存済み144試行の認識・再観測研究、本追補は別に実行した模倣学習の研究である。集計を混ぜない。')
    p('研究仮説と工夫',True)
    p('一回の画像だけでは亀裂が隠れて誤正常になる。認識の不確実性と既知地図の可視性を使って再観測を要求し、その走行指令を教師制御から学習する。課題サンプルの単純二輪操作やアームのお辞儀とは目的・認識・評価が異なる。全受講者との非重複は確認できていない。')
    p('実装と学習範囲',True)
    p('Gazebo Harmonicの車輪・接触・RGB画像をROS 2 Jazzyで処理。教師の走行をrosbag2 SQLiteへ保存し、同一メッセージ内で対応付けた観測と速度指令を抽出する。観測は距離/5・向き誤差のsin/cosの3値、行動は前進速度と旋回速度の2値。3-48-2のMLPで教師あり模倣学習する。')
    p('既知地図・オドメトリ・従来の経由点計画と最終向き合わせを使う。画像からの直接行動生成、ACT、SLAMの実装とは主張しない。画像MLPは前半と同じ固定モデルで、正解ラベルは描画と採点以外へ渡さない。')
    p(f"教師走行{len(ds['episodes'])}回、計{sum(ds['episodes'].values())}フレーム。走行単位で学習{train['train_frames']}・確認{train['validation_frames']}フレームに分離。確認MSEは前進{train['validation_mse_per_action'][0]:.6f}、旋回{train['validation_mse_per_action'][1]:.6f}。確認誤差だけで走行成功とはしない。")
    p('追加の学習・評価はCPUで実行。LeRobotDataset変換アダプタは用意したが未実行。主要結果はrosbag2由来NPZの軽量BCによるもの。Rerun・wandbは未使用。')
    story.append(PageBreak())
    p('未使用seedによる従来制御・BC比較',True)
    p('教師seed 3100-3107と評価seed 5100-5107を分離し、同じ8シーンを2制御方式で比較。距離3.5-4.5m・照度2水準・左右回り込み・正常/亀裂を含む。全シーン遮蔽あり。移動方式を比較するため再観測を全試行で要求する。認識の不確実性による省移動の比較は前半の144試行で扱う。')
    rows=[['方式','到達/試行','接触なし到達','平均移動','平均時間','最終正判定']]
    for mode,label in [('teacher','従来制御'),('bc','模倣学習')]:
        s=stats[mode]; n=s['trials']; rows.append([label,f"{s['reached']}/{n}",f"{s['reached_without_contact']}/{n}",f"{s['mean_distance_m']:.2f}m",f"{s['mean_sim_seconds']:.2f}s",f"{s['correct_final_decisions']}/{n}"])
    table(rows,[75,65,80,70,70,80])
    p('到達は各経由点まで0.10m以内、壁時計45秒またはシミュレーション65秒の制限内。接触なし到達と最終画像の正判定を別に数える。時間はシミュレーション時間。接触メッセージ0は安全保証ではない。')
    p('検証と限界',True)
    p(f"検証結果 {verified['status']}：SQLiteの話題別件数と抽出件数、走行単位の分離、時刻単調性、指令範囲、モデルと設計ソースのハッシュ、同一シーン比較、保存実位置からの距離・集計を独立照合。{verified['paired_trials']}試行の実測を記録した。")
    p('小規模・同じ生成器・既知地図の限定評価であり、未知環境での汎化や従来制御に対する優位性は示していない。BCの目的は教師行動の再現。既存認識の高確信度誤正常は3件残り、可視性ガードを組み合わせた単独デプロイは追加の動作確認に留まる。')
    story.append(RImage(str(OUT/'rviz.png'),width=445,height=278))
    p('RViz実画面：Xvfb内で起動し、ROSのRGBとオドメトリを表示。実機の映像ではない。')
    story.append(PageBreak())
    p('課題条件への対応・再現・提出',True)
    table([['要件','対応と状態'],
        ['AI×ロボット','固定画像MLP＋新規BC MLP。Gazeboで速度指令により走行。'],
        ['Docker / ROS 2 / Python','WSL Ubuntu内Docker Engineで実行。専用Dockerfile・Compose・起動スクリプト。'],
        ['Gazebo / RViz / rosbag2','物理走行・RGB取得、RViz実画面、教師8走行のSQLiteを保存。'],
        ['データ・学習・推論・評価','収録→NPZ→BC学習→Gazebo再投入→従来制御との16試行比較。'],
        ['1-3分の動画','高解像度3D映像・従来/BCの全走行比較・認識再観測・RVizで再構成。時間はmedia.json。'],
        ['コード・レポート','ソース・モデル・生ログ・検査・本PDFを専用提出ZIPに収録。'],
        ['public GitHub','既存リポジトリはprivate。公開設定または専用public作成が必要。'],
        ['YouTube / OmniCampus','アップロードと最終送信は未実施。本人アカウントでURL提出が必要。'],
        ['締切・名称','2026/11/2 10:00 JST。PDF名と動画タイトルにOmniCampus IDを入れる。']], [125,315])
    p('再現：ROS版フォルダで bash extension/run.sh build。その後 collect → dataset → train → evaluate → rviz の順。既存実測を残すため別コピーでextension/outputを空にして実行する。元のoutputは画像認識モデルを含むため保存する。')
    p('Docker DesktopのInference manager初期化は行わず、既存WSL内Engineへ切替。Desktopの復旧自体は未実施。バックアップなしの初期化・配布データ削除は行わない。Composeプラグインのない環境でもrun.shから実行できる。')
    p('出典：課題原本「最終課題（10_19 改訂版）」全5ページ、第5回講義資料の収集・変換・学習・推論・評価手順を確認。講義資料・講義コードは再配布しない。公式参考：huggingface.co/docs/lerobot/main/api/datasets、github.com/ros2/rosbag2。制作・実行検証にCodexを使用。')
    supplement=OUT/'supplement.pdf'
    SimpleDocTemplate(str(supplement),pagesize=(595,842),rightMargin=52,leftMargin=52,topMargin=45,bottomMargin=42).build(story)
    writer=PdfWriter()
    for source in [ROOT/'output/ros_report.pdf',supplement]: writer.append(PdfReader(source))
    with (OUT/'submission_report.pdf').open('wb') as f: writer.write(f)
    print('PDF pages',len(writer.pages))

def video():
    import imageio.v2 as imageio
    from video import base,text,card
    fps=12; frames=0; files=[]
    rec=np.load(OUT/'frames_5100_bc.npz'); span=float(rec['time'][-1]-rec['time'][0]); duration=math.ceil(span)+1
    assert 132+3+duration+5+4<=180,'show full motion within course time limit'
    stats=json.loads((OUT/'summary.json').read_text()); target=OUT/'submission_demo.mp4'
    writer=imageio.get_writer(str(target),fps=fps,codec='libx264',quality=7,macro_block_size=16)
    def append(im):
        nonlocal frames
        writer.append_data(np.asarray(im)); frames+=1
    try:
        with imageio.get_reader(str(ROOT/'output/gazebo_demo.mp4')) as reader:
            for frame in reader: append(frame)
        im=card('Extension / Learn the reinspection motion',[
            'Gazebo -> rosbag2 -> paired state/actions -> BC -> Gazebo',
            '8 teacher demonstrations; 8 fresh paired evaluation scenes.',
            'State policy, known map; not a vision-only ACT policy.',
            'The following clip replays actual BC motion at 1x sim time.'])
        for _ in range(3*fps): append(im)
        for k in range(duration*fps):
            desired=float(rec['time'][0])+k/fps
            i=min(len(rec['time'])-1,max(0,int(np.searchsorted(rec['time'],desired,side='right')-1)))
            im,d=base('Learned BC / held-out scene 5100','Actual Gazebo cameras | full recorded sequence | 1x simulation time')
            im.paste(Image.fromarray(rec['overview'][i]),(30,160))
            im.paste(Image.fromarray(rec['rgb'][i]).resize((384,288)),(546,160))
            text(d,35,550,'BC predicts wheel velocity commands from goal error state.',20)
            text(d,35,580,'Map waypoints and final facing remain conventional.',18)
            append(im)
            if k==6*fps: im.save(OUT/'bc_preview.png')
        a=stats['teacher']; b=stats['bc']
        im=card('Held-out locomotion comparison / measured',[
            f"Teacher: {a['reached_without_contact']}/{a['trials']} reached without body contact.",
            f"BC: {b['reached_without_contact']}/{b['trials']} reached without body contact.",
            f"Mean travel: teacher {a['mean_distance_m']:.2f} m / BC {b['mean_distance_m']:.2f} m.",
            'Small known-map simulation; no claim of real-world safety.'])
        for _ in range(5*fps): append(im)
        screen=Image.open(OUT/'rviz.png').convert('RGB').resize((896,560))
        im=Image.new('RGB',(960,640),'#0d1c2b'); im.paste(screen,(32,30))
        d=ImageDraw.Draw(im); text(d,32,604,'RViz + ROS camera/odometry | actual Xvfb screen',20)
        for _ in range(4*fps): append(im)
    finally: writer.close()
    decoded=0
    with imageio.get_reader(str(target)) as reader:
        for frame in reader:
            assert frame.shape==(640,960,3); decoded+=1
    assert frames==decoded
    result={'status':'PASS','seconds':frames/fps,'fps':fps,'frames':frames,
            'decoded_frames':decoded,'motion_speed':'1x simulation time plus explanation holds',
            'bc_scene':5100,'bc_recorded_span_s':span,'original_segment_seconds':132}
    (OUT/'media.json').write_text(json.dumps(result,indent=2)); print(result)

if __name__=='__main__':
    import sys
    if 'video' in sys.argv: video()
    else: report()
