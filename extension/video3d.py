"""3D-motion-first submission video, using only recorded Gazebo camera frames."""
from pathlib import Path
import json,math
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio.v2 as imageio
HERE=Path(__file__).resolve().parent; ROOT=HERE.parent; OUT=HERE/'output'
FPS=12; BG='#102232'; WHITE='#edf3fa'; TEAL='#62e0cd'; GRAY='#aabccd'

def font(n):
    for p in ['C:/Windows/Fonts/meiryo.ttc','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        if Path(p).exists(): return ImageFont.truetype(p,n)
    return ImageFont.load_default()

def text(d,xy,s,n=20,color=WHITE): d.text(xy,s,font=font(n),fill=color)

def card(title,lines):
    im=Image.new('RGB',(960,640),BG); d=ImageDraw.Draw(im)
    text(d,(32,25),'SECOND LOOK / Physical AI 2026',21,TEAL)
    text(d,(32,85),title,31)
    for i,line in enumerate(lines): text(d,(35,195+i*66),line,23,WHITE if i else TEAL)
    text(d,(32,600),'ROS 2 + Docker + Gazebo + RViz + rosbag2 + Python',18,GRAY)
    return im

def clip(record,t,title,subtitle,meta,hero=False):
    times=record['time']; desired=float(times[0])+t
    idx=min(len(times)-1,max(0,int(np.searchsorted(times,desired,side='right')-1)))
    im=Image.new('RGB',(960,640),BG); d=ImageDraw.Draw(im)
    text(d,(22,10),title,25,TEAL); text(d,(22,52),subtitle,17,GRAY)
    overview=Image.fromarray(record['overview'][idx])
    # Crop the actual external camera image to give the moving robot more space.
    w,h=overview.size
    if not hero:
        overview=overview.crop((int(.10*w),int(.08*h),int(.98*w),int(.96*h)))
    im.paste(overview.resize((680,510)),(16,88))
    im.paste(Image.fromarray(record['rgb'][idx]).resize((240,180)),(708,88))
    text(d,(708,285),'ロボット視点',18,TEAL)
    text(d,(708,335),f'経過 {min(t,float(times[-1]-times[0])):.1f} 秒',18)
    for i,line in enumerate(meta): text(d,(708,385+i*40),line,17)
    text(d,(22,610),'Gazebo実映像 / 走行はシミュレーション時間の1倍速 / 試行間リセット',16,GRAY)
    return im

def main():
    stats=json.loads((OUT/'summary.json').read_text()); plan=[]
    plan.append({'kind':'card','seconds':6,'title':'見えなければ、回り込んでもう一度点検',
        'lines':['画像AIの判断とロボットの観測行動を結びつける',
                 '遮蔽物の裏へ走り、亀裂をもう一度確認する',
                 '本動画は3D物理シミュレータGazeboの実行記録',
                 '実機ではありません / 移動は1倍速']})
    hero=json.loads((OUT/'hero3d_v2.json').read_text())
    specs=[('hero3d_v2.npz','学習した走行方策で、遮蔽物を回り込む','高解像度の3D実映像 / 追加デプロイ seed 7101',
            ['学習制御 BC','最終: '+hero['final_decision'],f"接触通知 {hero['contact_messages']}"],True)]
    for seed in [5100,5101]:
        for mode,name in [('teacher','従来制御'),('bc','模倣学習 BC')]:
            specs.append((f'frames_{seed}_{mode}.npz',f'同じ場面で比較：{name} / seed {seed}',
                '同じ経由点・地図・物理設定 / 新しい評価条件 / 全区間を再生',
                [name,'目標へ到達','本体接触通知 0'],False))
    for file,title,subtitle,meta,is_hero in specs:
        with np.load(OUT/file) as data: record={k:data[k] for k in data.files}
        seconds=math.ceil(float(record['time'][-1]-record['time'][0]))+1
        plan.append({'kind':'clip','seconds':seconds,'title':title,'subtitle':subtitle,
                     'record':record,'meta':meta,'file':file,'hero':is_hero})
    # Show the original recognition-triggered recovery from its real recording.
    rows=[json.loads(l) for l in (ROOT/'output/trials.jsonl').read_text().splitlines()]
    recovery=next(r for r in rows if r['policy']=='selective' and r['scene']['seed']==2301 and
        r['scene']['blocked'] and r['scene']['label'] and r['reobserved'] and r['decision']=='defect')
    name=f"trial_{recovery['scene']['id']}_selective.npz"
    with np.load(ROOT/'output/recordings'/name) as data: record={k:data[k] for k in data.files}
    plan.append({'kind':'clip','seconds':math.ceil(float(record['time'][-1]-record['time'][0]))+1,
        'title':'画像の不確実性をきっかけに、自律的に再観測',
        'subtitle':'既存の認識実験 / この区間の走行は従来制御 / 新規BC比較と区別',
        'record':record,'meta':['初回は判定保留','移動後に亀裂検出',f"{recovery['distance_m']:.2f} m 移動"],
        'file':'original/'+name,'hero':False})
    a=stats['teacher']; b=stats['bc']
    plan.append({'kind':'card','seconds':9,'title':'新規16試行の実測結果と限界',
        'lines':[f"従来制御 {a['reached']}/8 到達 / 模倣学習 {b['reached']}/8 到達",
                 f"平均移動：従来 {a['mean_distance_m']:.2f}m / 学習 {b['mean_distance_m']:.2f}m",
                 '本体接触通知は両方式0 / 小規模・既知地図での評価',
                 '模倣学習の優位性や実世界の安全性は示していない']})
    plan.append({'kind':'rviz','seconds':4})
    plan.append({'kind':'card','seconds':8,'title':'認識の失敗も残して、再現可能にする',
        'lines':['既存144試行：亀裂発見は単一観測8/24、再観測21/24',
                 '確信度だけの方式には、亀裂を正常とする誤りが3件',
                 '可視性ガードで対処 / 未知環境・未知損傷は未検証',
                 'コード・重み・rosbag2・全試行ログ・検査を保存']})
    seconds=sum(s['seconds'] for s in plan); assert 60<=seconds<=180,seconds
    writer=imageio.get_writer(str(OUT/'submission_demo.mp4'),fps=FPS,codec='libx264',quality=8,macro_block_size=16)
    manifest=[]; offset=0; preview=0
    try:
        for seg in plan:
            manifest.append({k:v for k,v in seg.items() if k not in ['record','meta','lines']})
            for k in range(seg['seconds']*FPS):
                if seg['kind']=='clip': im=clip(seg['record'],k/FPS,seg['title'],seg['subtitle'],seg['meta'],seg['hero'])
                elif seg['kind']=='card': im=card(seg['title'],seg['lines'])
                else:
                    im=Image.new('RGB',(960,640),BG)
                    im.paste(Image.open(OUT/'rviz.png').convert('RGB').resize((896,560)),(32,20))
                    text(ImageDraw.Draw(im),(32,602),'RVizの実画面：ROSのカメラ画像・オドメトリを確認',20)
                writer.append_data(np.asarray(im))
                if seg['kind']=='clip' and k==6*FPS:
                    im.save(OUT/f'video_preview_{preview}.png'); preview+=1
            offset+=seg['seconds']
    finally: writer.close()
    count=0
    with imageio.get_reader(str(OUT/'submission_demo.mp4')) as reader:
        for frame in reader:
            assert frame.shape==(640,960,3); count+=1
    assert count==seconds*FPS
    media={'status':'PASS','seconds':seconds,'frames':count,'fps':FPS,'decoded_frames':count,
        'motion_speed':'1x simulation time plus final explanatory holds',
        'three_dimensional_motion_seconds':sum(s['seconds'] for s in plan if s['kind']=='clip'),
        'segments':manifest,'all_robot_motion':'recorded Gazebo external/RGB cameras; no generated robot animation'}
    (OUT/'media.json').write_text(json.dumps(media,ensure_ascii=False,indent=2),encoding='utf-8')
    print('VERIFIED VIDEO',seconds,'seconds',count,'frames')

if __name__=='__main__': main()
