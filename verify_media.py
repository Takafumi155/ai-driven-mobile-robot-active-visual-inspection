"""Decode all video frames, validate final PDF, and create review images."""
from pathlib import Path
import json
import imageio.v2 as imageio
import pymupdf
from PIL import Image
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'; REVIEW=ROOT/'review'
def main():
    REVIEW.mkdir(exist_ok=True)
    manifest=json.loads((OUT/'video_manifest.json').read_text()); reader=imageio.get_reader(str(OUT/'gazebo_demo.mp4'))
    meta=reader.get_meta_data(); fps=meta['fps']; n=0
    samples={8,24,42,68,88,106,125}
    for frame in reader:
        assert frame.shape==(640,960,3)
        if n in {round(t*fps) for t in samples}: Image.fromarray(frame).save(REVIEW/f'video_{n//round(fps):03}.png')
        n+=1
    reader.close(); seconds=n/fps
    assert 60<=seconds<=180 and abs(seconds-manifest['seconds'])<.05
    doc=pymupdf.open(OUT/'ros_report.pdf')
    assert 4<=len(doc)<=8
    texts=[]
    for i,page in enumerate(doc):
        texts.append(page.get_text()); assert len(texts[-1])>100
        page.get_pixmap(matrix=pymupdf.Matrix(1,1)).save(str(REVIEW/f'report_{i+1}.png'))
    assert '実測結果' in ''.join(texts)
    result={'status':'PASS','video_seconds':seconds,'decoded_frames':n,'resolution':[960,640],'fps':fps,'pdf_pages':len(doc),'rendered_pdf_pages':len(doc),'note':'Rendered pages and representative video frames require visual review; decoding alone is not visual QA.'}
    (OUT/'media_verification.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
