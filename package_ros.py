"""Package only reviewable ROS/Gazebo artifacts; no private credentials or course PDFs."""
from pathlib import Path
import json,hashlib,zipfile,re
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'
def main():
    verification=json.loads((OUT/'verification.json').read_text()); media=json.loads((OUT/'media_verification.json').read_text())
    assert verification['status']=='PASS' and verification['trials']==144 and media['status']=='PASS'
    files=[]
    for p in ROOT.iterdir():
        if p.is_file() and (p.suffix in ['.py','.md','.txt','.xml','.cfg','.sh'] or p.name in ['Dockerfile','LICENSE','.dockerignore','.gitattributes','.gitignore']): files.append(p)
    files+=list((ROOT/'resource').glob('*'))
    files+=sorted(p for p in OUT.rglob('*') if p.is_file() and p.suffix in ['.json','.jsonl','.png','.npz','.sdf','.log','.mp4','.pdf','.csv','.srt'])
    patterns=[re.escape('C:'+chr(92)+'Users'+chr(92)),r'ghp_[A-Za-z0-9]{20,}',r'github_pat_[A-Za-z0-9_]+',r'AKIA[A-Z0-9]{16}',r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----']
    for p in files:
        if p.suffix in ['.py','.md','.txt','.xml','.cfg','.sh','.json','.jsonl','.sdf','.log']:
            text=p.read_text(encoding='utf-8-sig')
            for pattern in patterns: assert not re.search(pattern,text,re.I),'Privacy pattern found in '+p.name
    sums={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (ROOT/'SHA256SUMS_ROS.json').write_text(json.dumps(sums,indent=2))
    zpath=ROOT/'SECOND_LOOK_ROS_submission.zip'
    with zipfile.ZipFile(zpath,'w',zipfile.ZIP_DEFLATED) as z:
        for p in files+[ROOT/'SHA256SUMS_ROS.json']: z.write(p,'SECOND_LOOK_ROS/'+p.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(zpath) as z: assert z.testzip() is None
    report={'status':'PASS','packaged_files':len(files)+1,'zip_bytes':zpath.stat().st_size,'privacy_pattern_scan':'PASS','crc':'PASS','course_pdf_included':False,'virtualenv_included':False}
    print(json.dumps(report,indent=2))
if __name__=='__main__': main()
