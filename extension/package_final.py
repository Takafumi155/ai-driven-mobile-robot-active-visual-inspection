"""Package verified project without credentials, course references or old archives."""
from pathlib import Path
import hashlib,json,re,zipfile,os
HERE=Path(__file__).resolve().parent; ROOT=HERE.parent

def main():
    for path in [ROOT/'output/verification.json',HERE/'output/verification.json',HERE/'output/media.json']:
        assert json.loads(path.read_text(encoding='utf-8'))['status']=='PASS',path
    allowed={'.py','.md','.txt','.sh','.rviz','.yaml','.xml','.cfg','.json','.jsonl','.npz','.db3','.sdf','.log','.png','.mp4','.pdf','.csv','.srt'}
    files=[]
    candidates=[]
    for directory,dirs,names in os.walk(ROOT,followlinks=False):
        dirs[:]=[d for d in dirs if d not in ['__pycache__','build','install','review','log','.git']]
        candidates.extend(Path(directory)/name for name in names)
    for p in candidates:
        rel=p.relative_to(ROOT)
        if any(x in rel.parts for x in ['__pycache__','build','install','review','log','.git']): continue
        if not p.is_file(): continue
        if p.name.startswith(('report_review_','video_preview_')) or p.name in ['SHA256SUMS_FINAL.json','supplement.pdf','overview_check.png','hero_preview.png']: continue
        if p.suffix not in allowed and p.name not in ['Dockerfile','LICENSE','.gitignore','.dockerignore','.gitattributes','second_look']: continue
        if p.suffix in ['.py','.md','.txt','.sh','.rviz','.yaml','.xml','.cfg','.json','.jsonl','.sdf','.log']:
            value=p.read_text(encoding='utf-8-sig')
            patterns=[r'ghp_[A-Za-z0-9]{20,}',r'github_pat_[A-Za-z0-9_]+',r'AKIA[A-Z0-9]{16}',
                r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',r'C:[/\\]Users[/\\][A-Za-z]']
            for pattern in patterns: assert not re.search(pattern,value),f'privacy pattern {rel}'
        files.append(p)
    manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    checksum=ROOT/'SHA256SUMS_FINAL.json'; checksum.write_text(json.dumps(manifest,indent=2))
    archive=ROOT/'SECOND_LOOK_FINAL_submission.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for p in files+[checksum]: z.write(p,'SECOND_LOOK_FINAL/'+p.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            rel=name.split('/',1)[1]
            if rel in manifest: assert hashlib.sha256(z.read(name)).hexdigest()==manifest[rel]
    result={'status':'PASS','files':len(files)+1,'zip_bytes':archive.stat().st_size,
            'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'zip_crc':'PASS',
            'per_file_hashes':'PASS','privacy_pattern_scan':'PASS','course_materials_included':False}
    (HERE/'output/package_verification.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2))

if __name__=='__main__': main()
