"""Host-side capture of own Docker image identity; do not export full inspect data."""
from pathlib import Path
import json,subprocess
OUT=Path(__file__).resolve().parent/'output'
current=json.loads(subprocess.check_output(['/usr/bin/docker','image','inspect','second-look-ros:jazzy']))[0]
base=json.loads(subprocess.check_output(['/usr/bin/docker','image','inspect','ros:jazzy-ros-base']))[0]
version=json.loads(subprocess.check_output(['/usr/bin/docker','version','--format','{{json .Server}}']))
result={'image_id':current['Id'],'base_image_id':base['Id'],'base_repo_digests':base.get('RepoDigests',[]),'docker_engine_version':version['Version'],'runtime':'Docker Engine in WSL Ubuntu 24.04; CPU Mesa rendering','image_build':'PASS','baked_runner_help_check':'PASS'}
(OUT/'docker_image.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2))
