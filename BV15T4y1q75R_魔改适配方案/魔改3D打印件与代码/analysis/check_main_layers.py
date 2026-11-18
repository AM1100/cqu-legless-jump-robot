from pathlib import Path
import importlib.util,trimesh,numpy as np,json
p=Path(__file__).resolve().parent.parent
spec=importlib.util.spec_from_file_location('ag',p/'analyze_generate.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
m=trimesh.load_mesh(mod.SOURCE/'Main_Gear.stl')
for z in [-7,0,9]:
 a,r,pk=mod.radial(m,z,(178,-41.15));print(z,len(pk),flush=True)
 mask=r>21.788;n=len(r);st=np.where(mask&~np.roll(mask,1))[0];out=[]
 for start in st:
  k=1
  while k<n and mask[(start+k)%n]:k+=1
  out.append(float((start+(k-1)/2)*.05%360))
 print('tip_arc_midpoint_deg',out,flush=True)
