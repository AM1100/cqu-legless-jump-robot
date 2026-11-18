#!/usr/bin/env python3
"""Reproduce mesh/BRep metrology and geometry prototypes; units mm, no physical test claims."""
from pathlib import Path
import csv, hashlib, json, math, re, tempfile
import numpy as np
import trimesh
import cadquery as cq
from scipy.signal import find_peaks
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence,TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
SOURCE=ROOT/'开源参考仓库/Printables_156544_Jumping_Robot'
OUT=HERE/'stl'; REPORT=HERE/'analysis'
OUT.mkdir(exist_ok=True); REPORT.mkdir(exist_ok=True)
SCALE=0.60
DENSITIES={'PLA':1.24,'PETG':1.27} # explicit assumptions, not material certificates

def dump(path,obj): path.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')
def metrics(m):
    counts=np.bincount(m.edges_unique_inverse)
    v=abs(float(m.volume))/1000
    return dict(bbox_xyz_mm=m.extents.tolist(),bounds_mm=m.bounds.tolist(),vertices=len(m.vertices),triangles=len(m.faces),watertight=bool(m.is_watertight),winding_consistent=bool(m.is_winding_consistent),nonmanifold_edges=int(np.sum(counts!=2)),signed_volume_cm3=float(m.volume)/1000,surface_cm2=float(m.area)/100,solid_mass_est_g={k:v*d for k,d in DENSITIES.items()},mass20_uniform_lower_model_g={k:v*d*.2 for k,d in DENSITIES.items()},mass20_shell08_rough_g={k:d*(min(v,m.area*.8/1000)+.2*max(0,v-m.area*.8/1000)) for k,d in DENSITIES.items()})

def step_import():
    doc=TDocStd_Document(TCollection_ExtendedString('CAD'))
    reader=STEPCAFControl_Reader();reader.SetNameMode(True)
    assert int(reader.ReadFile(str(SOURCE/'Jumping_Robot_V2_.step')))==1
    assert reader.Transfer(doc)
    st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());roots=TDF_LabelSequence();st.GetFreeShapes(roots)
    rows=[];shapes={};cylinders={}
    def visit(label,path=''):
        attr=TDataStd_Name();name=attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),attr) else ''
        shape=cq.Shape.cast(st.GetShape_s(label));bbox=shape.BoundingBox()
        rows.append(dict(name=name,path=path+'/'+name,solids=len(shape.Solids()),bbox_xyz_mm=[bbox.xlen,bbox.ylen,bbox.zlen],volume_cm3=shape.Volume()/1000,valid=shape.isValid()))
        shapes[name.strip()]=shape
        if name.strip() in ['Main Gear','Second Gear','Servo Gear','Motor Gear 1','Rack']:
            vals=set()
            for f in shape.Faces():
                adaptor=BRepAdaptor_Surface(f.wrapped)
                if adaptor.GetType()==GeomAbs_Cylinder:
                    c=adaptor.Cylinder()
                    vals.add((round(c.Radius(),8),tuple(round(x,8) for x in c.Location().Coord()),tuple(round(x,8) for x in c.Axis().Direction().Coord())))
            cylinders[name.strip()]=[dict(radius_mm=a,origin_mm=b,axis=c) for a,b,c in sorted(vals)]
        components=TDF_LabelSequence()
        if st.GetComponents_s(label,components):
            for i in range(1,components.Length()+1):
                ref=TDF_Label()
                if st.GetReferredShape_s(components.Value(i),ref):visit(ref,path+'/'+name)
                else:visit(components.Value(i),path+'/'+name)
    for i in range(1,roots.Length()+1):visit(roots.Value(i))
    return rows,shapes,cylinders

def radial(mesh,z,center):
    segments=trimesh.intersections.mesh_plane(mesh,[0,0,1],[0,0,z])[:,:,:2]-np.asarray(center)
    aa=segments[:,0];vv=segments[:,1]-aa
    angle=np.linspace(0,2*np.pi,7200,endpoint=False);result=[]
    numerator=aa[:,0]*vv[:,1]-aa[:,1]*vv[:,0]
    for block in np.array_split(angle,60):
        direction=np.array([np.cos(block),np.sin(block)]).T
        den=direction[:,0,None]*vv[None,:,1]-direction[:,1,None]*vv[None,:,0]
        t=np.divide(numerator[None,:],den,out=np.full(den.shape,np.nan),where=np.abs(den)>1e-10)
        q=np.divide(aa[None,:,0]*direction[:,None,1]-aa[None,:,1]*direction[:,None,0],den,out=np.full(den.shape,np.nan),where=np.abs(den)>1e-10)
        result.extend(np.max(np.where((t>=0)&(q>=0)&(q<=1),t,0),axis=1))
    r=np.asarray(result);indices,_=find_peaks(np.tile(r,3),prominence=.3,distance=35)
    peaks=indices[(indices>=len(r))&(indices<2*len(r))]-len(r)
    return angle,r,peaks

def rack_features(mesh):
    segments=trimesh.intersections.mesh_plane(mesh,[0,0,1],[0,0,0])[:,:,:2]
    top=segments[:,:,1].max()
    intervals=sorted((min(a[0],b[0]),max(a[0],b[0])) for a,b in segments if abs(a[1]-top)<1e-5 and abs(b[1]-top)<1e-5)
    merged=[]
    for a,b in intervals:
        if merged and a<=merged[-1][1]+1e-4:merged[-1][1]=max(b,merged[-1][1])
        else:merged.append([float(a),float(b)])
    tips=[(a+b)/2 for a,b in merged if .2<b-a<5]
    pitch=np.diff(tips)
    return dict(section_z_mm=0,tooth_tip_plateaus_x_mm=merged,periodic_teeth_count=len(tips),tooth_centers_x_mm=tips,pitch_mean_mm=float(np.mean(pitch)),pitch_min_mm=float(min(pitch)),pitch_max_mm=float(max(pitch)),first_to_last_tooth_center_mm=tips[-1]-tips[0],nominal_main_active_teeth=10,nominal_loading_stroke_mm=10*float(np.mean(pitch)),actual_usable_stroke='TBD: requires assembled motion/contact test; rack length is not stroke')

def export_cq(shape,path):
    assert shape.val().isValid() and len(shape.val().Solids())==1
    cq.exporters.export(shape,str(path),tolerance=.015,angularTolerance=.08)

def new_parts():
    # All new parts are one solid; standard-size holes are NOT scaled with references.
    bar=cq.Workplane('XY').box(36,6,4,centered=(False,True,False)).union(cq.Workplane('XY').center(32,0).box(10,54,4,centered=(True,True,False))).union(cq.Workplane('XY').circle(5).extrude(6))
    for x,y,r in [(0,0,1.6),(29,-20,1.1),(35,-20,1.1),(29,20,1.1),(35,20,1.1),(15,0,1.1)]:
        bar=bar.cut(cq.Workplane('XY').workplane(offset=-1).center(x,y).circle(r).extrude(9))
    # Inclined contact support: does not by itself guarantee horizontal force or yaw.
    slope=math.tan(math.radians(15))
    wedge=cq.Workplane('XZ').polyline([(-7,0),(7,0),(7,4+7*slope),(-7,4-7*slope)]).close().extrude(6,both=True)
    for x in [-3,3]:wedge=wedge.cut(cq.Workplane('XY').workplane(offset=-1).center(x,0).circle(1.1).extrude(10))
    mirror=wedge.mirror('YZ',union=False)
    base=cq.Workplane('XY').box(34,28,3,centered=(True,True,False))
    for y in [-7.2,7.2]:base=base.union(cq.Workplane('XY').center(0,y).box(26,2,6,centered=(True,True,False)).translate((0,0,3)))
    front=cq.Workplane('XY').center(13,0).box(2,16.4,10.4,centered=(True,True,False)).translate((0,0,3))
    base=base.union(front)
    shaft=cq.Workplane('YZ').center(0,8.2).circle(1.7).extrude(40,both=True)
    base=base.cut(shaft)
    for x in [-14,14]:
        for y in [-11,11]:base=base.cut(cq.Workplane('XY').workplane(offset=-1).center(x,y).circle(1.6).extrude(20))
    for y in [-9.4,9.4]:base=base.cut(cq.Workplane('XY').workplane(offset=-1).center(0,y).rect(3,1.6).extrude(5))
    vals={'Pitch_Prop_TBar':bar,'Yaw_Skew_Pad':wedge,'Yaw_Skew_Pad_Mirrored':mirror,'N20_Mount_Adapter':base}
    for name,shape in vals.items():
        export_cq(shape,OUT/(name+'.stl'))
        cq.exporters.export(shape,str(OUT/(name+'.step')))
    return vals

def preview(paths,dest):
    cols=4;rows=math.ceil(len(paths)/cols);fig,axes=plt.subplots(rows,cols,figsize=(16,rows*4),squeeze=False)
    for ax,path in zip(axes.flat,paths):
        m=trimesh.load_mesh(path);tri=m.triangles
        # angled orthographic basis helps see through holes and cradles
        u=np.array([.8,-.6,0]);v=np.array([.3,.4,.8660254]);n=np.cross(u,v)
        visible=m.face_normals@n>0
        projected=np.stack([tri@u,tri@v],axis=-1)[visible]
        depth=(m.triangles_center@n)[visible];order=np.argsort(depth)
        ax.add_collection(PolyCollection(projected[order],facecolor='#83b5cc',edgecolor='#42657a',linewidth=.08))
        ax.autoscale();ax.set_aspect('equal');ax.set_title(path.stem,fontsize=8);ax.grid(alpha=.15)
    for ax in axes.flat[len(paths):]:ax.axis('off')
    fig.tight_layout();fig.savefig(dest,dpi=130);plt.close(fig)

def main():
    rows,shapes,cyl=step_import();dump(REPORT/'step_named_shapes.json',rows);dump(REPORT/'step_cylinders.json',cyl)
    originals=[];meshes={}
    mapping={'Back':'Back','Frame_V2':'Frame V2','Main_Gear':'Main Gear','Second_Gear':'Second Gear','Servo_Gear':'Servo Gear','Motor_Gear':'Motor Gear 1','Rack':'Rack'}
    for f in sorted(SOURCE.glob('*.stl')):
        m=trimesh.load_mesh(f);meshes[f.stem]=m
        d=dict(file=f.name,sha256=hashlib.sha256(f.read_bytes()).hexdigest(),**metrics(m))
        if f.stem in mapping:d['step_volume_cm3']=shapes[mapping[f.stem]].Volume()/1000
        originals.append(d)
    assert len(originals)==13
    dump(REPORT/'original_stl_metrics.json',originals)
    with (REPORT/'original_stl_metrics.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['file','X_mm','Y_mm','Z_mm','signed_volume_cm3','watertight','nonmanifold_edges','PLA_solid_est_g','PETG_solid_est_g','PLA_uniform20_lower_g','PLA_20_shell08_rough_g'])
        for d in originals:w.writerow([d['file'],*d['bbox_xyz_mm'],d['signed_volume_cm3'],d['watertight'],d['nonmanifold_edges'],d['solid_mass_est_g']['PLA'],d['solid_mass_est_g']['PETG'],d['mass20_uniform_lower_model_g']['PLA'],d['mass20_shell08_rough_g']['PLA']])
    features={};spec=[('Main_Gear',-13,(178,-41.15)),('Main_Gear',0,(178,-41.15)),('Second_Gear',-25,(148.8,-35.15)),('Second_Gear',-13,(148.8,-35.15)),('Servo_Gear',-25,(119.40654,-32.72241)),('Motor_Gear',-62,(139,-47.5))]
    fig,axes=plt.subplots(2,3,figsize=(16,8),subplot_kw={'projection':'polar'})
    for ax,(name,z,c) in zip(axes.flat,spec):
        angle,r,peaks=radial(meshes[name],z,c)
        key=name+'_z'+str(z);features[key]=dict(center_xy_mm=c,section_z_mm=z,section_rmin_mm=float(min(r)),section_rmax_mm=float(max(r)),detected_teeth=len(peaks),tip_angles_deg=np.degrees(angle[peaks]).tolist(),tip_angles_note='Raw local maxima on rounded tooth tips; not unique tooth center phases or a safe-stop window')
        np.savetxt(REPORT/(key+'_radial.csv'),np.column_stack([np.degrees(angle),r]),delimiter=',',header='angle_deg,radius_mm',comments='')
        ax.plot(angle,r,lw=.7);ax.plot(angle[peaks],r[peaks],'o',ms=2);ax.set_title(key+' / '+str(len(peaks))+' peaks',fontsize=10)
    fig.tight_layout();fig.savefig(REPORT/'gear_sections.png',dpi=140);plt.close(fig)
    rack=rack_features(meshes['Rack']);features['Rack']=rack
    features['gearing_inference']=dict(loading_input_teeth=14,intermediate_large_teeth=20,intermediate_small_teeth=7,main_nominal_teeth=14,main_central_active_teeth=10,main_missing_pitch_slots=4,nominal_missing_angle_deg=360*4/14,loading_ratio=(20/14)*(14/7),rack_transverse_module_mm=rack['pitch_mean_mm']/math.pi,normal_module_large_pair_inferred_mm=1.5,helix_large_pair_inferred_deg=30,normal_module_main_pair_inferred_mm=2,helix_main_pair_inferred_deg=45,wheel_motor_pinion_teeth=7,wheel_motor_pinion_module_inferred_mm=1.25,note='Module/helix inferred from tip/root radii, tooth count, pitch and axial section phase; not embedded CAD parameters. Safe stop angle is not established.')
    dump(REPORT/'transmission_features.json',features)
    exports=[]
    retained=['Main_Gear','Second_Gear','Motor_Gear','Rack','Frame_V2','Back','Spring_Tensioner_1','Spring_Tensioner_2','Servo_Gear','Wheel_1','Wheel_2','Wheel_Spacer','brake']
    for name in retained:
        m=meshes[name].copy();origin='original STL'
        if name=='Back':
            with tempfile.TemporaryDirectory() as td:
                relief=cq.Solid.makeCylinder(.4,11,cq.Vector(-16,-71.45,24.35),cq.Vector(1,0,0))
                back=shapes['Back'].cut(relief)
                tmp=Path(td)/'back.stl';cq.exporters.export(back,str(tmp),tolerance=.015,angularTolerance=.08);m=trimesh.load_mesh(tmp)
            origin='STEP Back remeshed with R0.4mm relief along original tangent/nonmanifold edge X=-15..-6,Y=-71.45,Z=24.35; after scale relief R0.24mm'
        m.apply_scale(SCALE)
        translation=np.array([-m.bounds[:,0].mean(),-m.bounds[:,1].mean(),-m.bounds[0,2]])
        m.apply_translation(translation);dest=OUT/(name+'_S060.stl');m.export(dest)
        exports.append(dict(file=dest.name,source=name+'.stl',source_route=origin,scale=SCALE,print_translation_after_scale_mm=translation.tolist(),role='geometry fit prototype; original mechanism retained, not compliance-certified assembly',**metrics(trimesh.load_mesh(dest))))
    # N20 input: retain correct Servo_Gear tooth geometry, fill servo holes, then create D bore.
    gear=trimesh.load_mesh(OUT/'Servo_Gear_S060.stl')
    shift=next(d['print_translation_after_scale_mm'] for d in exports if d['file']=='Servo_Gear_S060.stl')
    center=np.array([119.40654,-32.72241])*SCALE+np.array(shift[:2]);height=gear.extents[2]
    hub=trimesh.creation.cylinder(radius=5.2,height=height,sections=96);hub.apply_translation([*center,height/2])
    gear=trimesh.boolean.union([gear,hub],engine='manifold')
    dcut=cq.Workplane('XY').circle(1.6).extrude(height+2).intersect(cq.Workplane('XY').center(-4.45,0).box(11.1,10,height+2,centered=(True,True,False))).translate((center[0],center[1],-1))
    with tempfile.TemporaryDirectory() as td:
        f=Path(td)/'cut.stl';cq.exporters.export(dcut,str(f),tolerance=.008);cutter=trimesh.load_mesh(f)
    gear=trimesh.boolean.difference([gear,cutter],engine='manifold');dest=OUT/'Servo_Gear_S060_N20_D3p2.stl';gear.export(dest)
    exports.append(dict(file=dest.name,source='Servo_Gear_S060.stl; hub filled and D bore recut',role='N20 input geometry prototype; D flat x=+1.1mm, circle diameter3.2mm; verify actual shaft',**metrics(trimesh.load_mesh(dest))))
    new_parts()
    for name in ['Pitch_Prop_TBar','Yaw_Skew_Pad','Yaw_Skew_Pad_Mirrored','N20_Mount_Adapter']:
        exports.append(dict(file=name+'.stl',source='parametric CadQuery geometry in analyze_generate.py',role='new fit/contact prototype; no loaded assembly verification',**metrics(trimesh.load_mesh(OUT/(name+'.stl')))))
    for d in exports:
        assert d['watertight'] and d['winding_consistent'] and d['nonmanifold_edges']==0,d['file']
        assert d['signed_volume_cm3']>0,d['file']
        m=trimesh.load_mesh(OUT/d['file']);assert len(m.split())==1,d['file']
        d['sha256']=hashlib.sha256((OUT/d['file']).read_bytes()).hexdigest()
    dump(REPORT/'export_manifest.json',exports)
    dump(REPORT/'run_summary.json',dict(scale=SCALE,densities_g_cm3=DENSITIES,source_stl_count=13,step_total_solids=rows[0]['solids'],step_bbox_xyz_mm=rows[0]['bbox_xyz_mm'],step_volume_cm3=rows[0]['volume_cm3'],export_stl_count=len(exports),source_step_sha256=hashlib.sha256((SOURCE/'Jumping_Robot_V2_.step').read_bytes()).hexdigest(),metrology_note='Geometry-derived, not physical weighing. STL unitless: mm follows matching STEP millimetre units.',original_retained8_volume_cm3=sum(abs(meshes[n].volume)/1000 for n in retained[:8]),printed_mass_actual_g=None,assembly_validation='not performed'))
    preview([OUT/d['file'] for d in exports],REPORT/'exported_parts_preview.png')
    for d in originals:print(d['file'],'bbox',np.round(d['bbox_xyz_mm'],4),'V',round(d['signed_volume_cm3'],5),'watertight',d['watertight'])
    print('STEP solids',rows[0]['solids'],'exported',len(exports),'rack',rack)
    print('All exports: one connected positive-volume watertight consistently-wound manifold')
if __name__=='__main__':main()
