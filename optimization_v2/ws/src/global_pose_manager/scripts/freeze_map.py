"""Freeze measured S06 products. No simulator meshes or private labels are read."""
import argparse,subprocess,shutil,uuid,datetime
from s07_common import *
from mapping_export import verify_manifest
from mapping_analysis import ply
def freeze():
    receipt=read(BASE/'results/S06/latest.json');summary=read((ROOT/receipt['report_path']).parent/'result.json')
    if receipt['automated_status']!='PASS':raise ValueError('S06_NOT_PASS')
    source=ROOT/summary['cases']['mapping_full']['path'];verify_manifest(source/'map_manifest.json');folder=BASE/'maps/frozen'
    if (folder/'manifest.json').exists():return verify_bundle(folder)
    if folder.exists():raise ValueError('Incomplete freeze directory already exists; inspect before replacing')
    folder.mkdir(parents=True);before=digest(source/'mapping.db')
    shutil.copyfile(source/'mapping.db',folder/'map.db');shutil.copyfile(source/'map_optimized_cloud.ply',folder/'map_cloud.ply');shutil.copyfile(source/'db_graph.json',folder/'pose_graph.json')
    args=['rtabmap-export','--map','--opt','2','--output','navigation','--output_dir',str(folder),str(folder/'map.db')]
    r=subprocess.run(args,capture_output=True,text=True,timeout=180);(folder/'grid_export.log').write_text(r.stdout+r.stderr)
    dump(folder/'grid_export.json',dict(args=args,exit_code=r.returncode))
    if r.returncode:raise RuntimeError('occupancy export failed')
    points=ply(folder/'map_cloud.ply');resolution=.5;origin=np.floor(points[:,:2].min(axis=0)/resolution)*resolution;ij=np.floor((points[:,:2]-origin)/resolution).astype(int);shape=ij.max(axis=0)+1;index=ij[:,1]*shape[0]+ij[:,0];order=np.argsort(index);ids,start,count=np.unique(index[order],return_index=True,return_counts=True);height=np.full((shape[1],shape[0]),np.nan,np.float32);rough=height.copy();counts=np.zeros_like(height,dtype=np.int32)
    for key,at,n in zip(ids,start,count):
        if n<3:continue
        z=np.sort(points[order[at:at+n],2]);lower=z[:max(3,int(n*.3))];y,x=divmod(int(key),int(shape[0]));height[y,x]=np.median(lower);rough[y,x]=np.ptp(lower);counts[y,x]=n
    dy,dx=np.gradient(height,resolution);slope=np.degrees(np.arctan(np.hypot(dx,dy)));np.savez_compressed(folder/'coarse_terrain.npz',height=height,roughness=rough,slope_deg=slope,counts=counts,resolution=resolution,origin=origin)
    from PIL import Image
    image=np.full((*height.shape,3),[110,110,110],np.uint8);valid=np.isfinite(height);image[valid]=np.c_[np.clip(height[valid]*45+35,0,255),np.clip(150-height[valid]*20,0,255),np.full(valid.sum(),100)].astype(np.uint8);Image.fromarray(image[::-1]).save(folder/'coarse_height.png')
    dump(folder/'terrain_description.json',dict(resolution_m=resolution,origin_xy_m=origin.tolist(),observed_cells=int(valid.sum()),unknown='NaN; never filled with prior geometry',method='Lower 30% observed height per cell, minimum 3 points; finite-neighbour slope; preliminary surface, not traversability certification',ground_caveat='Lowest observed return is not guaranteed ground when occluded; S09 must perform online terrain/obstacle reasoning'))
    if before!=digest(source/'mapping.db') or before!=digest(folder/'map.db'):raise RuntimeError('database changed during export')
    files=[dict(path=str(p.relative_to(folder)),sha256=digest(p)) for p in folder.iterdir() if p.is_file()]
    m=dict(schema='S07-FROZEN/1',mode='LOCALIZATION',map_id=str(uuid.uuid4()),frame_epoch=read(source/'map_manifest.json')['frame_epoch'],frame='map',base_frame='base_link',source_map_id=read(source/'map_manifest.json')['map_id'],source_receipt_sha256=digest(BASE/'results/S06/latest.json'),source_database_sha256=before,calibration_revision=digest(BASE/'config/sensors.yaml'),serialization='RTAB-Map 0.23.13',origin=dict(kind='S06 arbitrary local SLAM frame',survey_reference=[28.,14.,0.],registered_to_gnss=False),files=files)
    m['revision']=canonical(m);dump(folder/'manifest.json',m)
    for p in folder.iterdir():
        if p.is_file():p.chmod(0o444)
    return verify_bundle(folder)
if __name__=='__main__':print(json.dumps(freeze(),ensure_ascii=False))
