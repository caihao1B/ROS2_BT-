"""Scientific plots of real map surfaces and independent GNSS station residuals."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from binding14 import atlas
from regional_math14 import rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
sys.path.insert(0,str(BASE/'ws/src/map_manager/scripts'))
from mapping_analysis import ply

def main():
    a=atlas();out=BASE/'maps/s14_regions';fig=plt.figure(figsize=(14,10),layout='constrained')
    for index,z in enumerate(['A','B']):
        c=a['regions'][z];R=rotation(c['rotation_yaw_rad']);t=np.array(c['translation_xy_m']);observations=c['train']+c['heldout'];gps=np.array([x['site_antenna_xy'] for x in observations]);corrected=np.array([x['registered_antenna_xy'] for x in observations]);p=(corrected-t)@R
        ax=fig.add_subplot(2,2,index*2+1);ax.scatter(p[:,0],p[:,1],c='#8a94a2',label='Legacy rigid frame reference');ax.scatter(corrected[:,0],corrected[:,1],c='#167c95',label='Independent regional scan match');ax.scatter(gps[:5,0],gps[:5,1],marker='+',s=110,c='#29933d',label='Training GNSS');ax.scatter(gps[5:,0],gps[5:,1],marker='x',s=90,c='#d85c0b',label='Held-out GNSS')
        for i,(old,new) in enumerate(zip(p,corrected)):ax.annotate('',xy=new,xytext=old,arrowprops=dict(arrowstyle='->',color='#8192a3'));ax.annotate(str(i),new,xytext=(5,5),textcoords='offset points',fontsize=8)
        ax.set_title(f'{z}1 independent holdout: max {100*max(c["heldout_residuals_m"]):.2f} cm, RMSE {100*c["heldout_rmse_m"]:.2f} cm');ax.set_xlabel('map / ENU X (m)');ax.set_ylabel('map / ENU Y (m)');ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
        ax=fig.add_subplot(2,2,index*2+2,projection='3d');surface=ply(ROOT/c['cloud_path']);lo,hi=np.array(c['site_extent_xy']);mask=np.all(surface[:,:2]>lo-8,axis=1)&np.all(surface[:,:2]<hi+8,axis=1)&(surface[:,2]>.2);surface=surface[mask];surface=surface[::max(1,len(surface)//7000)]
        ax.scatter(surface[:,0],surface[:,1],surface[:,2],s=.8,c=surface[:,2],cmap='viridis',alpha=.65);ax.scatter(gps[:,0],gps[:,1],np.zeros(len(gps)),c='#d85c0b',s=22);ax.set_title(f'{z}1 registered measured 3-D surfaces');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_zlabel('Sensor height (m)');ax.view_init(30,-60)
    fig.suptitle('S14 regional registration — sensor observations, separate train/holdout stations\nCalibration validation does not imply full mission completion',fontsize=14);fig.savefig(out/'registration_validation.png',dpi=150);plt.close(fig)
    print(out/'registration_validation.png')
    fig,axes=plt.subplots(1,2,figsize=(12,6),layout='constrained');mission=plan()
    for ax,z in zip(axes,['A','B']):
        c=a['regions'][z];grid=yaml.safe_load((ROOT/c['navigation_yaml']).read_text());pixels=np.asarray(Image.open(ROOT/c['navigation_pgm']))[::-1];origin=grid['origin'];res=grid['resolution'];h,w=pixels.shape
        ax.imshow(pixels,origin='lower',extent=[origin[0],origin[0]+w*res,origin[1],origin[1]+h*res],cmap='gray',vmin=0,vmax=254)
        route=np.array([p for s in mission['zones'][z]['plans'][0]['segments'] for p in s['poses']]);route[:,:2]-=[28,14];ax.plot(route[:,0],route[:,1],color='#167c95',linewidth=1,label='Original complete route')
        ax.scatter(*route[0,:2],c='#e06517',s=55,label='Original start');lo,hi=np.array(c['site_extent_xy']);ax.set_xlim(lo[0]-1,hi[0]+1);ax.set_ylim(lo[1]-1,hi[1]+1);ax.set_aspect('equal');ax.set_title(z+'1 measured navigation grid');ax.set_xlabel('ENU X (m)');ax.set_ylabel('ENU Y (m)');ax.legend(fontsize=8)
    fig.suptitle('White: confirmed ground rays; black: repeated obstacle hits; gray: unknown\nFixed task geometry is overlaid, never used to mark free space');fig.savefig(out/'navigation_validation.png',dpi=150);plt.close(fig)
if __name__=='__main__':main()
