"""Use independently measured regional occupancy in its declared ENU frame."""
from common14 import *
from binding14 import region

if __name__=='__main__':
    atlas,reg=region()
    if reg['cloud_frame']!='site_enu':raise ValueError('REGIONAL_OCCUPANCY_FRAME')
    source=(NAV/'scripts/prior_cost13.py').read_text()
    source=replace_once(source,"BASE/'maps/frozen/navigation.yaml'","ROOT/S14REGION['navigation_yaml']")
    source=replace_once(source,"BASE/'maps/frozen/navigation.pgm'","ROOT/S14REGION['navigation_pgm']")
    # Qualify whole vehicle inside the independently checked region. This is an
    # explicit scope boundary, not an obstacle inferred from simulator truth.
    source=replace_once(source,"  global_pub.publish(global_grid)","  if valid:\n   rr=global_grid.info.resolution;oo=global_grid.info.origin.position;lo,hi=S14REGION['site_extent_xy'];gx=oo.x+(np.arange(global_grid.info.width)+.5)*rr;gy=oo.y+(np.arange(global_grid.info.height)+.5)*rr;xx,yy=np.meshgrid(gx,gy);outside=(xx-rr/2<lo[0])|(xx+rr/2>hi[0])|(yy-rr/2<lo[1])|(yy+rr/2>hi[1]);values=np.asarray(global_grid.data).reshape(global_grid.info.height,global_grid.info.width);values[outside]=100;global_grid.data=values.astype('int8').ravel().tolist()\n  global_pub.publish(global_grid)")
    if '--registered' in sys.argv:raise ValueError('REGIONAL_OCCUPANCY_ALREADY_IN_SITE_FRAME')
    scope=dict(__name__='__main__',__file__=str(NAV/'scripts/prior_cost13.py'),S14REGION=reg)
    exec(compile(source,str(NAV/'scripts/prior_cost13.py'),'exec'),scope)
