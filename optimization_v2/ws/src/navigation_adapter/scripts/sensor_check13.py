"""Independent stationary low-obstacle qualification for the near-field branch."""
from common13 import *
def evaluate_sensor(out):
 out=Path(out);runtime=read(out/'runtime.json');log=rows(out/'near_field/events.jsonl');hits=[]
 for path in sorted((out/'cost_adapter').glob('grid_*.npz')):
  d=np.load(path);grid=d['cost'];r=float(d['resolution']);origin=d['origin'];x=origin[0]+(np.arange(grid.shape[1])+.5)*r;y=origin[1]+(np.arange(grid.shape[0])+.5)*r;xx,yy=np.meshgrid(x,y)
  # This is a post-run evaluator's declared fixture ROI, never a perception input.
  roi=(xx>.90)&(xx<1.32)&(abs(yy)<.40)
  if float(d['t'])>10:hits.append(dict(t=float(d['t']),lethal_cells=int(np.count_nonzero(grid[roi]==100))))
 confirmed_frames=0
 for path in (out/'prior_cost').glob('memory_*.npz'):
  d=np.load(path);grid=d['remembered'];r=float(d['resolution']);origin=d['origin'];xx,yy=np.meshgrid(origin[0]+(np.arange(grid.shape[1])+.5)*r,origin[1]+(np.arange(grid.shape[0])+.5)*r);roi=(xx>.90)&(xx<1.32)&(abs(yy)<.40)
  if float(d['t'])>10 and np.count_nonzero(grid[roi]==100)>=2:confirmed_frames+=1
 truth=rows(out/'evaluation/truth.jsonl');motion=max([math.dist(d['xyz'][:2],truth[0]['xyz'][:2]) for d in truth] or [1e9]);contacts=read(out/'evaluation/contacts.json');obstacle_contacts=sum(v for k,v in contacts.items() if 'ground::body::ground' not in k)
 r=dict(confirmed_obstacle_snapshots=confirmed_frames,completed=runtime['success'],max_stationary_displacement_m=motion,obstacle_contacts=obstacle_contacts,near_samples=len(log),near_points_median=float(np.median([d['near_points'] for d in log])) if log else 0,pair_delta_max_s=max([d['pair_delta_s'] for d in log] or [1e9]),low_obstacle_frames=sum(d['lethal_cells']>=2 for d in hits),scored_frames=len(hits),height_m=.08,distance_from_base_m=1.1,source='actual Gazebo optical returns with noise/dropout/delay; independent stationary fixture scoring',database_unchanged=runtime['database_unchanged'],shutdown=runtime.get('tool_shutdown'))
 r['passed']=bool(r['completed'] and motion<.02 and obstacle_contacts==0 and len(log)>50 and r['near_points_median']>500 and r['pair_delta_max_s']<=.055 and r['low_obstacle_frames']>=10 and confirmed_frames>=3 and r['database_unchanged'] and r['shutdown']['state']=='STOPPED');dump(out/'sensor_metrics.json',r);return r
if __name__=='__main__':
 r=evaluate_sensor(sys.argv[1]);print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
