"""Small map inset from declared tasks and actual confirmed ledger snapshots."""
from common_showcase import *
from PIL import Image,ImageDraw,ImageFont
import numpy as np

class Inset:
    def __init__(self,spec):
        self.spec=spec;self.plan=read(Path(spec['showcase_plan']));self.zone=self.plan['zones'][spec['cluster'][0]];self.p=self.zone['plans'][int(spec['cluster'][1])-1];self.font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',13);self.paths=[]
        if spec['operation']=='inspection':
            r=next(r for r in self.plan['connector_routes'] if r['vehicle']==spec['vehicle'] and r['direction']==('A_to_B' if spec['vehicle']=='diff' else 'B_to_A'));self.paths=[s['poses'] for s in r['segments']];xy=np.array([p[:2] for s in self.paths for p in s])
        else:
            xy=np.array(self.zone['task_domain']['coordinates'][0]);xy=np.vstack([xy,np.array([p[:2] for s in self.p['segments'] for p in s['poses']])])
        self.lo=xy.min(axis=0)-1;self.hi=xy.max(axis=0)+1;self.scale=min(224/(self.hi[0]-self.lo[0]),150/(self.hi[1]-self.lo[1]))
    def point(self,p,map_frame=False):
        q=np.array(p[:2])+([28,14] if map_frame else [0,0]);return (int(13+(q[0]-self.lo[0])*self.scale),int(178-(q[1]-self.lo[1])*self.scale))
    def render(self,m,ledger):
        im=Image.new('RGB',(250,200),(15,28,40));d=ImageDraw.Draw(im);d.text((10,5),'估计位置 / 反馈确认作业',font=self.font,fill='#e1edf4')
        if self.paths:
            for poses in self.paths:
                if len(poses)>1:d.line([self.point(p) for p in poses],fill='#e6a43b',width=3)
        else:
            for s in self.p['segments']:
                if len(s['poses'])>1:d.line([self.point(p) for p in s['poses']],fill='#4b6274',width=1)
            colors=['#286b68','#725d35','#5a4977']
            for i,f in enumerate(self.zone['partition']['clusters']['features']):d.polygon([self.point(p) for p in f['geometry']['coordinates'][0]],fill=colors[i],outline='#9aaebb')
            credit=ledger.get('credit') or {};polygons=[credit['coordinates']] if credit.get('type')=='Polygon' else credit.get('coordinates',[]) if credit.get('type')=='MultiPolygon' else []
            for poly in polygons:
                if poly and poly[0]:d.polygon([self.point(p,True) for p in poly[0]],fill='#53bf89')
                for hole in poly[1:]:d.polygon([self.point(p,True) for p in hole],fill='#384e5c')
            confirmed={r['key'] for r in ledger.get('tool_results',[]) if r.get('state')=='SUCCEEDED' and r.get('quantity',0)>=2 and not r.get('uncertain',True)}
            for i,s in enumerate(self.p['segments']):
                if s['kind']!='POINT_SERVICE':continue
                x,y=self.point(s['target_xy']);key=self.plan['plan_revision']+':'+self.spec['cluster']+'_'+str(i);d.ellipse((x-3,y-3,x+3,y+3),fill='#50dfa1' if key in confirmed else '#ffb465')
        if m.get('goal_map'):
            x,y=self.point(m['goal_map'],True);d.rectangle((x-3,y-3,x+3,y+3),outline='#ffb465',width=2)
        if m.get('position_map'):
            x,y=self.point(m['position_map'],True);d.ellipse((x-4,y-4,x+4,y+4),fill='#77c7ff',outline='white')
        d.text((10,182),'蓝: 车辆  橙: 目标  绿: 已确认',font=self.font,fill='#c0d5e2');return im
