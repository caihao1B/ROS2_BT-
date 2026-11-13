"""Planned reference is distinct from measured position and confirmed credit.

No executed trail is inferred from the reference path. This changes future
recordings only; archived frames and videos retain their original pixels.
"""
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
        im=Image.new('RGB',(250,240),(15,28,40));d=ImageDraw.Draw(im);d.text((10,5),'参考路径 / 实测反馈',font=self.font,fill='#e1edf4')
        # Crop the authorized motion area to the map viewport. A missing edge
        # means that boundary is outside this view, never unlimited permission.
        mask=Image.new('L',im.size,0);md=ImageDraw.Draw(mask)
        geo=self.zone['motion_domain'];polys=[geo['coordinates']] if geo['type']=='Polygon' else geo['coordinates']
        for poly in polys:
            md.polygon([self.point(p) for p in poly[0]],fill=255)
            for hole in poly[1:]:md.polygon([self.point(p) for p in hole],fill=0)
        clip=Image.new('L',im.size,0);clip.paste(mask.crop((13,28,238,179)),(13,28))
        im.paste((27,43,53),(0,0,250,240),clip)
        def dashed(points,color):
            for a,b in zip(points[:-1],points[1:]):
                length=float(np.linalg.norm(np.asarray(b)-a))
                for start in np.arange(0,length,7):
                    u=start/max(length,1e-9);v=min(start+3,length)/max(length,1e-9)
                    d.line([(a[0]+(b[0]-a[0])*u,a[1]+(b[1]-a[1])*u),(a[0]+(b[0]-a[0])*v,a[1]+(b[1]-a[1])*v)],fill=color,width=1)
        if self.paths:
            for poses in self.paths:
                if len(poses)>1:d.line([self.point(p) for p in poses],fill='#6a9bb8',width=2)
        else:
            colors=['#286b68','#725d35','#5a4977']
            for i,f in enumerate(self.zone['partition']['clusters']['features']):d.polygon([self.point(p) for p in f['geometry']['coordinates'][0]],fill=colors[i],outline='#9aaebb')
            for s in self.p['segments']:
                pts=[self.point(p) for p in s.get('poses',[])]
                if len(pts)>1:
                    if s.get('tool_on'):d.line(pts,fill='#77c7ff',width=2)
                    else:dashed(pts,'#899aa6')
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
        metres=next((v for v in [10,5,2,1,.5,.2,.1] if v*self.scale<=55),.1)
        end=18+metres*self.scale;d.rectangle((14,154,max(end+7,68),179),fill='#0f1c28')
        d.line([(18,174),(end,174)],fill='white',width=2);d.text((18,153),f'{metres:g} m',font=self.font,fill='white')
        small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',11)
        d.text((8,183),'蓝点:车辆  橙框:目标  绿面:已确认',font=small,fill='#c0d5e2')
        d.text((8,201),'灰线:计划空驶  蓝线:计划作业/巡检',font=small,fill='#c0d5e2')
        d.text((8,219),'灰底:许可通行区(局部)  非已走轨迹',font=small,fill='#c0d5e2');return im
