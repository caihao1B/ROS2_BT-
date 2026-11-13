"""Bounded exact-stamp terrain/health join; receive order cannot grant validity."""
import math

class TerrainJoin:
    def __init__(self,max_age=.45,capacity=4):
        self.max_age=max_age;self.capacity=capacity;self.grids={};self.health={}
    @staticmethod
    def key(t):
        if not math.isfinite(t) or t<0:raise ValueError('INVALID_STAMP')
        return round(t,9)
    def grid(self,t,kind,payload):
        self.grids[(self.key(t),kind)]=payload
        while len(self.grids)>self.capacity:del self.grids[min(self.grids)]
    def status(self,data):
        self.health[self.key(data['t'])]=dict(data)
        while len(self.health)>self.capacity:del self.health[min(self.health)]
    def take(self,now):
        result=[]
        for (stamp,kind),payload in sorted(list(self.grids.items())):
            age=now-stamp
            if age>self.max_age:del self.grids[(stamp,kind)];continue
            if age<0 or stamp not in self.health:continue
            result.append((payload,kind,self.health[stamp]));del self.grids[(stamp,kind)]
        for t in list(self.health):
            if now-t>self.max_age:del self.health[t]
        return result
