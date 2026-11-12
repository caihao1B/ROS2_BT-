"""Pure task accounting shared by the executor and the runnable Python lesson."""
def confirmed_points(jobs, results, revision):
    required={revision+':'+j['id'] for j in jobs if j['kind']=='POINT_SERVICE'}
    confirmed={r['key'] for r in results if r['key'] in required and r['state']=='SUCCEEDED' and not r['uncertain'] and r['quantity']>=2.}
    return dict(required=len(required),confirmed=len(confirmed),remaining=sorted(required-confirmed))

class FrozenSchedule:
    """Deferral reorders eligibility, never deletes obligations or earns credit."""
    def __init__(self,clusters):
        self.required={k:frozenset(v) for k,v in clusters.items()}
        if sum(map(len,self.required.values()))!=len(set().union(*self.required.values())):raise ValueError('DUPLICATE_OBLIGATION')
        self.done=set();self.deferred=set()
    def defer(self,cluster):
        if cluster not in self.required:raise KeyError(cluster)
        self.deferred.add(cluster)
    def confirm(self,obligation,feedback):
        if obligation not in set().union(*self.required.values()):raise KeyError(obligation)
        if feedback.get('state')=='SUCCEEDED' and not feedback.get('uncertain',True):self.done.add(obligation)
    def snapshot(self):
        return {k:dict(required=len(v),confirmed=len(v&self.done),remaining=sorted(v-self.done),deferred=k in self.deferred) for k,v in self.required.items()}
