"""Teaching example: retain a monotonic route cursor, never credit requested work."""
import math

def residual(path,index,observed_xy):
    # The production client also binds the plan version and awaits cancellation.
    nearest=min(range(index,len(path)),key=lambda i:math.dist(path[i][:2],observed_xy))
    return nearest,path[nearest:]

if __name__=='__main__':
    route=[(i*.1,0.,0.) for i in range(11)]
    idx,tail=residual(route,0,(.61,.01));assert idx==6 and len(tail)==5
    idx2,tail2=residual(route,idx,(.59,0.));assert idx2>=idx and tail2==tail
    assert tail[0]!=route[0]
    print('PASS: cancelled at 0.61 m; resume index 6; retain 5 poses; credit 0 m² before tool feedback')
