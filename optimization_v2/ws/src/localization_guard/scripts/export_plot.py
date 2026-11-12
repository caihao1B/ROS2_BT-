"""Standalone measured figure. Shows the entire fault sequence, not just the good window."""
import argparse
from support import *
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.font_manager import fontManager,FontProperties

def export(out):
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';fontManager.addfont(font);plt.rcParams['font.family']=FontProperties(fname=font).get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained');colors={'wheel':'#b87611','wheel_imu':'#306ab0','fused':'#00866e'};names={'wheel':'轮速','wheel_imu':'轮速 + IMU','fused':'多源融合'}
    for row,(kind,label) in enumerate([('diff','差速'),('ackermann','阿克曼')]):
        d=read(out/(kind+'_faults')/'plot_data.json');xy,err=axes[row]
        xy.plot(*np.array(d['fused']['truth'])[:,:2].T,color='#34404a',lw=2,label='独立真值')
        for k,c in colors.items():xy.plot(*np.array(d[k]['estimate'])[:,:2].T,color=c,lw=1.5,label=names[k]);err.plot(d[k]['t'],d[k]['error'],color=c,lw=1.5,label=names[k])
        xy.set(title=label+'：完整故障序列轨迹',xlabel='局部 x / m',ylabel='局部 y / m');xy.set_aspect('equal',adjustable='datalim');xy.grid(alpha=.2);xy.legend(fontsize=9)
        err.axvspan(20,50,color='#f1ca63',alpha=.22,label='轮滑窗口');err.axvspan(65,79,color='#df928d',alpha=.2,label='ICP 退化');err.axvspan(86,94,color='#777777',alpha=.15,label='IMU 断流');err.set(title=label+'：全程位置误差',xlabel='仿真时间 / s',ylabel='相对真值的位置误差 / m');err.grid(alpha=.2);err.legend(fontsize=8,ncol=3)
    fig.suptitle('S05 实测：轮滑窗口融合有收益，累计漂移仍需全局定位处理',fontsize=15);fig.savefig(out/'comparison.png',dpi=160);plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();export(a.out.resolve())
