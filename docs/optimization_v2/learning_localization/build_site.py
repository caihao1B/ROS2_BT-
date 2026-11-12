#!/usr/bin/env python3
"""Build an offline learning manual and a narrowly scoped source index.

Reads selected production sources; writes only this documentation directory.
No ROS commands, receipt updates, network access or third-party dependencies.
"""
import ast
import hashlib
import html
import json
import re
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

SOURCES = [
    ('optimization_v2/config/local_estimator.yaml', 'S05 输入掩码与 EKF 状态选择'),
    ('optimization_v2/config/localization_policy.yaml', 'S05 噪声、轮滑、几何门限'),
    ('optimization_v2/ws/src/localization_guard/scripts/runtime.py', '实际启动的 EKF 与 ICP 节点'),
    ('optimization_v2/ws/src/localization_guard/scripts/input_adapter.py', 'IMU 标定、倾斜估计、轮滑降权'),
    ('optimization_v2/ws/src/localization_guard/scripts/cloud_filter.py', '点云过滤与法向几何退化检测'),
    ('optimization_v2/ws/src/localization_guard/scripts/icp_gate.py', '独立激光配准的时间与质量门控'),
    ('optimization_v2/ws/src/localization_guard/scripts/odom_output.py', '公开连续里程计：积分估计速度'),
    ('optimization_v2/ws/src/localization_guard/src/health_guard.cpp', 'C++ 局部定位健康监督'),
    ('optimization_v2/references/S05/ekf.cpp', '上游 EKF 学习副本；predict 与 correct'),
    ('optimization_v2/config/slam_mapping.yaml', 'RTAB-Map 建图、ICP、重力、GTSAM 配置'),
    ('optimization_v2/ws/src/map_manager/scripts/mapping_inputs.py', '建图输入同步与时序几何过滤'),
    ('optimization_v2/ws/src/map_manager/scripts/mapping_runtime.py', '建图模式启动与 TF 职责'),
    ('optimization_v2/config/slam_localization.yaml', '冻结地图只读定位配置'),
    ('optimization_v2/ws/src/global_pose_manager/scripts/localization_preprocessor.py', '位置插值、SLERP、整帧时间对齐'),
    ('optimization_v2/ws/src/global_pose_manager/scripts/localization_quality.py', 'S07 几何观测审核'),
    ('optimization_v2/ws/src/global_pose_manager/scripts/map_refinement.py', 'S07 有界匹配修正'),
    ('optimization_v2/config/gnss_alignment.yaml', 'S08 控制点与 ENU 地图配准'),
    ('optimization_v2/config/localization_supervisor.yaml', 'S08 GNSS/地图监督门限'),
    ('optimization_v2/ws/src/gnss_supervisor/scripts/alignment.py', '坐标转换、刚体 SVD、协方差传播'),
    ('optimization_v2/ws/src/gnss_supervisor/scripts/source_adapter.py', 'GNSS 与地图证据适配'),
    ('optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp', 'C++ 创新检查、来源选择与 map→odom'),
    ('optimization_v2/ws/src/mission_bt/scripts/runtime14.py', '当前区域定位分支的实际选择'),
    ('optimization_v2/ws/src/mission_bt/scripts/regional_localizer14.py', '区域定位：初值、歧义、修正与版本门控'),
    ('optimization_v2/ws/src/mission_bt/scripts/regional_math14.py', 'SE(2) 粗配准、鲁棒点到平面、留出验证'),
    ('optimization_v2/ws/src/mission_bt/scripts/regional_source14.py', 'S14 注册区域 GNSS 输入适配'),
    ('optimization_v2/maps/frozen/manifest.json', 'S07 冻结地图版本契约；仅读取 manifest'),
    ('optimization_v2/maps/s14_regions/manifest.json', 'S14 冻结区域地图版本契约'),
    ('optimization_v2/ws/src/object_fusion/scripts/fusion_node.py', '相机语义、激光与双目深度关联'),
    ('optimization_v2/ws/src/object_fusion/scripts/tracker.py', '障碍物 KF 与匈牙利匹配；不是自身定位'),
    ('optimization_v2/config/s05_dependency_lock.json', 'S05 已安装依赖版本记录'),
    ('optimization_v2/config/s06_dependency_lock.json', 'S06 已安装依赖版本记录'),
    ('optimization_v2/references/S06/README.md', '上游来源与版本差异说明'),
]


def source_index():
    entries = []
    lines = ['# 源码导航', '', '本表由实际文件生成。链接打开源码；函数后的数字是快照时的行号。',
             '在终端使用 `less +行号 文件路径` 或编辑器跳转查看。完整 SHA-256 见 SOURCE_SNAPSHOT.json。', '',
             '| 文件（相对项目根目录） | 作用与阅读入口 |', '| --- | --- |']
    for rel, role in SOURCES:
        p = ROOT / rel
        data = p.read_bytes()
        text = data.decode('utf-8')
        functions = []
        if p.suffix == '.py':
            tree = ast.parse(text)
            functions = sorted([{'name': n.name, 'line': n.lineno} for n in ast.walk(tree)
                                if isinstance(n, (ast.FunctionDef, ast.ClassDef))], key=lambda x:x['line'])
        elif p.suffix == '.cpp':
            for n, line in enumerate(text.splitlines(), 1):
                for name in ['Ekf::predict', 'Ekf::correct', 'mapEvidence', 'gpsEvidence', 'tick']:
                    if re.search(r'\b(?:void|bool)\s+' + re.escape(name) + r'\s*\(', line):
                        functions.append(dict(name=name, line=n))
        entries.append(dict(path=rel, role=role, bytes=len(data), sha256=hashlib.sha256(data).hexdigest(),
                            functions=functions))
        selected = [f for f in functions if f['name'] not in ['main','end','__init__']][:10]
        extra = '；'.join(f"`{f['name']}:{f['line']}`" for f in selected)
        lines.append(f'| [{rel}](../../../{rel}) | {role}' + ('；'+extra if extra else '') + ' |')
    snapshot = dict(schema='LEARNING_SOURCE_SNAPSHOT/1', captured_utc=datetime.now(timezone.utc).isoformat(),
                    project_root_at_capture=str(ROOT), scope='Read-only source inspection, not stage revalidation',
                    files=entries)
    (HERE/'SOURCE_SNAPSHOT.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2)+'\n')
    (HERE/'SOURCE_INDEX.md').write_text('\n'.join(lines)+'\n')
    return entries


def inline(text):
    # Escape by default; recognize only the narrow Markdown used in this manual.
    tokens = {}
    def token(value):
        key = f'\x01{len(tokens)}\x02'; tokens[key] = value; return key
    text = re.sub(r'`([^`]+)`', lambda m:token('<code>'+html.escape(m[1])+'</code>'), text)
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)',
                  lambda m:token('<a href="'+html.escape(m[2], quote=True)+'">'+html.escape(m[1])+'</a>'), text)
    text = html.escape(text)
    text = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', text)
    for k, value in tokens.items(): text = text.replace(k, value)
    return text


def render(md, widgets):
    lines = md.splitlines(); result=[]; toc=[]; i=0
    while i < len(lines):
        s=lines[i].strip()
        if not s: i+=1; continue
        if s.startswith('```'):
            lang=s[3:]; code=[]; i+=1
            while i<len(lines) and not lines[i].startswith('```'): code.append(lines[i]); i+=1
            result.append('<div class="code-wrap"><span class="code-lang">'+html.escape(lang or 'text')+
                          '</span><button type="button" class="copy" aria-label="复制这段代码">复制</button><pre><code>'+
                          html.escape('\n'.join(code))+'</code></pre></div>'); i+=1; continue
        widget=re.fullmatch(r'<!-- widget:(\w+) -->',s)
        if widget: result.append(widgets[widget[1]]); i+=1; continue
        if s.startswith('# '): i+=1; continue  # hero owns the page title
        if s.startswith('## '):
            title=s[3:]; ident='chapter-'+str(len(toc)+1).zfill(2)
            toc.append((ident,title)); result.append(f'<h2 id="{ident}" tabindex="-1">{inline(title)}</h2>'); i+=1; continue
        if s.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].strip().startswith('|'):
                row=[x.strip() for x in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?',x.replace(' ','')) for x in row): rows.append(row)
                i+=1
            result.append('<div class="table-scroll" tabindex="0"><table><thead><tr>'+''.join('<th scope="col">'+inline(x)+'</th>' for x in rows[0])+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+inline(x)+'</td>' for x in row)+'</tr>' for row in rows[1:])+'</tbody></table></div>'); continue
        if s.startswith('> '): result.append('<aside class="note">'+inline(s[2:])+'</aside>'); i+=1; continue
        if re.match(r'^(\d+\. |\- )',s):
            ordered=bool(re.match(r'^\d+\.',s)); tag='ol' if ordered else 'ul';items=[]
            pattern=r'^\d+\. ' if ordered else r'^\- '
            while i<len(lines) and re.match(pattern,lines[i].strip()):
                items.append(re.sub(pattern,'',lines[i].strip()));i+=1
            result.append('<'+tag+'>'+''.join('<li>'+inline(x)+'</li>' for x in items)+'</'+tag+'>');continue
        paragraph=[s];i+=1
        while i<len(lines) and lines[i].strip() and not re.match(r'^(#|\||```|<!--|> |\d+\. |\- )',lines[i]):
            paragraph.append(lines[i].strip());i+=1
        result.append('<p>'+inline(' '.join(paragraph))+'</p>')
    return '\n'.join(result),toc


def main():
    entries=source_index()
    widgets=json.loads((HERE/'widgets.json').read_text())
    content,toc=render((HERE/'README.md').read_text(),widgets)
    index,_=render((HERE/'SOURCE_INDEX.md').read_text(),widgets)
    nav=''.join('<a href="#'+ident+'">'+html.escape(title)+'</a>' for ident,title in toc)
    page=(HERE/'template.html').read_text().replace('{{NAV}}',nav).replace('{{CONTENT}}',content).replace('{{INDEX}}',index)
    (HERE/'index.html').write_text(page)
    print(f'Built index.html; {len(toc)} chapters; {len(entries)} source files indexed; production files untouched.')


if __name__ == '__main__':main()
