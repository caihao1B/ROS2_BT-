"""Real result documentation generated only from the current stage record."""
from common12 import *

def write_results(out,data):
    docs=ROOT/'docs/optimization_v2/S12';docs.mkdir(parents=True,exist_ok=True)
    lines=['# S12 本轮实测结果','',f'运行 `{data["run_id"]}`，自动 **{data["status"]}**，人工 **NOT_CHECKED**。',data['next_step_verdict'],'',f'[交互报告]({out}/index.html) · [原始指标]({out}/result.json) · [命令退出码]({out}/commands.jsonl) · [浏览器证据]({out}/browser_check.json)','',f'离线计算墙钟 {data["wall_seconds"]:.2f} s，主进程峰值 RSS {data["peak_process_rss_mib"]:.1f} MiB；仿真钟 N/A。以下任务秒数来自路线模型，实际仿真任务 NOT_RUN。', '', '| 区域/簇 | 任务区域/m² | 点数或覆盖率 | 路线/m | 运动/min | 工具/s | 服务停靠/s | 余量/s | 名义/min |','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for z,d in data.get('zones',{}).items():
        for p in d['plans']:
            t=p['timing'];cv=p['coverage_check'];work=str(cv['planned_count'])+' 点' if z=='A' else f'{100*cv["fraction"]:.3f}%';area=d['area_m2'] if z=='A' else cv['domain_area_m2']
            lines.append(f'| {z}{p["cluster"]+1} | {area:.3f} | {work} | {t["length_m"]:.3f} | {t["motion_s"]/60:.3f} | {t["tool_switch_s"]:.2f} | {t["dwell_and_dock_s"]:.2f} | {t["reserve_s"]:.2f} | {t["total_s"]/60:.3f} |')
    lines+=['','A 表格面积为本版整区；B 为各簇覆盖分母。每簇另含 10 s 任务准备。','', '本次采用显式任务改版；旧地图/旧任务保持不变，以下范围未计为完成：']
    for z,d in data.get('zones',{}).items():lines.append(f'- {z}：原区域 {d["original_area_m2"]:.3f} m² → 本版 {d["area_m2"]:.3f} m²；原范围外剩余 {d["original_area_m2"]-d["area_m2"]:.3f} m²；线性尺度 {d["scale"]}，选中反馈迭代 {d["iteration"]}。')
    lines+=['','## 真实搜索与反馈','', '| 区域 | 尺度 | 迭代 | 三簇名义时间/min | 状态 | 反馈改变单元数 |','|---|---:|---:|---|---|---:|']
    for z,attempts in data['attempts'].items():
        for a in attempts:lines.append(f'| {z} | {a["scale"]} | {a["iteration"]} | '+ ' / '.join(f'{t/60:.3f}' for t in a['times_s'])+f' | {a["status"]} | {a["changed_cells"]} |')
    lines+=['','尺寸和有界启发式不是全局最优解；较大任务失败是实验结论，未用放宽速度、曲率或删点掩盖。','', '## 独立巡检路线','', '| 底盘 | 方向 | 长度/m | 名义/min | 验证 |','|---|---|---:|---:|---|']
    for p in data.get('connector_metrics',[]):lines.append(f'| {p["vehicle"]} | {p["direction"]} | {p["timing"]["length_m"]:.3f} | {p["timing"]["total_s"]/60:.3f} | {p["passed"]} |')
    lines+=['','连接路包含进出停车点，独立于单簇作业预算；没有工具作业记账。','', '## 验收与限制','',f'检查项：`{json.dumps(data["checks"],ensure_ascii=False)}`。正常/失败单元测试、真实 C++/Python 教学例、浏览器控制均由 commands.jsonl 留有退出码。', '原整区 INFEASIBLE、狭窄空间两类转弯拒绝、不可达点拒绝、地图/车辆/工具/配置过期及计划正文篡改拒绝，详见 faults/faults.json。', 'S12-C01：全部本版喷洒目标唯一归属；割草足迹并集、边缘/地头/残余明确。S12-C02：逐段完整物理包络、曲率、运动方向、速度与加减速。S12-C03：六簇名义 ≤1440 s，尺寸/反馈有限，原大任务显式不可行。S12-C04：反馈后复验 S11 连通、完整、不重叠；版本变更拒绝。', '', '未安装新依赖。实际版本见 environment_verified.json。原 S01–S11 源码、配置、候选产品保持不变。', '', '在线任务、控制器跟踪、动态避障、实测覆盖和 1800 s 仿真退出：NOT_RUN，留给 S13/S14/S17。S08 既有地图非刚性误差仍需运行时监督。本版包含普通 Dubins 停车转向事件，不能由后续控制器忽略。VALIDATED 仅是离线计划状态，online_execution_allowed=false。']
    if data.get('error'):lines+=['',f'本轮错误：`{data["error"]}`']
    (docs/'RESULTS.md').write_text('\n'.join(lines)+'\n')
