#!/usr/bin/env python3
"""Read the staged plan and verified receipts; never invent a completed stage."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
PLAN = ROOT / 'optimization_v2/plan/steps.json'

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def contained(path):
    candidate = (ROOT / path).resolve()
    if not candidate.is_relative_to(ROOT.resolve()):
        raise ValueError('路径越出主项目目录')
    return candidate

def stage_id(value):
    number = int(value.upper().removeprefix('S'))
    if not 1 <= number <= 18:
        raise ValueError('步骤必须为 01 至 18')
    return f'S{number:02d}'

def load_plan():
    data = json.loads(PLAN.read_text())
    return data, {s['id']: s for s in data['steps']}

def receipt_path(sid):
    return ROOT / 'optimization_v2/results' / sid / 'latest.json'

def inspect(sid, plan, steps, trail=()):
    if sid in trail:
        return 'INVALID', '步骤依赖形成循环', None
    path = receipt_path(sid)
    if not path.is_file():
        return 'NOT_RUN', '尚无执行回执；不表示功能已实现', None
    data = None
    try:
        data = json.loads(path.read_text())
        if not isinstance(data, dict):
            raise ValueError('回执必须是 JSON 对象')
        if data.get('schema_version') != 1 or data.get('step') != sid:
            raise ValueError('回执 schema 或步骤编号不匹配')
        if data.get('plan_revision') != plan['revision']:
            return 'STALE', '回执对应的计划版本已变化', data
        state = data.get('automated_status')
        if state not in ('PASS', 'FAIL', 'PARTIAL', 'NOT_RUN'):
            raise ValueError('无有效 automated_status')
        if state != 'PASS':
            return state, data.get('reason', '以本轮结果为准'), data
        if not data.get('run_id') or not data.get('checked_at_utc') or data.get('check_exit_code') != 0:
            raise ValueError('缺少运行标识、检查时间或成功退出码')
        required = [c['id'] for c in steps[sid]['checks']]
        if not required or any(data.get('checks', {}).get(k) != 'PASS' for k in required):
            raise ValueError('必需检查项没有全部通过')
        artifacts = data.get('artifacts', [])
        sources = data.get('source_files', [])
        if not sources or not artifacts:
            raise ValueError('缺少源码或证据文件清单')
        expected_docs = {
            f'docs/optimization_v2/{sid}/{name}'
            for name in ('TECHNICAL.md', 'CODE_WALKTHROUGH.md', 'MANUAL_CHECK.md', 'RESULTS.md')
        }
        expected_docs.update({f'optimization_v2/lessons/{sid}/{name}'
                              for name in ('lesson.cpp', 'lesson.py', 'README.md')})
        declared = {str(contained(a['path']).relative_to(ROOT.resolve())) for a in artifacts}
        report = str(contained(data['report_path']).relative_to(ROOT.resolve()))
        if not expected_docs.issubset(declared) or report not in declared:
            raise ValueError('缺少学习验收材料、两种语言教学例或可视报告')
        for item in artifacts + sources:
            p = contained(item['path'])
            if not p.is_file():
                raise ValueError(f'清单文件不存在：{item["path"]}')
            if digest(p) != item.get('sha256'):
                return 'STALE', f'文件已变化：{item["path"]}', data
        for dep in steps[sid]['depends_on']:
            state_dep, why, _ = inspect(dep, plan, steps, (*trail, sid))
            if state_dep != 'PASS':
                return 'STALE', f'前置 {dep} 当前为 {state_dep}：{why}', data
            if data.get('dependency_receipts', {}).get(dep) != digest(receipt_path(dep)):
                return 'STALE', f'前置 {dep} 的回执已变化，需要重验', data
        return 'PASS', '自动检查及文件哈希有效；人工查看单独记录', data
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        return 'INVALID', str(exc), data if isinstance(data, dict) else None

def bounded_run(argv, timeout):
    """Only signal this launched process group, never unrelated ROS/Gazebo jobs."""
    proc = subprocess.Popen(argv, cwd=ROOT, start_new_session=True)
    interrupted = False
    try:
        return proc.wait(timeout=timeout)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        interrupted = True
        for sig, grace in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                proc.poll()
                try:
                    os.killpg(proc.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
        proc.wait(timeout=5)
        return 124
    finally:
        if interrupted:
            print('本次启动的进程已进入有界收尾；请查看步骤结果，超时不算通过。', file=sys.stderr)

def main(argv=None):
    ap = argparse.ArgumentParser(description='18 步计划索引。prompt/manual/status/check 可直接使用；demo/test 由各阶段交付。')
    ap.add_argument('command', choices=['list', 'prompt', 'manual', 'status', 'check', 'next', 'open', 'demo', 'test', 'stop'])
    ap.add_argument('step', nargs='?')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--print-path', action='store_true')
    ap.add_argument('--wall-timeout', type=int)
    ap.add_argument('--sim-seconds', type=int)
    args, extra = ap.parse_known_args(argv)
    plan, steps = load_plan()
    if extra and args.command not in ('demo', 'test', 'stop'):
        ap.error('该命令不接受额外参数')
    sid = stage_id(args.step) if args.step else None
    if args.command not in ('list', 'status') and not sid:
        ap.error('此命令需要步骤编号')
    if args.command == 'next':
        current, why, _ = inspect(sid, plan, steps)
        if current != 'PASS':
            print(f'暂不能执行下一步：{sid} 为 {current}。{why}'); return 1
        number = int(sid[1:]) + 1
        if number > 18:
            print('18 步已完成；没有下一步。'); return 0
        following = f'S{number:02d}'
        for dep in steps[following]['depends_on']:
            state, why, _ = inspect(dep, plan, steps)
            if state != 'PASS':
                print(f'暂不能执行第 {number} 步：前置 {dep} 为 {state}。{why}'); return 1
        print(f'可以执行第 {number} 步：{steps[following]["title"]}。本步及下一步必需前置回执有效。')
        print('人工查看单独记录；该判断不会自动开始下一步。'); return 0
    if args.command in ('list', 'status', 'check'):
        selected = [steps[sid]] if sid else list(steps.values())
        results = []
        for step in selected:
            state, why, data = inspect(step['id'], plan, steps)
            results.append({'step':step['id'], 'title':step['title'], 'status':state, 'reason':why,
                            'depends_on':step['depends_on'], 'manual_status':(data or {}).get('manual_status','NOT_CHECKED')})
        if args.json:
            print(json.dumps(results, ensure_ascii=False, indent=2))
        else:
            for row in results:
                print(f"{row['step']}  {row['title']}  [{row['status']}]  人工={row['manual_status']}")
                if sid: print(row['reason'])
            if not sid: print('执行阶段请在聊天中说“执行第 N 步”；本索引不会自行实施机器人改造。')
        return (0 if results[0]['status']=='PASS' else 3 if results[0]['status']=='NOT_RUN' else 1) if args.command=='check' else 0
    step = steps[sid]
    if args.command == 'prompt':
        print(contained(step['prompt_path']).read_text())
        return 0
    if args.command == 'manual':
        print('以下为阶段交付后可运行的手工验收命令；缺少处理器时会报 NOT_READY。\n')
        print(step['manual_markdown'])
        return 0
    if args.command == 'open':
        path = receipt_path(sid)
        if not path.exists():
            print(f'{sid} NOT_READY：尚未生成本步可视报告。', file=sys.stderr);return 3
        data = json.loads(path.read_text()); report = contained(data['report_path'])
        if not report.is_file():
            print('报告文件不存在', file=sys.stderr);return 3
        print(report)
        if not args.print_path:
            subprocess.run(['xdg-open', str(report)], check=True)
        return 0
    handler = ROOT / 'optimization_v2/stages' / sid / 'run.sh'
    if not handler.is_file():
        print(f'{sid} NOT_READY：阶段执行器尚未交付。先在聊天中说“执行第 {int(sid[1:])} 步”。',file=sys.stderr)
        return 3
    if args.command != 'stop':
        for dep in step['depends_on']:
            state, why, _ = inspect(dep, plan, steps)
            if state != 'PASS':
                print(f'前置 {dep} 未就绪：{state}，{why}',file=sys.stderr);return 4
        if args.wall_timeout is None or not 1 <= args.wall_timeout <= 21600:
            ap.error('demo/test 必须设置 --wall-timeout，范围 1 至 21600 现实秒')
        if args.sim_seconds is None or not 1 <= args.sim_seconds <= 1800:
            ap.error('demo/test 必须设置 --sim-seconds，单场景上限为 1800 仿真秒')
    argv = ['bash', str(handler), args.command]
    if args.sim_seconds is not None: argv += ['--sim-seconds',str(args.sim_seconds)]
    if args.wall_timeout is not None: argv += ['--wall-timeout',str(args.wall_timeout)]
    argv += extra
    return bounded_run(argv, args.wall_timeout if args.command != 'stop' else 30)

if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, KeyError, OSError, subprocess.CalledProcessError) as exc:
        print(f'步骤索引错误：{exc}',file=sys.stderr);sys.exit(2)
