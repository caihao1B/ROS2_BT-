"""Read-only R10 spray collision diagnostics for a single running session.

Uses the controller's own warning and published cost/terrain maps. The sampled
footprint is overlaid on the latest published raster. Nav2 checks a private
costmap at control frequency, so this does not identify the exact internal cell.
"""
import argparse
import base64
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import time
from types import SimpleNamespace
import zlib

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.srv import GetCostmap
from std_msgs.msg import String

TOPICS = ('/r06/directive', '/s05/odometry', '/r07/risk_local',
          '/local_costmap/costmap', '/lookahead_collision_arc', '/s09/terrain/cells')


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def cell_at(grid, x, y):
    res = grid.info.resolution
    i = math.floor((x - grid.info.origin.position.x) / res)
    j = math.floor((y - grid.info.origin.position.y) / res)
    if 0 <= i < grid.info.width and 0 <= j < grid.info.height:
        return int(grid.data[j * grid.info.width + i])
    return None


def perimeter_cells(grid, pose, footprint):
    """Rasterized padded footprint edges on the published local grid."""
    res = grid.info.resolution
    x0 = grid.info.origin.position.x
    y0 = grid.info.origin.position.y
    x, y, heading = pose
    co, si = math.cos(heading), math.sin(heading)
    vertices = [
        (math.floor((x + co*u - si*v - x0)/res), math.floor((y + si*u + co*v - y0)/res))
        for u, v in footprint
    ]
    samples = set()
    # Follow the same dominant-axis Bresenham tie choice as nav2_util::LineIterator.
    for (col, row), (end_col, end_row) in zip(vertices, vertices[1:] + vertices[:1]):
        dx, dy = abs(end_col - col), abs(end_row - row)
        sx, sy = 1 if end_col >= col else -1, 1 if end_row >= row else -1
        major, minor = max(dx, dy), min(dx, dy)
        remainder = major // 2
        for _ in range(major + 1):
            samples.add((col, row))
            if dx >= dy:
                col += sx
            else:
                row += sy
            remainder += minor
            if major and remainder >= major:
                if dx >= dy:
                    row += sy
                else:
                    col += sx
                remainder -= major
    return [(x0 + (i+.5)*res, y0 + (j+.5)*res) for i, j in sorted(samples)]


def service_grid(message):
    """Translate the native unsigned cost to the ROS occupancy convention."""
    meta = message.metadata
    lut = bytes([0]+[1+97*(v-1)//251 for v in range(1,253)]+[99,100,255])
    translated = memoryview(bytes(message.data).translate(lut)).cast('b')
    return SimpleNamespace(data=translated, info=SimpleNamespace(
        width=meta.size_x, height=meta.size_y, resolution=meta.resolution, origin=meta.origin))


def grid_patch(grid, pose):
    """A 3 m crop in the original odom grid for offline arc/footprint replay."""
    res = grid.info.resolution
    size = min(32, math.ceil(3. / res))
    gx, gy = grid.info.origin.position.x, grid.info.origin.position.y
    i = math.floor((pose[0]-gx)/res)-size//2
    j = math.floor((pose[1]-gy)/res)-size//2
    if i<0 or j<0 or i+size>grid.info.width or j+size>grid.info.height:
        return None
    raw = memoryview(grid.data).cast('B')
    data = b''.join(raw[(y*grid.info.width+i):(y*grid.info.width+i+size)].tobytes()
                    for y in range(j,j+size))
    return dict(origin=[gx+i*res,gy+j*res], size=size, resolution=res,
                int8_zlib_base64=base64.b64encode(zlib.compress(data,1)).decode('ascii'))


def summarize(grid, risk, pose, footprint):
    counts = Counter()
    bad_cells = {'UNKNOWN': [], 'LETHAL': []}
    for x, y in perimeter_cells(grid, pose, footprint):
        cost = cell_at(grid, x, y)
        if cost is None:
            kind = 'OUTSIDE'
        elif cost < 0:
            kind = 'UNKNOWN'
        elif cost >= 100:
            kind = 'LETHAL'
        elif cost == 99:
            kind = 'INSCRIBED'
        else:
            kind = 'SUB_LETHAL'
        counts[kind] += 1
        if kind in bad_cells and len(bad_cells[kind]) < 8:
            bad_cells[kind].append([round(x, 3), round(y, 3), cell_at(risk, x, y)])
    return dict(edge_cell_counts=dict(counts), unknown=bad_cells['UNKNOWN'], lethal=bad_cells['LETHAL'])


def terrain_near_blocking_cells(raw, pose, bad_cells, stamp_s):
    data = json.loads(raw.data)
    result = {'terrain_t': data['t']}
    if abs(data['t'] - stamp_s) <= 1.25:
        result['terrain_columns'] = data['columns']
        hits = []
        for detail in data['cells']:
            if abs(detail[0]-pose[0]) > 1.4 or abs(detail[1]-pose[1]) > 1.4:
                continue
            if any(math.hypot(detail[0]-x, detail[1]-y) < .075 for x, y, _ in bad_cells):
                hits.append(detail)
                if len(hits) == 12:
                    break
        result['terrain_rows'] = hits
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--wall-seconds',type=float,default=3600)
    a=ap.parse_args()
    spec=json.loads((a.run/'spec.json').read_text())
    cfg=json.loads((a.run/'mission.json').read_text())
    if (spec['task']!='spray' or spec['session']!=a.run.name or
            os.environ.get('ROS_DOMAIN_ID')!='119' or not 0<a.wall_seconds<=5400 or
            (a.run/'runtime.json').exists()):
        raise SystemExit('An active R10 spray session on domain 119 is required')
    import yaml
    policy=Path(__file__).resolve().parents[1]/'editions/R10/spray/nav2.yaml'
    nav=yaml.safe_load(policy.read_text())['local_costmap']['local_costmap']['ros__parameters']
    pad=float(nav['footprint_padding'])
    footprint=[[u+math.copysign(pad,u),v+math.copysign(pad,v)]
               for u,v in json.loads(nav['footprint'])]
    a.out.mkdir(parents=True,exist_ok=True)
    if any((a.out/name).exists() for name in ('summary.json','footprint_samples.jsonl','private_map_samples.jsonl')):
        raise SystemExit('Choose a new output directory to preserve the previous observation.')
    rclpy.init()
    n=rclpy.create_node('r10_readonly_collision_observer',enable_rosout=False,
                        start_parameter_services=False)
    state=dict(session_ok=False,pose=None,odom_t=0.,nav=None,risk=None,arc=None,cells=None)
    counts, fields = Counter(), Counter()
    running = True
    last_sample = -10.
    total = sampled = 0
    began = last_flush = time.monotonic()
    read_cost=n.create_client(GetCostmap,'/local_costmap/get_costmap')
    pending = None
    receipt = a.out / 'summary.json'
    def save(reason):
        doc=dict(session=spec['session'],plan_revision=cfg['version'],domain=119,
                 topics=TOPICS,read_only_service='/local_costmap/get_costmap',observed_warning_lines=total,sampled_warnings=sampled,
                 current_footprint=dict(counts),received_messages=dict(fields),
                 last_sample_sim=last_sample if sampled else None,wall_elapsed_s=round(time.monotonic()-began,2),
                 exit_reason=reason,read_only=True,
                 scope='Time-stamped footprint overlays and bounded read-only Nav2 costmap queries. A sample may differ from the grid/pose used by the preceding controller call.')
        tmp = receipt.with_suffix('.tmp')
        tmp.write_text(json.dumps(doc, indent=2) + '\n')
        tmp.replace(receipt)
    def storing(key):
        def callback(msg):
            state[key] = msg
            fields[key] += 1
        return callback
    def odom(msg):
        p=msg.pose.pose;q=p.orientation
        state.update(pose=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))),odom_t=stamp(msg))
        fields['odom']+=1
    def active(m):
        state['session_ok'] = json.loads(m.data).get('session') == spec['session']
        fields['directive'] += 1
    n.create_subscription(String,TOPICS[0],active,qos_profile_sensor_data)
    n.create_subscription(Odometry,TOPICS[1],odom,qos_profile_sensor_data)
    n.create_subscription(OccupancyGrid,TOPICS[2],storing('risk'),qos_profile_sensor_data)
    n.create_subscription(OccupancyGrid,TOPICS[3],storing('nav'),qos_profile_sensor_data)
    n.create_subscription(NavPath,TOPICS[4],storing('arc'),qos_profile_sensor_data)
    n.create_subscription(String,TOPICS[5],storing('cells'),qos_profile_sensor_data)
    def stop(*_):
        nonlocal running
        running=False
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,stop)
    reason = 'STOPPED'
    warning_file = None
    try:
        with (a.out/'footprint_samples.jsonl').open('x') as out, (a.out/'private_map_samples.jsonl').open('x') as services:
            while running and time.monotonic()-began<a.wall_seconds:
                if (a.run/'runtime.json').exists() or (a.run/'mission_result.json').exists():
                    reason = 'SESSION_ENDED'
                    break
                rclpy.spin_once(n,timeout_sec=.025)
                if pending is not None:
                    future, sent_wall, sample_t, sampled_pose, sampled_risk = pending
                    delay=time.monotonic()-sent_wall
                    if future.done():
                        try:
                            response=future.result()
                        except Exception:
                            fields['private_map_request_error']+=1
                            response=None
                        if response and response.map.header.frame_id=='odom':
                            grid=service_grid(response.map)
                            doc=dict(trigger_t=sample_t,reply_odom_t=state['odom_t'],
                                     map_header_t=stamp(response.map),round_trip_wall_s=delay,
                                     pose=sampled_pose,private_patch=grid_patch(grid,sampled_pose),
                                     current=summarize(grid,sampled_risk,sampled_pose,footprint))
                            services.write(json.dumps(doc,separators=(',',':'))+'\n')
                            services.flush()
                            fields['private_map_response']+=1
                        pending=None
                    elif delay>1.2:
                        read_cost.remove_pending_request(future)
                        fields['private_map_timeout'] += 1
                        pending = None
                if warning_file is None:
                    log=a.run/'controller_server.log'
                    if not log.exists():
                        continue
                    warning_file=log.open()
                    # Sample only live warnings. Never associate pre-attachment lines with current grids.
                    warning_file.seek(0,2)
                lines=warning_file.readlines(65536)
                burst=sum('RegulatedPurePursuitController detected collision ahead' in line for line in lines)
                total+=burst
                now=time.monotonic();sim=state['odom_t'];pose=state['pose'];nav=state['nav'];risk=state['risk']
                if now-last_flush>5:
                    save('OBSERVING')
                    last_flush = now
                if not (burst and state['session_ok'] and pose and nav and sim-last_sample >= .75 and sampled < 2200):
                    continue
                last_sample = sim
                sampled += 1
                if (nav.header.frame_id != 'odom' or risk is None or risk.header.frame_id != 'odom'
                    or not -.1 <= sim-stamp(nav) <= .7 or not -.1 <= sim-stamp(risk) <= .5):
                    counts['GRID_AGE_OR_FRAME'] += 1
                    continue
                row=dict(t=round(sim,3),nav_t=stamp(nav),risk_t=stamp(risk),pose=pose,
                         current=summarize(nav,risk,pose,footprint))
                if pending is None and read_cost.service_is_ready():
                    pending=(read_cost.call_async(GetCostmap.Request()),time.monotonic(),sim,pose,risk)
                row['nav_patch']=grid_patch(nav,pose);row['risk_patch']=grid_patch(risk,pose)
                edge_counts = row['current']['edge_cell_counts']
                if edge_counts.get('LETHAL'):
                    counts['LETHAL'] += 1
                elif edge_counts.get('UNKNOWN'):
                    counts['UNKNOWN'] += 1
                else:
                    counts['NO_BLOCKING_EDGE'] += 1
                arc=state['arc']
                if arc is not None and arc.header.frame_id=='odom':
                    row['last_arc_t']=stamp(arc)
                    row['last_arc_xy']=[[round(p.pose.position.x,3),round(p.pose.position.y,3)] for p in arc.poses[-8:]]
                # A terrain-details frame may be older. Keep age and values instead of inferring an unseen cell.
                raw=state['cells']
                bad=row['current']['lethal']+row['current']['unknown']
                if bad and raw and sampled%4==1:
                    row.update(terrain_near_blocking_cells(raw,pose,bad,sim))
                out.write(json.dumps(row,separators=(',',':'))+'\n')
                out.flush()
            if time.monotonic()-began>=a.wall_seconds:reason='WALL_LIMIT'
    finally:
        if warning_file:
            warning_file.close()
        save(reason)
        n.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
