import json
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from coverage_msgs.srv import D1Evaluate
from coverage_replanner.common import GridSource, astar_cost, point_risk_cost, wrap_pi

try:
    from nav2_msgs.msg import Costmap  # type: ignore
except Exception:  # pragma: no cover
    Costmap = None  # type: ignore


def _as_pose_xyyaw(p: Dict[str, Any]) -> Tuple[float, float, float]:
    return float(p['x']), float(p['y']), float(p.get('yaw', 0.0))


def _seg_kind(seg: Dict[str, Any]) -> str:
    return str(seg.get('kind', 'row'))


class D1Advisor(Node):
    def __init__(self):
        super().__init__('coverage_d1_advisor')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap_raw')
        self.declare_parameter('occ_thresh', 50)
        self.declare_parameter('w_p', 1.0)
        self.declare_parameter('w_h', 1.2)
        self.declare_parameter('w_s', 1.5)
        self.declare_parameter('w_r', 1.8)
        self.declare_parameter('w_b', 1.0)
        self.declare_parameter('turn_kind_penalty', 0.35)
        self.declare_parameter('tail_discount', 0.65)
        self.declare_parameter('replace_improvement_min', 0.08)
        self.declare_parameter('candidate_window_segments', 5)
        self.declare_parameter('beam_width', 12)
        self.declare_parameter('top_seed_count', 4)

        self._grid = GridSource()
        qos_map = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value), self._cb_map, qos_map)
        if Costmap is not None:
            try:
                self.create_subscription(Costmap, str(self.get_parameter('costmap_topic').value), self._cb_costmap, 10)
            except Exception:
                self.get_logger().warn('[d1_advisor] failed to subscribe costmap topic, fallback to /map only')

        self._srv = self.create_service(D1Evaluate, '/coverage/d1_advisor/evaluate', self._cb_eval)
        self.get_logger().info('[d1_advisor] service ready: /coverage/d1_advisor/evaluate')

    def _cb_map(self, msg: OccupancyGrid):
        if self._grid.grid is None:
            self._grid.update_from_occgrid(msg)

    def _cb_costmap(self, msg):
        self._grid.update_from_costmap(msg)

    def _edge_cost(
        self,
        a_xy: Tuple[float, float],
        a_yaw: float,
        seg: Dict[str, Any],
        backlog: float,
        order_bias: float,
        *,
        prefix_pos: int,
    ) -> Tuple[float, Dict[str, float]]:
        w_p = float(self.get_parameter('w_p').value)
        w_h = float(self.get_parameter('w_h').value)
        w_s = float(self.get_parameter('w_s').value)
        w_r = float(self.get_parameter('w_r').value)
        w_b = float(self.get_parameter('w_b').value)
        occ_thresh = int(self.get_parameter('occ_thresh').value)

        sx, sy, syaw = _as_pose_xyyaw(seg['start_pose'])
        planner_cost, reachable = astar_cost(self._grid.grid, a_xy, (sx, sy), occ_thresh=occ_thresh)
        heading_cost = abs(wrap_pi(float(syaw) - float(a_yaw)))
        risk = point_risk_cost(self._grid.grid, sx, sy, occ_thresh=occ_thresh)
        kind_pen = float(self.get_parameter('turn_kind_penalty').value) if _seg_kind(seg) == 'turn' else 0.0
        # Slightly discount backlog contribution later in the prefix so the advisor does not
        # overreact to global backlog when only a small local reorder is needed.
        backlog_local = float(backlog) / max(1.0, 1.0 + 0.25 * float(prefix_pos))
        cost = (
            w_p * float(planner_cost)
            + w_h * float(heading_cost)
            + w_s * float(order_bias)
            + w_r * float(risk)
            + w_b * float(backlog_local)
            + float(kind_pen)
        )
        return cost, {
            'planner_cost': float(planner_cost),
            'reachable': 1.0 if reachable else 0.0,
            'heading_cost': float(heading_cost),
            'order_bias': float(order_bias),
            'risk_cost': float(risk),
            'backlog_cost': float(backlog_local),
            'kind_penalty': float(kind_pen),
        }

    def _route_cost(
        self,
        robot_pose: Dict[str, Any],
        route: Sequence[Dict[str, Any]],
        backlog: float,
        current_order_map: Dict[int, int],
        *,
        discount_tail_after: Optional[int] = None,
    ) -> Tuple[float, List[Dict[str, Any]]]:
        if not route:
            return 0.0, []
        cur_xy = (float(robot_pose['x']), float(robot_pose['y']))
        cur_yaw = float(robot_pose.get('yaw', 0.0))
        total = 0.0
        breakdowns: List[Dict[str, Any]] = []
        tail_discount = float(self.get_parameter('tail_discount').value)
        for pos, seg in enumerate(route):
            rank = int(current_order_map.get(int(seg['seg_idx']), pos))
            order_bias = abs(float(rank - pos))
            c, parts = self._edge_cost(cur_xy, cur_yaw, seg, backlog=backlog, order_bias=order_bias, prefix_pos=pos)
            if discount_tail_after is not None and pos >= int(discount_tail_after):
                c *= max(0.10, min(1.0, tail_discount))
                parts['tail_discount'] = float(tail_discount)
            total += float(c)
            parts['seg_idx'] = int(seg['seg_idx'])
            breakdowns.append(parts)
            ex, ey, eyaw = _as_pose_xyyaw(seg['end_pose'])
            cur_xy = (ex, ey)
            cur_yaw = eyaw
        return total, breakdowns

    def _beam_search(
        self,
        robot_pose: Dict[str, Any],
        window: Sequence[Dict[str, Any]],
        backlog: float,
        current_order_map: Dict[int, int],
    ) -> Tuple[List[Dict[str, Any]], float, List[Dict[str, Any]]]:
        if not window:
            return [], 0.0, []

        def start_state_cost(seg: Dict[str, Any]) -> float:
            c, _ = self._edge_cost(
                (float(robot_pose['x']), float(robot_pose['y'])),
                float(robot_pose.get('yaw', 0.0)),
                seg,
                backlog=backlog,
                order_bias=abs(float(current_order_map.get(int(seg['seg_idx']), 0))),
                prefix_pos=0,
            )
            return float(c)

        seeds = sorted(list(window), key=start_state_cost)[: max(1, int(self.get_parameter('top_seed_count').value))]
        beam: List[Tuple[float, List[Dict[str, Any]], List[Dict[str, Any]], Tuple[float, float], float, List[Dict[str, Any]]]] = []
        for seg in seeds:
            c, parts = self._edge_cost(
                (float(robot_pose['x']), float(robot_pose['y'])),
                float(robot_pose.get('yaw', 0.0)),
                seg,
                backlog=backlog,
                order_bias=abs(float(current_order_map.get(int(seg['seg_idx']), 0))),
                prefix_pos=0,
            )
            ex, ey, eyaw = _as_pose_xyyaw(seg['end_pose'])
            remaining = [s for s in window if int(s['seg_idx']) != int(seg['seg_idx'])]
            beam.append((float(c), [seg], remaining, (ex, ey), float(eyaw), [dict(parts, seg_idx=int(seg['seg_idx']))]))

        beam_width = max(1, int(self.get_parameter('beam_width').value))
        while True:
            expanded: List[Tuple[float, List[Dict[str, Any]], List[Dict[str, Any]], Tuple[float, float], float, List[Dict[str, Any]]]] = []
            complete = True
            for cost_so_far, prefix, remaining, cur_xy, cur_yaw, parts_so_far in beam:
                if not remaining:
                    expanded.append((cost_so_far, prefix, remaining, cur_xy, cur_yaw, parts_so_far))
                    continue
                complete = False
                local_scored: List[Tuple[float, Dict[str, Any], Dict[str, Any]]] = []
                pos = len(prefix)
                for seg in remaining:
                    rank = int(current_order_map.get(int(seg['seg_idx']), pos))
                    order_bias = abs(float(rank - pos))
                    c, parts = self._edge_cost(cur_xy, cur_yaw, seg, backlog=backlog, order_bias=order_bias, prefix_pos=pos)
                    local_scored.append((float(c), seg, dict(parts, seg_idx=int(seg['seg_idx']))))
                local_scored.sort(key=lambda t: (t[0], t[1].get('queue_rank', 0)))
                # Mild pruning inside each state keeps search cheap for runtime use.
                for c, seg, parts in local_scored[: max(2, beam_width // 2)]:
                    ex, ey, eyaw = _as_pose_xyyaw(seg['end_pose'])
                    expanded.append((
                        float(cost_so_far) + float(c),
                        prefix + [seg],
                        [s for s in remaining if int(s['seg_idx']) != int(seg['seg_idx'])],
                        (ex, ey),
                        float(eyaw),
                        parts_so_far + [parts],
                    ))
            if complete:
                break
            expanded.sort(key=lambda t: t[0])
            beam = expanded[:beam_width]
        beam.sort(key=lambda t: t[0])
        best = beam[0]
        return best[1], float(best[0]), best[5]

    def _cb_eval(self, req: D1Evaluate.Request, resp: D1Evaluate.Response):
        try:
            snap = json.loads(req.snapshot_json)
        except Exception as e:
            resp.replace_route = False
            resp.message = f'bad snapshot_json: {e}'
            return resp

        robot_pose = snap.get('robot_pose') or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        segs = list(snap.get('remaining_segments') or [])
        if not segs:
            resp.replace_route = False
            resp.message = 'no remaining segments'
            return resp

        # Work on the active cluster only.
        cid = int(snap.get('cluster_id', segs[0].get('cluster_id', -1)))
        same_cluster = [s for s in segs if int(s.get('cluster_id', cid)) == cid]
        if not same_cluster:
            same_cluster = segs

        current_route = sorted(same_cluster, key=lambda x: int(x.get('queue_rank', 0)))
        current_order = [int(s['seg_idx']) for s in current_route]
        current_order_map = {int(s['seg_idx']): int(s.get('queue_rank', i)) for i, s in enumerate(current_route)}
        backlog = float((snap.get('region_summary') or {}).get('cleanup_backlog', snap.get('cleanup_backlog', 0.0)))

        current_cost, current_break = self._route_cost(robot_pose, current_route, backlog, current_order_map)

        window_n = max(2, int(self.get_parameter('candidate_window_segments').value))
        prefix = current_route[:window_n]
        tail = current_route[window_n:]
        best_prefix, prefix_cost, prefix_break = self._beam_search(robot_pose, prefix, backlog, current_order_map)
        best_route = best_prefix + tail
        best_cost, best_break = self._route_cost(
            robot_pose,
            best_route,
            backlog,
            current_order_map,
            discount_tail_after=len(best_prefix),
        )

        # Always keep the current route as a valid candidate.
        if best_cost > current_cost:
            best_route = current_route
            best_cost = current_cost
            best_break = current_break

        improvement = 0.0
        if current_cost > 1e-6:
            improvement = max(0.0, float(current_cost - best_cost) / float(current_cost))

        region = snap.get('region_summary') or {}
        hard_d1 = bool(region.get('hard_d1', False))
        improve_min = float(self.get_parameter('replace_improvement_min').value)
        replace = bool(hard_d1 or improvement >= improve_min)

        entries = []
        for seg in best_route:
            entries.append({
                'seg_idx': int(seg['seg_idx']),
                'pose_idx': int(seg.get('entry_pose_idx', 0)),
                'x': float(seg['start_pose']['x']),
                'y': float(seg['start_pose']['y']),
                'yaw': float(seg['start_pose'].get('yaw', 0.0)),
            })

        resp.replace_route = replace
        resp.ordered_segment_entries_json = json.dumps(entries, ensure_ascii=False)
        resp.improvement = float(improvement)
        resp.score_breakdown_json = json.dumps({
            'cluster_id': cid,
            'current_order': current_order,
            'window_size': len(prefix),
            'best_order': [int(s['seg_idx']) for s in best_route],
            'current_cost': float(current_cost),
            'best_cost': float(best_cost),
            'prefix_cost': float(prefix_cost),
            'current_breakdown': current_break,
            'best_breakdown': best_break,
            'hard_d1': hard_d1,
        }, ensure_ascii=False)
        resp.message = 'ok'
        return resp


def main():
    rclpy.init()
    node = D1Advisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
