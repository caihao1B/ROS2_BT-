import json
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from coverage_msgs.srv import D2Replan
from coverage_replanner.common import GridSource
from coverage_replanner.runtime_planner import (
    SweepRuntimeConfig,
    generate_sweep_plan,
    recommend_orientations,
    score_plan_candidate,
)

try:
    from nav2_msgs.msg import Costmap  # type: ignore
except Exception:  # pragma: no cover
    Costmap = None  # type: ignore


class D2Advisor(Node):
    def __init__(self):
        super().__init__('coverage_d2_advisor')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap_raw')
        self.declare_parameter('occ_thresh', 50)
        self.declare_parameter('sweep_spacing_m', 0.45)
        self.declare_parameter('sample_step_m', 0.10)
        self.declare_parameter('min_run_length_m', 0.60)
        self.declare_parameter('pose_step_m', 0.20)
        self.declare_parameter('blocked_zone_pad_m', 0.22)
        self.declare_parameter('nominal_speed_mps', 0.35)
        self.declare_parameter('row_segment_target_len_m', 5.0)
        self.declare_parameter('row_segment_min_len_m', 3.2)
        self.declare_parameter('tiny_segment_len_m', 0.80)
        self.declare_parameter('candidate_limit', 3)
        self.declare_parameter('replace_improvement_min', 0.10)
        # Local-window replanning on micro maps: replan only the active neighbourhood, not the whole tail.
        self.declare_parameter('window_max_segments', 4)
        self.declare_parameter('window_forward_span_m', 5.0)
        self.declare_parameter('window_bbox_pad_m', 0.55)
        # Replace acceptance: allow a candidate to win when it substantially reduces blocked exposure,
        # even if it is not dramatically shorter/cleaner than the current tail.
        self.declare_parameter('replace_current_blocked_exposure_min_m', 0.75)
        self.declare_parameter('replace_blocked_relief_min_m', 0.40)
        self.declare_parameter('service_name', '/coverage/d2_advisor/replan')

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
                self.get_logger().warn('[d2_advisor] failed to subscribe costmap topic, fallback to /map only')

        service_name = str(self.get_parameter('service_name').value)
        self._srv = self.create_service(D2Replan, service_name, self._cb_replan)
        self.get_logger().info(f'[d2_advisor] service ready: {service_name}')

    def _cb_map(self, msg: OccupancyGrid):
        if self._grid.grid is None:
            self._grid.update_from_occgrid(msg)

    def _cb_costmap(self, msg):
        self._grid.update_from_costmap(msg)

    def _cfg(self) -> SweepRuntimeConfig:
        return SweepRuntimeConfig(
            occ_thresh=int(self.get_parameter('occ_thresh').value),
            sweep_spacing_m=float(self.get_parameter('sweep_spacing_m').value),
            sample_step_m=float(self.get_parameter('sample_step_m').value),
            min_run_length_m=float(self.get_parameter('min_run_length_m').value),
            pose_step_m=float(self.get_parameter('pose_step_m').value),
            blocked_zone_pad_m=float(self.get_parameter('blocked_zone_pad_m').value),
            row_segment_target_len_m=float(self.get_parameter('row_segment_target_len_m').value),
            row_segment_min_len_m=float(self.get_parameter('row_segment_min_len_m').value),
            tiny_segment_len_m=float(self.get_parameter('tiny_segment_len_m').value),
            nominal_speed_mps=float(self.get_parameter('nominal_speed_mps').value),
        )

    @staticmethod
    def _seg_len_snapshot(seg: Dict[str, Any]) -> float:
        poses = list(seg.get('poses') or [])
        total = 0.0
        for a, b in zip(poses[:-1], poses[1:]):
            try:
                total += math.hypot(float(b['x']) - float(a['x']), float(b['y']) - float(a['y']))
            except Exception:
                continue
        return float(total)

    @staticmethod
    def _seg_to_plan_entry(seg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        poses = list(seg.get('poses') or [])
        if len(poses) >= 2:
            return {
                'id': str(seg.get('seg_id', seg.get('seg_idx', 'seg'))),
                'kind': str(seg.get('kind', 'row')),
                'cluster_id': int(seg.get('cluster_id', 0)),
                'poses': poses,
            }
        p0 = seg.get('start_pose')
        p1 = seg.get('end_pose')
        if p0 and p1:
            return {
                'id': str(seg.get('seg_id', seg.get('seg_idx', 'seg'))),
                'kind': str(seg.get('kind', 'row')),
                'cluster_id': int(seg.get('cluster_id', 0)),
                'poses': [p0, p1],
            }
        return None

    def _ordered_cluster_segments(self, remaining_segments: Sequence[Dict[str, Any]], cluster_id: int) -> List[Dict[str, Any]]:
        segs = [s for s in remaining_segments if int(s.get('cluster_id', cluster_id)) == cluster_id]
        return sorted(segs, key=lambda s: int(s.get('queue_rank', 0)))

    def _window_scope(self, snap: Dict[str, Any], cluster_id: int) -> List[Dict[str, Any]]:
        ordered = self._ordered_cluster_segments(list(snap.get('remaining_segments') or []), cluster_id)
        if not ordered:
            return []
        max_segments = max(1, int(snap.get('window_hint_max_segments', self.get_parameter('window_max_segments').value)))
        forward_span = max(0.8, float(snap.get('window_hint_forward_span_m', self.get_parameter('window_forward_span_m').value)))
        scope: List[Dict[str, Any]] = []
        total = 0.0
        for seg in ordered:
            seg_len = self._seg_len_snapshot(seg)
            if scope and len(scope) >= max_segments and total >= forward_span:
                break
            scope.append(seg)
            total += seg_len
            # Always keep at least two segments when available; otherwise the local D2 scope is too tiny on turn entries.
            if len(scope) < 2:
                continue
            if len(scope) >= max_segments and total >= forward_span:
                break
        return scope

    def _bbox_from_scope(self, scope_segments: Sequence[Dict[str, Any]], fallback_bbox: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        pts: List[Tuple[float, float]] = []
        for seg in scope_segments:
            for p in list(seg.get('poses') or []):
                try:
                    pts.append((float(p['x']), float(p['y'])))
                except Exception:
                    continue
        if not pts:
            return fallback_bbox
        pad = max(0.10, float(self.get_parameter('window_bbox_pad_m').value))
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)

    def _current_plan_from_scope(self, scope_segments: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for seg in scope_segments:
            item = self._seg_to_plan_entry(seg)
            if item is not None:
                out.append(item)
        return out

    def _selector(
        self,
        snap: Dict[str, Any],
        bbox_t: Tuple[float, float, float, float],
        current_segments: Sequence[Dict[str, Any]],
        scope_segments: Sequence[Dict[str, Any]],
    ) -> List[Tuple[str, float]]:
        robot_pose = snap.get('robot_pose') or {'x': bbox_t[0], 'y': bbox_t[1], 'yaw': 0.0}
        region = snap.get('region_summary') or {}
        candidate_limit = int(snap.get('candidate_limit', self.get_parameter('candidate_limit').value))
        raw = recommend_orientations(
            remaining_segments=list(scope_segments),
            bbox=bbox_t,
            robot_pose=robot_pose,
            candidate_limit=max(3, candidate_limit + 2),
        )

        frag = float(region.get('fragment_count', 0.0))
        unk = float(region.get('unknown_ratio', 0.0))
        all_entries_no_route = bool(region.get('all_entries_no_route', False))
        score = []
        for name, ang in raw:
            s = 0.0
            if name.startswith('grid_'):
                s += 0.3
                if frag >= 2.0:
                    s += 1.0
                if unk >= 0.20:
                    s += 0.8
                if all_entries_no_route:
                    s += 0.6
            else:
                s += 0.3
                if frag < 2.0:
                    s += 0.9
                if unk < 0.20:
                    s += 0.4
            if name == 'geom_plan' and len(current_segments) >= 2:
                s += 0.4
            if name == 'geom_major':
                s += 0.2
            score.append((float(-s), str(name), float(ang)))
        score.sort()
        return [(name, ang) for _neg, name, ang in score[: max(1, candidate_limit)]]

    def _build_candidate_plans(
        self,
        *,
        snap: Dict[str, Any],
        bbox_t: Tuple[float, float, float, float],
        cluster_id: int,
        selectors: Sequence[Tuple[str, float]],
        cfg: SweepRuntimeConfig,
        current_segments: Sequence[Dict[str, Any]],
        blocked_zones: Sequence[Dict[str, Any]],
    ) -> List[Tuple[str, List[Dict[str, Any]], Dict[str, float]]]:
        robot_pose = snap.get('robot_pose') or {'x': bbox_t[0], 'y': bbox_t[1], 'yaw': 0.0}
        region = snap.get('region_summary') or {}
        candidates: List[Tuple[str, List[Dict[str, Any]], Dict[str, float]]] = []
        for name, angle_deg in selectors:
            plan_segments = generate_sweep_plan(
                grid=self._grid.grid,
                bbox=bbox_t,
                blocked_zones=blocked_zones,
                cluster_id=cluster_id,
                orientation_deg=float(angle_deg),
                cfg=cfg,
                backend_name=str(name),
            )
            if not plan_segments:
                continue
            score = score_plan_candidate(
                plan_segments,
                robot_pose=robot_pose,
                grid=self._grid.grid,
                region=region,
                cfg=cfg,
                switch_cost=1.0,
                blocked_zones=blocked_zones,
                blocked_zone_pad_m=float(cfg.blocked_zone_pad_m),
                current_plan_segments=current_segments,
            )
            score['orientation_deg'] = float(angle_deg)
            candidates.append((str(name), plan_segments, score))

        if current_segments:
            cur_score = score_plan_candidate(
                current_segments,
                robot_pose=robot_pose,
                grid=self._grid.grid,
                region=region,
                cfg=cfg,
                switch_cost=0.0,
                blocked_zones=blocked_zones,
                blocked_zone_pad_m=float(cfg.blocked_zone_pad_m),
                current_plan_segments=current_segments,
            )
            cur_score['orientation_deg'] = float('nan')
            candidates.append(('current_plan', list(current_segments), cur_score))
        return candidates

    def _cb_replan(self, req: D2Replan.Request, resp: D2Replan.Response):
        try:
            snap = json.loads(req.snapshot_json)
        except Exception as e:
            resp.replace_plan = False
            resp.message = f'bad snapshot_json: {e}'
            return resp

        cluster_id = int(snap.get('cluster_id', 0))
        bbox = snap.get('cluster_bbox') or snap.get('bbox')
        if not bbox or len(bbox) != 4:
            resp.replace_plan = False
            resp.message = 'cluster_bbox missing'
            return resp
        fallback_bbox = (float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
        robot_pose = snap.get('robot_pose') or {'x': fallback_bbox[0], 'y': fallback_bbox[1], 'yaw': 0.0}
        region = snap.get('region_summary') or {}
        cfg = self._cfg()
        scope_segments = self._window_scope(snap, cluster_id)
        if not scope_segments:
            resp.replace_plan = False
            resp.message = 'window scope empty'
            return resp
        scope_seg_indices = [int(s.get('seg_idx', -1)) for s in scope_segments if int(s.get('seg_idx', -1)) >= 0]
        scope_seg_ids = [str(s.get('seg_id', s.get('seg_idx', 'seg'))) for s in scope_segments]
        bbox_t = self._bbox_from_scope(scope_segments, fallback_bbox)
        current_segments = self._current_plan_from_scope(scope_segments)
        blocked_zones = list(snap.get('blocked_zones') or [])
        selectors = self._selector(snap, bbox_t, current_segments, scope_segments)
        candidates = self._build_candidate_plans(
            snap=snap,
            bbox_t=bbox_t,
            cluster_id=cluster_id,
            selectors=selectors,
            cfg=cfg,
            current_segments=current_segments,
            blocked_zones=blocked_zones,
        )
        if not candidates:
            resp.replace_plan = False
            resp.message = 'no candidate plan generated'
            return resp

        ranking = []
        best_backend = 'current_plan'
        best_plan = list(current_segments)
        best_score: Optional[Dict[str, float]] = None
        best_non_current: Optional[Tuple[str, List[Dict[str, Any]], Dict[str, float]]] = None
        relief_candidate: Optional[Tuple[str, List[Dict[str, Any]], Dict[str, float]]] = None
        current_score: Optional[Dict[str, float]] = None
        for backend, plan_segments, score in candidates:
            ranking.append({'backend': backend, 'score': score})
            if backend == 'current_plan':
                current_score = score
            else:
                if best_non_current is None or float(score['J']) < float(best_non_current[2]['J']):
                    best_non_current = (backend, plan_segments, score)
                if relief_candidate is None or float(score.get('blocked_relief_m', 0.0)) > float(relief_candidate[2].get('blocked_relief_m', 0.0)):
                    relief_candidate = (backend, plan_segments, score)
            if best_score is None or float(score['J']) < float(best_score['J']):
                best_backend = backend
                best_plan = plan_segments
                best_score = score

        if current_score is None:
            current_score = score_plan_candidate(
                current_segments,
                robot_pose=robot_pose,
                grid=self._grid.grid,
                region=region,
                cfg=cfg,
                switch_cost=0.0,
                blocked_zones=blocked_zones,
                blocked_zone_pad_m=float(cfg.blocked_zone_pad_m),
                current_plan_segments=current_segments,
            )
            ranking.append({'backend': 'current_plan', 'score': current_score})
            if best_score is None or float(current_score['J']) < float(best_score['J']):
                best_backend = 'current_plan'
                best_plan = list(current_segments)
                best_score = current_score

        assert best_score is not None
        current_exposure = float(current_score.get('blocked_exposure_m', 0.0))
        best_exposure = float(best_score.get('blocked_exposure_m', 0.0))
        blocked_relief = max(0.0, current_exposure - best_exposure)

        exposure_min = float(self.get_parameter('replace_current_blocked_exposure_min_m').value)
        relief_min = float(self.get_parameter('replace_blocked_relief_min_m').value)
        override_reason = ''
        # If the plain J ranking still keeps current_plan at the top, but there exists a window-local
        # candidate that materially reduces blocked exposure, allow that candidate to win.
        if best_backend == 'current_plan' and relief_candidate is not None:
            cand_backend, cand_plan, cand_score = relief_candidate
            cand_relief = float(cand_score.get('blocked_relief_m', 0.0))
            if current_exposure >= exposure_min and cand_relief >= relief_min:
                best_backend = cand_backend
                best_plan = cand_plan
                best_score = cand_score
                best_exposure = float(best_score.get('blocked_exposure_m', 0.0))
                blocked_relief = max(0.0, current_exposure - best_exposure)
                override_reason = 'blocked_relief_override'

        improvement = 0.0
        if float(current_score['J']) > 1e-6:
            improvement = float(current_score['J'] - best_score['J']) / float(current_score['J'])
        hard_d2 = bool(region.get('hard_d2', False))
        replace_due_to_blocked = bool(
            best_backend != 'current_plan'
            and current_exposure >= exposure_min
            and blocked_relief >= relief_min
        )
        replace_due_to_cost = bool(best_backend != 'current_plan' and improvement >= float(self.get_parameter('replace_improvement_min').value))
        replace = bool(best_backend != 'current_plan' and (hard_d2 or replace_due_to_cost or replace_due_to_blocked))

        ranking.sort(key=lambda rec: float(rec['score']['J']))
        plan_json = {
            'frame_id': 'map',
            'coverage_width': float(snap.get('coverage_width', self.get_parameter('sweep_spacing_m').value)),
            'meta': {
                'source': 'd2_advisor',
                'backend': best_backend,
                'selector': selectors,
                'scope_seg_indices': scope_seg_indices,
                'scope_seg_ids': scope_seg_ids,
                'scope_bbox': list(bbox_t),
                'current_blocked_exposure_m': float(current_exposure),
                'best_blocked_exposure_m': float(best_exposure),
                'blocked_relief_m': float(blocked_relief),
                'replace_reason': override_reason or ('blocked_relief' if replace_due_to_blocked else 'improvement' if replace_due_to_cost else 'hard_d2' if hard_d2 else 'no_replace'),
            },
            'segments': best_plan,
        }
        resp.replace_plan = replace
        resp.selected_backend = str(best_backend)
        resp.selected_plan_json = json.dumps(plan_json, ensure_ascii=False)
        resp.ranking_json = json.dumps({
            'current': current_score,
            'selected': best_score,
            'candidates': ranking,
            'selector': selectors,
            'improvement': float(improvement),
            'hard_d2': hard_d2,
            'scope_seg_indices': scope_seg_indices,
            'scope_seg_ids': scope_seg_ids,
            'scope_bbox': list(bbox_t),
            'current_blocked_exposure_m': float(current_exposure),
            'best_blocked_exposure_m': float(best_exposure),
            'blocked_relief_m': float(blocked_relief),
            'replace_due_to_blocked': bool(replace_due_to_blocked),
            'replace_due_to_cost': bool(replace_due_to_cost),
            'override_reason': override_reason,
        }, ensure_ascii=False)
        resp.improvement = float(max(0.0, improvement))
        if replace:
            resp.message = 'ok'
        else:
            resp.message = (
                f'no_replace backend={best_backend} hard_d2={hard_d2} '
                f'improvement={improvement:.3f} blocked_relief={blocked_relief:.3f}'
            )
        return resp


def main():
    rclpy.init()
    node = D2Advisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
