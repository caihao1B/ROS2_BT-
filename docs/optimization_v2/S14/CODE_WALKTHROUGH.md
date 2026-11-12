# S14 核心代码导读

生产调用链：stepctl → stage14 → runtime14（受检查的 S13 启动适配）→ C++ BT / Python worker → Nav2 / ToolTask → S10 账本与独立安全门。

## src/mission_bt.cpp

- [class Dispatch:public BT::StatefulActionNode {](../../../optimization_v2/ws/src/mission_bt/src/mission_bt.cpp#L17)
- [BT::NodeStatus update(){](../../../optimization_v2/ws/src/mission_bt/src/mission_bt.cpp#L19)
- [BT::NodeStatus onStart()override{return update();}](../../../optimization_v2/ws/src/mission_bt/src/mission_bt.cpp#L34)
- [BT::NodeStatus onRunning()override{return update();}](../../../optimization_v2/ws/src/mission_bt/src/mission_bt.cpp#L35)
- [void onHalted()override{s_.publish({{"t",s_.node->now().seconds()},{"id",id_},{"action","CANCEL"},{"reason_code","TREE_HALTED"}});}](../../../optimization_v2/ws/src/mission_bt/src/mission_bt.cpp#L36)
- [int main(int argc,char**argv){](../../../optimization_v2/ws/src/mission_bt/src/mission_bt.cpp#L38)

## src/ackermann_planner14.cpp

- [class AckermannPlanner : public nav2_core::GlobalPlanner {](../../../optimization_v2/ws/src/mission_bt/src/ackermann_planner14.cpp#L12)

## include/path_retention.hpp

- [inline double heading(const geometry_msgs::msg::Pose & p) {](../../../optimization_v2/ws/src/mission_bt/include/path_retention.hpp#L8)
- [inline double angle(double x){return std::atan2(std::sin(x),std::cos(x));}](../../../optimization_v2/ws/src/mission_bt/include/path_retention.hpp#L11)
- [inline bool bodyClear(nav2_costmap_2d::Costmap2D & map,](../../../optimization_v2/ws/src/mission_bt/include/path_retention.hpp#L13)

## scripts/navigation_config14.py

- [def compose(kind,out):](../../../optimization_v2/ws/src/mission_bt/scripts/navigation_config14.py#L4)

## include/context_clock.hpp

- [class ContextBuffer {](../../../optimization_v2/ws/src/mission_bt/include/context_clock.hpp#L6)

## include/collision_volume.hpp

- [inline bool inCollisionVolume(double height, double ceiling) {](../../../optimization_v2/ws/src/mission_bt/include/collision_volume.hpp#L5)

## include/slope_observability.hpp

- [inline double slopeSigma(const Eigen::Matrix3d& centered_sum, double sigma_z) {](../../../optimization_v2/ws/src/mission_bt/include/slope_observability.hpp#L7)
- [inline int unobservableSlopeRisk(double height,double reference_slope,const s09::Profile& profile) {](../../../optimization_v2/ws/src/mission_bt/include/slope_observability.hpp#L12)

## include/local_surface.hpp


## tests/local_surface.cpp

- [int main(int argc,char** argv){](../../../optimization_v2/ws/src/mission_bt/tests/local_surface.cpp#L7)

## include/risk_evidence.hpp

- [inline void observeRisk(int candidate, double t, int required_clear_frames,](../../../optimization_v2/ws/src/mission_bt/include/risk_evidence.hpp#L8)
- [inline int riskAt(int stored, double evidence_t, double now, double max_age) {](../../../optimization_v2/ws/src/mission_bt/include/risk_evidence.hpp#L23)

## tests/risk_evidence.cpp

- [int main(int argc, char** argv) {](../../../optimization_v2/ws/src/mission_bt/tests/risk_evidence.cpp#L8)

## scripts/collision_volume14.py

- [def envelope():](../../../optimization_v2/ws/src/mission_bt/scripts/collision_volume14.py#L11)
- [def hazardous(normal_distance, normal_z, ceiling):](../../../optimization_v2/ws/src/mission_bt/scripts/collision_volume14.py#L30)

## include/decision.hpp

- [class Policy {](../../../optimization_v2/ws/src/mission_bt/include/decision.hpp#L7)
- [J choose(const J& e) {](../../../optimization_v2/ws/src/mission_bt/include/decision.hpp#L11)

## include/tool_permit.hpp

- [inline bool toolPermit(const nlohmann::json& tool,const nlohmann::json& lease){](../../../optimization_v2/ws/src/mission_bt/include/tool_permit.hpp#L4)

## scripts/runtime14.py

- [def runtime(probe=False, survey=False, registered=False):](../../../optimization_v2/ws/src/mission_bt/scripts/runtime14.py#L7)
- [def run_case(*args, probe=False, survey=False, **kwargs):](../../../optimization_v2/ws/src/mission_bt/scripts/runtime14.py#L50)

## scripts/worker14.py

- [def adapted_source():](../../../optimization_v2/ws/src/mission_bt/scripts/worker14.py#L6)
- [def record_nav_plan(msg):](../../../optimization_v2/ws/src/mission_bt/scripts/worker14.py#L24)
- [def record_navigation_decision(msg):](../../../optimization_v2/ws/src/mission_bt/scripts/worker14.py#L29)

## scripts/mission14.py

- [class Suspended(RuntimeError):pass](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L17)
- [class Mission:](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L19)
- [def __init__(self,a,st,node,api):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L20)
- [def make_jobs(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L49)
- [def transit_job(self,s,label,reference=False):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L77)
- [def segment_jobs(self,s,label):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L82)
- [def receive(self,m):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L93)
- [def claim_directive(self,d,j):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L96)
- [def log(self,event,**kw):self.api['log'](event,**kw)](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L104)
- [def lease_fields(self):return dict(work=self.working,tool_key=self.key,segment=self.segment)](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L105)
- [def work_permission(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L106)
- [def pulse(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L110)
- [def wait(self,pred,sim=20,wall=80):return self.api['wait'](pred,sim,wall)](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L148)
- [def spin(self):self.api['tick']()](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L149)
- [def tool_stop(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L151)
- [def halt(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L163)
- [def cancel_patch(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L172)
- [def start_patch(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L182)
- [def poll_patch(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L192)
- [def tool_goal(self,j):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L202)
- [def start_tool(self,j):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L214)
- [def collect_tool(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L220)
- [def execute_job(self,j,replan=False):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L231)
- [def save(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L273)
- [def run(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L284)
- [def shutdown(self):](../../../optimization_v2/ws/src/mission_bt/scripts/mission14.py#L329)

## scripts/admission_recovery14.py

- [class AdmissionRecovery:](../../../optimization_v2/ws/src/mission_bt/scripts/admission_recovery14.py#L8)
- [def __init__(self, clear_hold_s, max_block_s, max_resumes=12):](../../../optimization_v2/ws/src/mission_bt/scripts/admission_recovery14.py#L9)
- [def begin(self, t):](../../../optimization_v2/ws/src/mission_bt/scripts/admission_recovery14.py#L18)
- [def update(self, t, qualified):](../../../optimization_v2/ws/src/mission_bt/scripts/admission_recovery14.py#L23)
- [def observation_interruption(reason, platform_ok, perception):](../../../optimization_v2/ws/src/mission_bt/scripts/admission_recovery14.py#L42)

## tests/admission_recovery14.py


## scripts/budget14.py

- [def remaining_arc(poses,xy,prefix_m=None):](../../../optimization_v2/ws/src/mission_bt/scripts/budget14.py#L5)
- [def estimate(job,xy,t,busy,nav_plan=None,live_replan=False):](../../../optimization_v2/ws/src/mission_bt/scripts/budget14.py#L16)

## scripts/accounting14.py

- [def confirmed_points(jobs, results, revision):](../../../optimization_v2/ws/src/mission_bt/scripts/accounting14.py#L2)
- [class FrozenSchedule:](../../../optimization_v2/ws/src/mission_bt/scripts/accounting14.py#L7)
- [def __init__(self,clusters):](../../../optimization_v2/ws/src/mission_bt/scripts/accounting14.py#L9)
- [def defer(self,cluster):](../../../optimization_v2/ws/src/mission_bt/scripts/accounting14.py#L13)
- [def confirm(self,obligation,feedback):](../../../optimization_v2/ws/src/mission_bt/scripts/accounting14.py#L16)
- [def snapshot(self):](../../../optimization_v2/ws/src/mission_bt/scripts/accounting14.py#L19)

## scripts/ledger_geometry14.py

- [def polygon_parts(geometry):](../../../optimization_v2/ws/src/mission_bt/scripts/ledger_geometry14.py#L11)
- [def area_union(credit,footprint,precision):](../../../optimization_v2/ws/src/mission_bt/scripts/ledger_geometry14.py#L20)

## scripts/ledger14.py


## scripts/residual14.py

- [def candidate(cluster, residual_map, pose_map, out, attempt):](../../../optimization_v2/ws/src/mission_bt/scripts/residual14.py#L11)

## scripts/analysis14.py

- [def evaluate(out):](../../../optimization_v2/ws/src/mission_bt/scripts/analysis14.py#L3)
- [def assess(data):](../../../optimization_v2/ws/src/mission_bt/scripts/analysis14.py#L24)

## scripts/checkpoint14.py

- [def complete(case,result):](../../../optimization_v2/ws/src/mission_bt/scripts/checkpoint14.py#L7)
- [def save(out,case,result,binding,dependencies):](../../../optimization_v2/ws/src/mission_bt/scripts/checkpoint14.py#L14)
- [def reuse(parents,out,case,binding,dependencies):](../../../optimization_v2/ws/src/mission_bt/scripts/checkpoint14.py#L20)

## scripts/regional_math14.py

- [def rotation(yaw):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_math14.py#L8)
- [class SurfaceMap:](../../../optimization_v2/ws/src/mission_bt/scripts/regional_math14.py#L11)
- [def __init__(self,points):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_math14.py#L12)
- [def prepare_normals(self):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_math14.py#L15)
- [def align(self,raw,seed,coarse=True):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_math14.py#L20)
- [def rigid_fit(source,target):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_math14.py#L50)

## scripts/regional_localizer14.py

- [def main():](../../../optimization_v2/ws/src/mission_bt/scripts/regional_localizer14.py#L21)
- [def hint(m):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_localizer14.py#L33)
- [def end(s,f):state['stop']=True](../../../optimization_v2/ws/src/mission_bt/scripts/regional_localizer14.py#L37)

## scripts/regional_source14.py

- [def main():](../../../optimization_v2/ws/src/mission_bt/scripts/regional_source14.py#L10)
- [def convert(llh,cov,a):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_source14.py#L13)

## scripts/binding14.py

- [def signatures(paths):](../../../optimization_v2/ws/src/mission_bt/scripts/binding14.py#L9)
- [def canonical(d):return hashlib.sha256(json.dumps(d,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()](../../../optimization_v2/ws/src/mission_bt/scripts/binding14.py#L15)
- [def atlas(validate=True):](../../../optimization_v2/ws/src/mission_bt/scripts/binding14.py#L17)
- [def verify_derived(d,original,a,name):](../../../optimization_v2/ws/src/mission_bt/scripts/binding14.py#L30)
- [def active_plan():](../../../optimization_v2/ws/src/mission_bt/scripts/binding14.py#L52)
- [def region():](../../../optimization_v2/ws/src/mission_bt/scripts/binding14.py#L74)
- [def execution_cluster(kind):](../../../optimization_v2/ws/src/mission_bt/scripts/binding14.py#L79)

## scripts/routing14.py

- [class CollisionMap:](../../../optimization_v2/ws/src/mission_bt/scripts/routing14.py#L14)
- [def __init__(self,observation,extent):](../../../optimization_v2/ws/src/mission_bt/scripts/routing14.py#L15)
- [def valid(self,poses):](../../../optimization_v2/ws/src/mission_bt/scripts/routing14.py#L21)
- [def work_options(zone,raw,phase,collision,c):](../../../optimization_v2/ws/src/mission_bt/scripts/routing14.py#L31)
- [def candidate(zone,raw,phase,collision,c):](../../../optimization_v2/ws/src/mission_bt/scripts/routing14.py#L58)
- [def verify_route(route,original,atlas):](../../../optimization_v2/ws/src/mission_bt/scripts/routing14.py#L100)

## tests/build_routes14.py

- [def main():](../../../optimization_v2/ws/src/mission_bt/tests/build_routes14.py#L14)

## scripts/regional_occupancy14.py

- [def ground_plane(points):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_occupancy14.py#L11)
- [def build(stations,poses,out,zone):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_occupancy14.py#L29)
- [def ids(xy):](../../../optimization_v2/ws/src/mission_bt/scripts/regional_occupancy14.py#L38)

## include/step_reference.hpp

- [template<class V> double planeHeight(const V& mean,const V& normal,const V& point){](../../../optimization_v2/ws/src/mission_bt/include/step_reference.hpp#L5)

## scripts/terrain_source14.py


## tests/build_atlas14.py

- [def station(name,s):](../../../optimization_v2/ws/src/mission_bt/tests/build_atlas14.py#L19)
- [def merged(stations,poses):](../../../optimization_v2/ws/src/mission_bt/tests/build_atlas14.py#L30)
- [def match(s,reference,seed):](../../../optimization_v2/ws/src/mission_bt/tests/build_atlas14.py#L40)
- [def main():](../../../optimization_v2/ws/src/mission_bt/tests/build_atlas14.py#L55)

## 按一条实际请求阅读代码

1. `Mission.pulse()` 将定位准入、剩余时间、当前义务和工具状态发到 `/s14/context`；这些值可以与报告决策证据逐项对照。
2. C++ `Policy.choose()` 先检查失联、安全和预算，再判断等待、重规划或补扫；每次返回结构化 reason_code、输入快照、门限及候选成本。它没有直接发布底盘速度。
3. `Dispatch::update()` 在动作切换时先发送 CANCEL。Python `Mission.halt()` 取消实际导航并确认工具停止，黑板 idle=true 后 C++ 才发新的编号。`onHalted()` 负责树停止时撤销当前执行。
4. `Mission.execute_job()` 将喷洒目标变成 NavigateToPose 与 POINT_SERVICE，将割草工作段变成 FollowPath 与 START_WORK。`work_permission()` 和 `toolPermit()` 共同核验工具 key、工作反馈及段类型。
5. `Mission.collect_tool()` 接收真实 ToolTask 结果；`confirmed_points()` 对成功、数量、重复 key 和不确定状态作检查。覆盖几何采用并集后裁到固定区域，`save()` 同时写任务状态和 residual.geojson。
6. `residual14.candidate()` 只根据实际残余和原合法作业带提出补扫；`FrozenSchedule` 用于教学及调度模型验证，当前生产 worker 仍按单簇执行，不代表六簇调度已完成。
7. `analysis14.assess()` 分开判定 C01–C04。短任务达到95%不会改变完整 B1 的分母；冻结图不准入时 C03 必须失败。

先读 `decisions.jsonl` 的 reason_code 和 evidence，再对照 `client/events.jsonl` 中 observation_admission_restored 或 replan_attempt 与实际 Action 返回；最后核对 `client/mission.json` 与 SQLite。观测失联恢复不应被解释成路径受阻重规划。这样可以区分“策略选择了动作”和“动作真的执行并获得反馈”。

导航、工具与融合上游职责：S13 的 execute/revalidate 负责 ROS Action、地图版本、成本地图准入；S10 action_server.cpp 负责工具物理反馈状态机，ledger_core.py 负责反馈配对、幂等事务与有效足迹。S14 只发目标和许可，不直接替换物理反馈。

C++ 条件/联锁例直接测试生产 header；Python 例直接调用生产确认点计数及保留原簇义务的纯账本模型。异步协议测试是真实 BT 进程与测试黑板，不能当成 Gazebo 作业证据。
