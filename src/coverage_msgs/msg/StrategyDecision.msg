# mode:
#   "NONE" "A_WAIT" "B_YIELD" "C_DEFER" "D_REPLAN" "E_SKIP"
string mode

# hard safety
bool stop_now

# soft control
float32 slowdown_ratio

# timing estimates (sec)
float32 t_clear
float32 t_block

# bookkeeping / scheduling
string affected_segment_id
string recommended_cluster_id

# optional poses
geometry_msgs/PoseStamped safe_stop_pose
geometry_msgs/PoseStamped rejoin_pose
