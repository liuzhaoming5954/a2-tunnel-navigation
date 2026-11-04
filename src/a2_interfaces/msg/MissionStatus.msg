# Draft v0.2 enum values. These are observations, not movement permissions.
uint8 READY=0
uint8 PRECHECK=1
uint8 LOCALIZING=2
uint8 INSPECTING=3
uint8 WAITING_OBSTACLE=4
uint8 PAUSED_OPERATOR=5
uint8 RETURNING=6
uint8 ALIGNING=7
uint8 COMPLETED_ALIGNED=8
uint8 HANDOFF_PENDING=9
uint8 HANDED_OFF=10
uint8 PAUSED_FAULT=11
uint8 CANCELLED=12
# No current task. Never interpret the default READY value as an idle state.
uint8 STATE_UNAVAILABLE=255

uint16 schema_version
std_msgs/Header header
string robot_id
string boot_id
string controller_session_id
string mission_id
string bundle_id
string map_version
string config_version
string route_id
uint8 state
string reason
string[] completed_visits
# The current mock-only mission adapter sets these to true / false.
# Neither a task state nor a local admission credential authorizes movement.
bool simulated
bool motion_capable
