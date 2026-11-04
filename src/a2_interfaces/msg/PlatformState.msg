# Draft v0.1 observation contract. The readonly publisher sets schema_version=1.
uint16 schema_version

# ROS receive/publication clock, not a claimed vendor acquisition timestamp.
# frame_id identifies the frame of body_velocity, normally base_link.
std_msgs/Header header
string robot_id
string source
string source_session_id

# Per-source-session sample counter. Republishing a sample must not advance it.
uint64 sample_seq
# Local steady-clock time of the latest received sample. Traceability only
# across processes or machines; consumers must not compare unrelated clocks.
uint64 received_mono_ns
uint64 age_ms

# Observation validity is not localization validity or motion authorization.
bool valid
# Always false for the readonly platform node.
bool motion_capable
bool simulated

# Percent in [0, 100]. Invalid/unavailable battery uses NaN and false.
bool battery_valid
float32 battery_percent

# Measured planar base_link vx, vy and yaw rate, never a command.
# velocity_valid covers linear.x, linear.y and angular.z only. Unmeasured
# linear.z, angular.x and angular.y remain NaN. Ignore stale/invalid data.
bool velocity_valid
geometry_msgs/Twist body_velocity
bool mode_valid
uint32 mode

# Diagnostic reason, e.g. NO_DATA, VALID, STALE or INVALID.
string reason
