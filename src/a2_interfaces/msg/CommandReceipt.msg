# State values otherwise refer to MissionStatus constants.
# A rejected command with no task state uses this sentinel, never READY.
uint8 STATE_UNAVAILABLE=255

uint16 schema_version
string request_id
string mission_id
string controller_session_id
bool accepted
string code
# State at original command acceptance/rejection, not current task state.
uint8 state
