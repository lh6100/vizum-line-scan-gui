uint8 UNKNOWN=0
uint8 OFF=1
uint8 LASER_450=2
uint8 LASER_650=3

std_msgs/Header header
uint8 mode
bool reachable
bool readback_valid
bool watchdog_healthy
bool lease_active
string lease_id
int32 lease_remaining_ms
string fault
