# P2PRange measurement enriched with the known world position of the
# destination (anchor) node

uint8 RADIO=0

uint8 TAG=0
uint8 BASE=1

std_msgs/Header header
uint8 radiation_type
uint64 source_id
uint8 source_type
uint64 destination_id
uint8 destination_type
float32 rssi
float32 range
float32 variance
geometry_msgs/PointStamped position
