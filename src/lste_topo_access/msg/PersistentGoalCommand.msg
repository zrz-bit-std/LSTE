# Explicit goal ownership for the persistent Navfn/TEB stream.
#
# Do not use PoseStamped.header.seq for this identity: ROS publishers own and
# rewrite that field as a transport sequence counter.

uint8 KIND_CLEAR=0
uint8 KIND_MISSION=1
uint8 KIND_TARGET_REQUEST=2
uint8 KIND_TARGET_INSTALLED=3

uint32 transaction_id
uint8 kind
uint32 route_id
string lifecycle_transaction_id
uint32 action_generation
uint64 map_epoch
uint64 origin_epoch
uint64 projection_epoch
# Graph planner transaction; zero for direct semantic target routes.
uint64 graph_transaction_id
string route_kind
string mission_route_kind
string graph_action
string graph_obligation_kind
geometry_msgs/PoseStamped goal
