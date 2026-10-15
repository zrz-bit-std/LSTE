# Immutable completion contract for one frontier route action.
#
# A PoseStamped alone identifies an endpoint only geometrically. Adjacent
# frontier actions can be closer than a controller acceptance radius, so the
# route ID is the authoritative identity at the bridge/frontier boundary.

uint32 route_id
uint32 transaction_id
uint64 map_epoch
# Graph planner transaction; independent from the GoalManager mission ID.
uint64 graph_transaction_id
# LifecycleManager uses a full UUID1 integer; keep it lossless across ROS.
string lifecycle_transaction_id
uint32 action_generation
string route_kind
string mission_route_kind
string graph_action
string graph_obligation_kind
geometry_msgs/PoseStamped goal
# Actionlib result plus an explicit outcome make this a typed terminal rather
# than a geometry-only completion notice. ``failure`` is authoritative for
# controller-stall handling; ``status`` retains the native action status.
uint8 status
bool failure
string reason
# Snapshot lineage carried across the graph -> mission -> controller boundary.
uint64 origin_epoch
uint64 projection_epoch
