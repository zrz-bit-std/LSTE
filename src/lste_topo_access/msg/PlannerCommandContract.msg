# Identity-bearing command emitted by the persistent local planner.
#
# The underlying TEB FeedbackMsg and move_base cmd_vel stream are legacy
# identity-free messages. This companion message binds each command to the
# exact lifecycle/route/action generation that authorized it.

uint8 STATE_ZERO=0
uint8 STATE_ACTIVE=1
uint8 STATE_INVALIDATED=2

uint32 transaction_id
uint32 route_id
uint64 graph_transaction_id
uint64 map_epoch
uint64 origin_epoch
uint64 projection_epoch
string lifecycle_transaction_id
uint32 action_generation
uint64 planner_sequence
geometry_msgs/Twist command
uint8 state
string reason
string producer
