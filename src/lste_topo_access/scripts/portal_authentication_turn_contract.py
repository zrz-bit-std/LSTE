"""Identity-bound physical yaw and sweep direction for doorway observations."""
import math


def authentication_turn_contract(value, route_id, graph_id, lifecycle_id):
    if not isinstance(value, dict):
        return None
    try:
        expected = (int(route_id), int(graph_id), int(lifecycle_id))
        actual = tuple(int(value[k]) for k in
                       ('route_id', 'graph_transaction_id', 'lifecycle_transaction_id'))
        if min(expected) <= 0 or actual != expected:
            return None
        if value['frame_id'] != 'odom' or value['direction'] not in ('clockwise', 'counterclockwise'):
            return None
        if int(value['node_id']) <= 0 or int(value['branch_id']) <= 0:
            return None
        if not math.isfinite(float(value['target_yaw'])):
            return None
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    return dict(value)


def directed_turn_error(target, current, direction):
    error = math.atan2(math.sin(target-current), math.cos(target-current))
    if direction == 'clockwise' and error > 0.:
        error -= 2.*math.pi
    elif direction == 'counterclockwise' and error < 0.:
        error += 2.*math.pi
    return error


def active_authentication_turn(supervisor):
    if (getattr(supervisor, 'active_action_source', '') != 'global_slam_frontier'
            or getattr(supervisor, 'active_action_route_kind', '') != 'frontier_turn_connector'
            or getattr(supervisor, 'pose_frame', '') != 'odom'):
        return None
    return authentication_turn_contract(
        getattr(supervisor, 'latest_authentication_turn', None),
        getattr(supervisor, 'active_action_route_id', 0),
        getattr(supervisor, 'active_action_graph_transaction_id', 0),
        supervisor._lifecycle_transaction_id_locked())
