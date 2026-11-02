"""Positive exact owned-release evidence (cancellation ACK is not task success) shared by offline acceptance tools."""
from pathlib import Path
import hashlib
SOURCE_SHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()

IDENTITY_FIELDS=('route_id','transaction_id','graph_transaction_id',
                 'lifecycle_transaction_id','action_generation',
                 'map_epoch','origin_epoch','projection_epoch')


def route_retired(data):
    route=data.get('active_execution_route_id',data.get('route_id',0))
    released=data.get('released_controller_route') or {}
    ack=data.get('last_controller_lease_release_ack') or {}
    return (data.get('active') is False and data.get('pending') is False
            and route not in (None,0) and released.get('route_id')==route
            and released.get('controller_pending') is False
            and not data.get('pending_controller_lease_releases')
            and not data.get('awaiting_controller_lease_release_ack',False)
            and all(released.get(k)is not None and released.get(k)==ack.get(k)
                    for k in IDENTITY_FIELDS))


def execution_identity(data):
    route=data.get('active_execution_route_id',data.get('route_id',0))
    released=data.get('released_controller_route') or {}
    generation=data.get('action_generation')
    if data.get('active') is not True and released.get('route_id')==route:
        generation=released.get('action_generation',generation)
    return route,generation
