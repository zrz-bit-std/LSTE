"""The explicit lifecycle states for a persistent observation place."""

PLACE_OPEN = "open"
PLACE_READY_TO_EXIT = "ready_to_exit"
PLACE_DORMANT = "dormant"
# The robot left this observed Place to follow a novel branch before every
# local ObservationWorkItem was resolved. Its work remains resumable.
PLACE_SUSPENDED = "suspended"


