"""One bounded native TEB warm-start reset after an owned recovery turn."""
import math
import threading
import time


def _client():
    from dynamic_reconfigure.client import Client
    return Client('/move_base/TebLocalPlannerROS', timeout=1.0)


class NativeTebReseed:
    """Reinitialize TEB's own band; never publish a goal or velocity."""
    def __init__(self, owner, client_factory=_client, clock=time.monotonic,
                 sleep=time.sleep, thread_factory=threading.Thread):
        self.owner=owner;self.client_factory=client_factory;self.client=None
        self.clock=clock;self.sleep=sleep;self.thread_factory=thread_factory
        self.lock=threading.Lock();self.busy=False;self.last_identity=None

    def _current(self, identity):
        o=self.owner
        return (o.active_action and o.active_action_identity==identity
                and not o.task_done and not o.navigation_hold)

    def _emit(self, event, identity, **fields):
        with self.owner.lock:
            self.owner.publish_status_locked(event, reseed_action_identity=identity, **fields)

    def request(self, identity):
        with self.lock:
            if self.busy or identity is None or identity==self.last_identity or not self._current(identity):
                return False
            self.busy=True;self.last_identity=identity
        try:
            self.thread_factory(target=self._run,args=(identity,),daemon=True).start()
        except Exception as error:
            with self.lock:self.busy=False
            self._emit('native_teb_reseed_failed',identity,reason=str(error))
            return False
        return True

    def _run(self, identity):
        original=None;attempted=False;applied=0.0
        try:
            if not self._current(identity):return
            if self.client is None:self.client=self.client_factory()
            # Client.get_configuration is a subscriber cache. Query the server
            # atomically so a delayed parameter update cannot defeat restore.
            config=self.client.update_configuration({})
            original=float(config['force_reinit_new_goal_dist'])
            if not math.isfinite(original) or original<=0.0:
                self._emit('native_teb_reseed_skipped',identity,reason='no_positive_warm_start_threshold')
                return
            if not self._current(identity):return
            attempted=True
            updated=self.client.update_configuration({'force_reinit_new_goal_dist':0.0})
            applied=float(updated['force_reinit_new_goal_dist'])
            if applied!=0.0:raise RuntimeError('native planner did not accept reinitialization threshold')
            baseline=self.owner.latest_trajectory_command_wall
            started=self.clock();acknowledged=False
            self._emit('native_teb_reseed_started',identity,previous_threshold=original)
            while self._current(identity) and self.clock()-started<1.0:
                if (self.clock()-started>=.10
                        and self.owner.latest_trajectory_command_wall>baseline
                        and self.owner.trajectory_feedback_valid
                        and self.owner.trajectory_feedback_identity==identity):
                    acknowledged=True;break
                self.sleep(.02)
            self._emit('native_teb_reseed_observed',identity,feedback_acknowledged=acknowledged,
                       action_still_current=self._current(identity))
        except Exception as error:
            self._emit('native_teb_reseed_failed',identity,reason=str(error))
        finally:
            if attempted and original is not None:
                for attempt in range(2):
                    try:
                        current=float(self.client.update_configuration({})['force_reinit_new_goal_dist'])
                        if current not in (0.0,applied):
                            self._emit('native_teb_reseed_restore_skipped',identity,reason='configuration_changed_externally')
                        else:
                            restored=self.client.update_configuration({'force_reinit_new_goal_dist':original})
                            if float(restored['force_reinit_new_goal_dist'])!=original:
                                raise RuntimeError('native warm-start threshold restoration not acknowledged')
                            self._emit('native_teb_reseed_restored',identity,threshold=original)
                        break
                    except Exception as error:
                        if attempt==1:self._emit('native_teb_reseed_restore_failed',identity,reason=str(error))
            with self.lock:self.busy=False
