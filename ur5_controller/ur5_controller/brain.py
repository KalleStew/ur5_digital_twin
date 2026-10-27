"""Atomic control-authority handoff between Brain A (JTC) and Brain B (raw effort)."""
from __future__ import annotations

import time

from controller_manager_msgs.srv import ListControllers, SwitchController

from .robot import call_service
from .scheme import PlatformError


class Brains:
    def __init__(self, node, cfg: dict) -> None:
        cm = cfg['controller_manager']
        self.a = cfg['brain_a']['controller']
        self.b = cfg['brain_b']['controller']
        self.jsb = cfg['joint_state_broadcaster']
        self.list_cli = node.create_client(ListControllers, f'{cm}/list_controllers')
        self.switch_cli = node.create_client(SwitchController, f'{cm}/switch_controller')

    def states(self, timeout: float = 10.0) -> dict[str, str]:
        res = call_service(self.list_cli, ListControllers.Request(), timeout)
        if res is None:
            raise PlatformError('controller_manager not reachable; is the sim running? (ur5 ctl)')
        return {c.name: c.state for c in res.controller}

    def wait_ready(self, timeout: float, abort=None) -> None:
        """Block until the broadcaster is active and both brains are loaded and configured (bring-up)."""
        ok = ('active', 'inactive')
        # Wall-clock: give-up timeout only; the spawner chain runs on its own schedule.
        deadline = time.monotonic() + timeout
        last = 'controller_manager not reachable'
        while time.monotonic() < deadline and not (abort and abort.is_set()):
            try:
                st = self.states(timeout=5.0)
                if st.get(self.jsb) == 'active' and st.get(self.a) in ok and st.get(self.b) in ok:
                    return
                last = f'controllers {st}'
            except PlatformError as e:
                last = str(e)
            time.sleep(0.5)
        if abort and abort.is_set():
            return
        raise PlatformError(f'bring-up not ready after {timeout:.0f}s: {last}')

    def active(self) -> str | None:
        """'a', 'b', or None when neither brain holds authority."""
        st = self.states()
        a, b = st.get(self.a) == 'active', st.get(self.b) == 'active'
        if a and b:
            raise PlatformError(f'{self.a} and {self.b} are both active')
        return 'a' if a else 'b' if b else None

    def switch_to(self, brain: str, timeout: float = 10.0) -> bool:
        """Give `brain` exclusive authority in one STRICT switch. Returns False if already held."""
        if brain not in ('a', 'b'):
            raise ValueError(f'brain must be a or b, got {brain!r}')
        on, off = (self.a, self.b) if brain == 'a' else (self.b, self.a)
        st = self.states()
        for name in (on, off):
            if name not in st:
                raise PlatformError(f'{name} is not loaded (ur5 ctl)')
        if st[on] == 'active' and st[off] != 'active':
            return False
        if st[on] not in ('active', 'inactive'):
            raise PlatformError(f'{on} is {st[on]}, expected inactive')
        req = SwitchController.Request()
        req.activate_controllers = [on] if st[on] != 'active' else []
        req.deactivate_controllers = [off] if st[off] == 'active' else []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = int(timeout)
        res = call_service(self.switch_cli, req, timeout + 5.0)
        if res is None:
            raise PlatformError(f'switch_controller to brain {brain} timed out')
        if not res.ok:
            raise PlatformError(f'switch_controller to brain {brain} refused: {res.message}')
        after = self.states()
        if after.get(on) != 'active' or after.get(off) == 'active':
            raise PlatformError(f'post-condition failed after switch to {brain}: {after}')
        return True
