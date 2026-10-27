"""Atomic control-authority handoff between Brain A (JTC) and Brain B (raw effort)."""
from __future__ import annotations

from controller_manager_msgs.srv import ListControllers, SwitchController

from .robot import call_service
from .scheme import PlatformError


class Brains:
    def __init__(self, node, cfg: dict) -> None:
        cm = cfg['controller_manager']
        self.a = cfg['brain_a']['controller']
        self.b = cfg['brain_b']['controller']
        self.list_cli = node.create_client(ListControllers, f'{cm}/list_controllers')
        self.switch_cli = node.create_client(SwitchController, f'{cm}/switch_controller')

    def states(self, timeout: float = 10.0) -> dict[str, str]:
        res = call_service(self.list_cli, ListControllers.Request(), timeout)
        if res is None:
            raise PlatformError('controller_manager not reachable; is the sim running? (ur5 ctl)')
        return {c.name: c.state for c in res.controller}

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
