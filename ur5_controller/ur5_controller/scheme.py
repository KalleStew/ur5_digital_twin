"""Interface shared by the position (Brain A) and torque (Brain B) schemes."""
from __future__ import annotations

from dataclasses import dataclass, field

EXIT_OK = 0
EXIT_RUN_FAILED = 1
EXIT_INPUT = 2
EXIT_PLATFORM = 3
EXIT_SAFETY = 4
EXIT_INTERRUPTED = 130


@dataclass
class RunResult:
    ok: bool
    code: int
    reason: str
    outputs: list[str] = field(default_factory=list)
    # (path, telemetry.Recorder, metadata) for the export stage; None when already written.
    export: tuple | None = None

    @classmethod
    def success(cls, reason: str = 'ok', outputs=None) -> 'RunResult':
        return cls(True, EXIT_OK, reason, list(outputs or []))

    @classmethod
    def fail(cls, code: int, reason: str, outputs=None) -> 'RunResult':
        return cls(False, code, reason, list(outputs or []))


class PlatformError(RuntimeError):
    """Platform not ready or a control-authority post-condition failed."""

    code = EXIT_PLATFORM


class InputError(ValueError):
    """Operator input (CSV, YAML, name) rejected before any motion."""

    code = EXIT_INPUT


class Scheme:
    def preflight(self) -> RunResult:
        raise NotImplementedError

    def run(self, spec) -> RunResult:
        raise NotImplementedError

    def abort(self) -> None:
        raise NotImplementedError
