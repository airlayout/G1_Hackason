"""Select deployment independently from the mapping algorithm."""

from __future__ import annotations

from .config import Settings


def get_runtime(settings: Settings):
    if settings.runtime == "native":
        from .native import NativeRuntime

        return NativeRuntime(settings)
    if settings.runtime == "docker":
        from .compose import ComposeRuntime

        return ComposeRuntime(settings)
    raise ValueError(f"未知のMAPPING_RUNTIMEです: {settings.runtime}")
