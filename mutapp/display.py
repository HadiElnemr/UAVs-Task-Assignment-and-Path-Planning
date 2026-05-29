"""Configure matplotlib for headless PNG export or interactive display."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

_INTERACTIVE_BACKENDS = ("TkAgg", "Qt5Agg", "GTK3Agg", "WXAgg")


_NON_INTERACTIVE_BACKENDS = frozenset(
    {"agg", "svg", "pdf", "ps", "cairo", "template"}
)


def configure_display(*, interactive: bool) -> str:
    """
    Select matplotlib backend before pyplot is imported elsewhere.

    Call this at the start of ``main()`` (before importing ``mutapp.pipelines``).
    """
    import matplotlib

    if interactive:
        os.environ.pop("MPLBACKEND", None)
        for name in _INTERACTIVE_BACKENDS:
            try:
                matplotlib.use(name, force=True)
                backend = matplotlib.get_backend()
                if backend.lower() not in _NON_INTERACTIVE_BACKENDS:
                    print(f"matplotlib backend: {backend}", file=sys.stderr)
                    return backend
            except Exception:
                continue
        print(
            "Warning: no interactive matplotlib backend available. "
            "Install Tk: sudo apt install python3-tk",
            file=sys.stderr,
        )
        return matplotlib.get_backend()

    if os.environ.get("MPLBACKEND"):
        return matplotlib.get_backend()

    matplotlib.use("Agg", force=True)
    return "Agg"


def backend_is_interactive() -> bool:
    import matplotlib

    return matplotlib.get_backend().lower() not in _NON_INTERACTIVE_BACKENDS


def open_paths(paths: list[Path]) -> None:
    """Open saved PNGs with the desktop default viewer when possible."""
    opener = shutil.which("xdg-open") or shutil.which("gio")
    if opener is None:
        return
    for path in paths:
        if path.suffix.lower() in {".png", ".gif", ".jpg", ".jpeg"} and path.is_file():
            subprocess.Popen([opener, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
