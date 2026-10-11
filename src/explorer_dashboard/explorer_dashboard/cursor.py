"""Where is the mouse cursor inside our window?

threepp's Python module offers mouse orbiting (OrbitControls) and key polling, but it
does not tell a script where the cursor is. For the one feature that needs it ("G: send
a goal to the floor point under the cursor") we ask the operating system directly.

Only Windows is implemented, because that is what the course runs on. On any other
system cursor_in_window() returns None, and the dashboard falls back to the point the
camera is looking at (see DashboardWindow.floor_point_under_cursor).
"""

import ctypes
import sys
from typing import Optional, Tuple


def cursor_in_window(window_title: str) -> Optional[Tuple[float, float]]:
    """The cursor as fractions (u, v) of the window's drawing area, or None.

    (0, 0) is the top-left corner and (1, 1) the bottom-right one. None means: not
    available on this system, window not found, or the cursor is outside the window.
    Fractions, not pixels, so display scaling (125 %, 150 %) cannot confuse us.
    """
    if sys.platform != 'win32':
        return None

    # ctypes.wintypes only imports on Windows, hence the import inside the function.
    from ctypes import wintypes

    user32 = ctypes.windll.user32
    # Tell ctypes the real parameter and return types. Its default is a 32 bit int,
    # which would cut a 64 bit window handle in half.
    user32.FindWindowW.restype = wintypes.HWND
    user32.FindWindowW.argtypes = (wintypes.LPCWSTR, wintypes.LPCWSTR)
    user32.ScreenToClient.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.POINT))
    user32.GetClientRect.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.RECT))

    window = user32.FindWindowW(None, window_title)
    point = wintypes.POINT()
    area = wintypes.RECT()
    if not window or not user32.GetCursorPos(ctypes.byref(point)):
        return None
    # The cursor position is in screen coordinates. Make it relative to the top-left
    # corner of the window's client area (the part inside the title bar and borders).
    if not user32.ScreenToClient(window, ctypes.byref(point)):
        return None
    if not user32.GetClientRect(window, ctypes.byref(area)) or area.right <= 0:
        return None
    u, v = point.x / area.right, point.y / area.bottom
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return None
    return u, v
