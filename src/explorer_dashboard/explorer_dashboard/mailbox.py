"""How the two threads of the dashboard hand things to each other.

The dashboard has two threads, and each one owns something the other must not touch:

* The RENDER thread (the main thread) owns the window. OpenGL is "thread-affine": an
  OpenGL context belongs to the thread that created it, and a call from any other
  thread is undefined behaviour (a crash if you are lucky, a corrupted picture if you
  are not). threepp draws with OpenGL, so only this thread may call threepp.
* The ROS thread runs the executor, which calls our subscription, service and action
  callbacks whenever a message arrives.

So a callback must never draw. It leaves what it received in a Mailbox, and the render
thread picks it up at the start of its next frame. The C++ simulator of this demo is
built the same way.

Three small tools cover every case:

* Mailbox:    ROS thread -> render thread, for STATE ("the newest map"). Newest wins.
* queue.SimpleQueue (standard library): for EVENTS ("this message must be shown"),
  where every single one counts. See DashboardNode.notices.
* RenderJobs: ROS thread -> render thread, for WORK that needs an answer ("save a
  screenshot and tell me when it is on disk").
"""

import queue
import threading
from concurrent.futures import Future
from typing import Callable, Generic, Optional, Tuple, TypeVar

T = TypeVar('T')


class Mailbox(Generic[T]):
    """A slot for one value, written by one thread and read by another. Newest wins.

    A mailbox is not a queue. If the map is published twice before the render thread
    looks, the older map is simply overwritten: drawing an outdated map first would
    only waste time. That is the right behaviour for state. It would be wrong for
    events, where dropping one loses information.

    Why a lock, when CPython has the GIL? The GIL makes a single bytecode atomic, not
    a method. post() changes two fields, and take() reads and changes two fields. A
    thread switch can happen between any two of those steps, for example after take()
    has seen "fresh" but before it has cleared the flag, and then the same value is
    delivered twice or a new one is marked as already seen. The lock turns each method
    into one indivisible step, so we do not have to reason about interleavings at all.
    (The GIL is also an implementation detail: Python 3.13 can be built without it.)
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: Optional[T] = None
        self._fresh = False

    def post(self, value: T) -> None:
        """Store a new value, replacing any value that was not picked up yet."""
        with self._lock:
            self._value = value
            self._fresh = True

    def take(self) -> Optional[T]:
        """Return the value if it is new since the last take(), otherwise None.

        The critical section is two assignments long. That is the whole trick of the
        pattern: the lock is never held while anybody does real work (parsing a
        message, drawing a frame), so neither thread can make the other one wait.
        """
        with self._lock:
            if not self._fresh:
                return None
            self._fresh = False
            return self._value


class RenderJobs:
    """Work that another thread asks the render thread to do, with a result.

    The screenshot service is the example: its callback runs on the ROS thread, but
    reading the frame back from the graphics card is an OpenGL call. So the callback
    wraps the work in a job, gets a Future ("a result that will exist later"), and
    blocks on future.result(timeout=...). The render thread runs the job at the end of
    its frame and stores the outcome in the Future, which wakes the caller up.

    In C++ this is std::packaged_task and std::future.
    """

    def __init__(self) -> None:
        # queue.SimpleQueue is thread-safe by itself: it has its lock built in.
        self._jobs: 'queue.SimpleQueue[Tuple[Callable[[], object], Future]]' = (
            queue.SimpleQueue())

    def submit(self, job: Callable[[], object]) -> Future:
        """Called from any thread. Returns at once with a Future for the result."""
        future: Future = Future()
        self._jobs.put((job, future))
        return future

    def run_pending(self) -> None:
        """Called from the render thread, once per frame: run every waiting job."""
        while True:
            try:
                job, future = self._jobs.get_nowait()
            except queue.Empty:
                return
            # A caller that gave up waiting has cancelled its Future. Then this
            # returns False, and we skip work that nobody is waiting for.
            if not future.set_running_or_notify_cancel():
                continue
            try:
                future.set_result(job())
            except Exception as error:
                # The exception travels through the Future and is raised again in the
                # thread that called future.result(). The render loop keeps running.
                future.set_exception(error)
