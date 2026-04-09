import os
import sys
from contextlib import contextmanager


@contextmanager
def suppress_stderr():
    """
    Helper method to suppress stderr temporarily.
    """

    with open(os.devnull, "w") as devnull:
        old_stderr = sys.stderr
        sys.stderr = devnull
        try:
            yield
        finally:
            sys.stderr = old_stderr
