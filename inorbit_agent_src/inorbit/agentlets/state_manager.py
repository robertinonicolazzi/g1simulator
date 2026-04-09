# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
# Module States Manager
#
import os
import shelve
import threading
import time

import inorbit.logger  # Import before other modules to set logging format
from inorbit import INORBIT_MODULE_STATES_CACHE_FILE
from util.once_logger import OnceLogger


# TODO(diegobatt): Make this threshold configurable
MAX_CACHE_AGE = 60 * 60 * 24 * 2  # 2 days


class ModuleStateManager(object):
    """
    Manager for module states.

    This class provides tools for persisting module states into disk, loading and
    setting their state

    TODO(diegobatt): This module should overtake most of the concerns in the Agent's
    main entrypoint. Such as defining default modules and ros dependant ones. Also,
    Subscribing to modules/set_state and modules/get_state_options topics.


    TODO(diegobatt): This module uses shelve as the cache backend, that is proving to be not as
    reliable as we would want. Consider replacing it with another disk-persistent data structure.
    """

    def __init__(self, modules):
        self._modules = modules
        self.logger = inorbit.logger.getLog(self.__class__.__name__)
        self.once_logger = OnceLogger(self.logger)
        # Lock for accessing the modules cache as it is not natively thread-safe
        # for write operations
        self._cache_mutex = threading.Lock()
        # Dict like object persisted into disk
        try:
            self._cache = shelve.open(INORBIT_MODULE_STATES_CACHE_FILE)
        except Exception:
            # If for some reason the cache is corrupted and can't be opened, force
            # the creation of a new one
            self._force_cache_refresh()

    def _force_cache_refresh(self):
        """
        Forces the creation of a new cache in case it is corrupted for some reason.
        """

        self.logger.info("Clearing corrupted module states cache")
        try:
            if os.path.exists(INORBIT_MODULE_STATES_CACHE_FILE):
                os.remove(INORBIT_MODULE_STATES_CACHE_FILE)
            self._cache = shelve.open(INORBIT_MODULE_STATES_CACHE_FILE, "n")
        except Exception as e:
            self.logger.error("Failed to clear module states cache: %s", str(e))

    def save_module_state(self, module_name):
        """
        Persists a module's state into a disk cache.
        """

        # Python 2/3 compatibility: if in Python 2 type(module_name) is <type 'unicode'>
        # The cache won't work, so we convert it to string
        module_name = str(module_name)
        module = self._modules.get(module_name)
        if not module:
            return
        state = module.get_state()
        with self._cache_mutex:
            try:
                self._cache[module_name] = (time.time(), state)
                # Flush content to disk
                self._cache.sync()
            except Exception as e:
                self.logger.error("Failed to save module state for %s: %s", module_name, str(e))
                self.logger.error(e, exc_info=True)
                self._force_cache_refresh()

    def load_module_state(self, module_name):
        """
        Loads a module's state from a disk cache and apply it
        NOTE(diegobatt): This method is awfully similar to inorbit.py's load_module,
        when this manager overtakes all the module-related responsibilities from inorbit.py
        this method should have the exact same functionality as that one. For instance,
        this is not currently checking that ros is enabled for the ros dependant modules
        """

        # Python 2/3 compatibility: if in Python 2 type(module_name) is <type 'unicode'>
        # The cache won't work, so we convert it to string
        module_name = str(module_name)
        with self._cache_mutex:
            try:
                ts, state = self._cache.get(module_name, (None, None))
                # If module is not in the cache, return
                if state is None:
                    return
                if time.time() - ts > MAX_CACHE_AGE:
                    self.logger.info(
                        "Avoiding to load module %s from cache as it is too old", module_name
                    )
                    # If key is too old, remove it
                    del self._cache[module_name]
                    return
            except Exception as e:
                self.logger.error("Failed to load module %s from cache: %s", module_name, str(e))
                return
        if state["loaded"]:
            current_state = self._modules[module_name].get_state()
            # Avoid loading a module that is already loaded as module loading is not
            # always idempotent.
            if not current_state["loaded"]:
                self._modules[module_name].load(state["runlevel"])
                self.logger.info(
                    "Module %s loaded from cache with runlevel %s", module_name, state["runlevel"]
                )
            elif current_state["runlevel"] != state["runlevel"]:
                self._modules[module_name].set_runlevel(state["runlevel"])
                self.logger.info(
                    "Setting Module %s loaded from cache to runlevel %s",
                    module_name,
                    state["runlevel"],
                )
        # NOTE(diegobatt): Not all modules have set_state implemented
        try:
            self.logger.info("Module %s setting state from cache: %s", module_name, state)
            self._modules[module_name].set_state(state)
        except NotImplementedError:
            pass

    def load_module_states(self):
        """
        Loads every available module in the cache.
        """

        with self._cache_mutex:
            try:
                keys = list(self._cache.keys())
            except Exception:
                self.logger.exception("Failed to read cache keys from disk")
                return
        for module_name in keys:
            self.load_module_state(module_name)

    def clear_module_states(self):
        """
        Clears all modules from the cache.
        """

        self.logger.info("Clearing module states cache")
        with self._cache_mutex:
            try:
                self._cache.clear()
            except Exception:
                # If gracefully clearing fails, force it by opening a new empty db
                self._force_cache_refresh()

    def stop(self):
        """
        Gracefully stops manager.
        """

        self._cache.close()
