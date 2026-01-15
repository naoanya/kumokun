"""Kumokun servo wrapper.

Provides a thin wrapper around `servo_controller.ServoController` and
`servo_controller_dummy.ServoControllerDummy` that stores the last-received
position information internally instead of returning raw response strings.

This module is intentionally conservative: it parses position-bearing
responses and updates an internal snapshot (`get_last_positions`). The
receive path is left synchronous for now; a simple polling thread helper is
provided and can be replaced with a true asynchronous receiver later.
"""

from __future__ import annotations

import copy
import json
import logging
import os
import sys
from typing import Any, Dict, Iterable, List, Optional, Union

# Make local imports reliable for scripts that add py_utils to sys.path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Assume ServoController exists in the environment (per user request)
from servo_controller import ServoController
from servo_controller_dummy import ServoControllerDummy
from servo_converter import ServoConverter
from kumokun_config import SERVO_CONFIG

logger = logging.getLogger(__name__)


class KumokunServo:
    """Wrapper that maintains last received servo positions.

    Usage notes:
    - Public methods like `set_pos` / `set_all_pos` forward to the underlying
      controller but return only a (success, message) tuple rather than raw
      response payloads.
    - The parsed servo feedback is stored in `get_last_positions()`.
    - Use `start_receive_thread()` to enable periodic polling (placeholder).
    """

    def __init__(self, use_dummy: bool = False) -> None:
        # Choose controller implementation explicitly
        if use_dummy:
            print("KumokunServo: Using Dummy Servo Controller")
            self.controller = ServoControllerDummy()
        else:
            print("KumokunServo: Using Real Servo Controller")
            self.controller = ServoController()

        # last known positions (1..18). None = unknown
        self._last_positions: Dict[int, Optional[int]] = {i: None for i in range(1, 19)}
        # free state per physical servo id (1..18). True means torque off/free.
        # Initial state is free as requested.
        self._free_state: Dict[int, bool] = {i: True for i in range(1, 19)}

        # Build shared ServoConverter instances for all logical sids
        try:
            self.converters: Dict[int, ServoConverter] = {}
            for sid, conf in SERVO_CONFIG.items():
                self.converters[sid] = ServoConverter(
                    direction=conf.get("direction"),
                    offset=conf.get("offset"),
                    min_angle=conf.get("min_angle"),
                    max_angle=conf.get("max_angle"),
                )
        except Exception:
            # If config isn't available or build fails, leave converters unset
            self.converters = None

    # Context manager support
    def __enter__(self) -> "KumokunServo":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            self.disconnect()
        except Exception:
            pass

    # --- Connection proxies ---
    def list_ports(self) -> List[str]:
        return self.controller.list_ports()

    def connect(self, port: str, baudrate: int = 115200):
        return self.controller.connect(port, baudrate=baudrate)

    def disconnect(self) -> Any:
        return self.controller.disconnect()

    @property
    def is_connected(self) -> bool:
        """Proxy to underlying controller connection state."""
        return bool(getattr(self.controller, "is_connected", False))

    # --- Position control (forward, but update internal state) ---
    def set_all_pos(self, positions: Iterable[int]) -> (bool, str):
        pos_list = list(positions)
        if len(pos_list) != 18:
            return False, "Must provide exactly 18 positions"
        # Update free state immediately based on values: 0 => free, non-zero => not free
        self._set_free_state_from_positions(pos_list)

        resp, err = self._call_controller("set_all_pos", pos_list)
        if err:
            return False, err
        # also sync last positions from provided values if response didn't include them
        self._sync_last_positions_from_positions(pos_list)
        return True, "OK"

    def get_all_pos(self) -> (bool, str):
        resp, err = self._call_controller("get_all_pos")
        if err:
            return False, err
        return True, "OK"

    def free_all(self) -> (bool, str):
        # Mark all servos as free immediately
        self._set_all_free()

        resp, err = self._call_controller("free_all")
        if err:
            return False, err
        return True, "OK"

    def set_pos(self, servo_id: int, pos: int) -> (bool, str):
        # Update free state immediately: pos == 0 -> free
        self._set_free_state_for_id(servo_id, pos)

        # use the 'light' command variant for individual sets
        resp, err = self._call_controller("set_pos_light", servo_id, pos)
        if err:
            return False, err
        return True, "OK"

    def get_pos(self, servo_id: int) -> (bool, str):
        # call the 'light' getter which returns a single response
        resp, err = self._call_controller("get_pos_light", servo_id)
        if err:
            return False, err
        return True, "OK"

    def free(self, servo_id: int) -> (bool, str):
        # Mark single servo as free
        self._set_free_state_for_id(servo_id, 0)

        resp, err = self._call_controller("free", servo_id)
        if err:
            return False, err
        return True, "OK"

    # --- Accessors ---
    def get_last_positions(self) -> Dict[int, Optional[int]]:
        return copy.deepcopy(self._last_positions)

    def get_last_position(self, servo_id: int) -> Optional[int]:
        return self._last_positions.get(int(servo_id))

    # --- Free state API ---
    def is_free(self, servo_id: int) -> bool:
        """Return True if the given physical servo id (1..18) is considered free."""
        try:
            return bool(self._free_state.get(int(servo_id), True))
        except Exception:
            return True

    def get_free_states(self) -> Dict[int, bool]:
        """Return a copy of the free-state mapping for all servos."""
        return copy.deepcopy(self._free_state)

    # --- Internal controller call helper ---
    def _call_controller(self, method: str, *args, update_positions: bool = True):
        """Call underlying controller method and update internal state.

        Returns (resp, err) where err is None on success.
        """
        fn = getattr(self.controller, method, None)
        if fn is None:
            return None, f"Controller has no method {method}"
        try:
            resp, err = fn(*args)
        except Exception as e:
            return None, str(e)

        if err:
            return None, err
        if resp is None:
            return None, "No response"

        if update_positions:
            try:
                self._update_from_response_list(resp)
            except Exception:
                logger.debug("Failed to update positions from response: %r", resp)

        return resp, None

    # --- Internal state helpers ---
    def _set_free_state_for_id(self, servo_id: int, pos_or_state: int) -> None:
        try:
            self._free_state[int(servo_id)] = (int(pos_or_state) == 0)
        except Exception:
            # keep previous state on parse error
            pass

    def _set_free_state_from_positions(self, positions: Iterable[int]) -> None:
        for idx, v in enumerate(positions, start=1):
            self._set_free_state_for_id(idx, v)

    def _set_all_free(self) -> None:
        for i in range(1, 19):
            self._free_state[i] = True

    def _sync_last_positions_from_positions(self, positions: Iterable[int]) -> None:
        for idx, v in enumerate(positions, start=1):
            try:
                self._last_positions[idx] = int(v)
            except Exception:
                pass

    # --- Internal parsing helpers ---
    def _update_from_response_list(self, resp_list: Union[List[Any], Any]) -> None:
        # Accept either a list of responses or a single bulk response
        if isinstance(resp_list, (list, tuple)):
            for item in resp_list:
                self._update_from_response(item)
        else:
            self._update_from_response(resp_list)

    def _update_from_response(self, resp: Any) -> None:
        if resp is None:
            return

        data: Optional[Dict[str, Any]] = None
        if isinstance(resp, str):
            try:
                data = json.loads(resp)
            except Exception:
                logger.debug("Failed to json-decode response: %r", resp)
                return
        elif isinstance(resp, dict):
            data = resp
        else:
            # unknown format
            return

        sid = data.get("id")
        if sid is None:
            return
        # support either 'feedback' or 'pos'
        pos_val = data.get("feedback", data.get("pos"))
        try:
            sid_i = int(sid)
            if pos_val is not None:
                # tolerate numeric strings
                self._last_positions[sid_i] = int(float(pos_val))
        except Exception:
            logger.debug("Invalid id/pos in response: %r", data)
            return

    # Note: receive thread support removed per user request. Async reception
    # can be implemented later when needed.


__all__ = ["KumokunServo"]
