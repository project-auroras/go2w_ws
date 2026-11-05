# easynav_support_py

ROS 2 package (ament_python) that ships the Python module `easynav_goalmanager_py` with a drop-in
`GoalManagerClient` compatible with the C++ GoalManager, plus exhaustive tests.

## Python import

```python
from easynav_goalmanager_py import GoalManagerClient, ClientState
```

## Build

```bash
colcon build --packages-select easynav_support_py
. install/setup.bash
```

## Tests

- **Unit (exhaustive):** `test/test_goalmanager_client_unit.py` injects synthetic `NavigationControl` messages
  to cover all state transitions (`ACCEPT`, `REJECT`, `FINISHED`, `FAILED`, `CANCELLED`, `ERROR`, `FEEDBACK`).

Run:
```bash
colcon test --packages-select easynav_support_py --event-handlers console_direct+
colcon test-result --all
```
