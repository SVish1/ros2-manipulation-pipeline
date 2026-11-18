# ROS 2 Fault-Tolerant Pick-and-Place Pipeline

A ROS 2 (Humble) manipulation pipeline that drives a **Franka Emika Panda** arm through a full pick-and-place task using **MoveIt 2**. It finds objects through **TF2**, monitors every motion with timeouts, and recovers automatically when a step fails.

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![MoveIt 2](https://img.shields.io/badge/MoveIt-2-blue)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-ready-2496ED?logo=docker&logoColor=white)
![pytest](https://img.shields.io/badge/tested_with-pytest-0A9EDC?logo=pytest&logoColor=white)

---

## Highlights

- **End-to-end pick-and-place:** a 9-step sequence (approach, descend, grasp, retreat, traverse, place, release, retreat, return home) runs as a state machine.
- **Perception through TF2:** object and place-target poses come from the TF tree at runtime, not from hard-coded values. A perception system could replace the simulated broadcaster without changing the motion code.
- **Direct MoveIt 2 action interface:** I build `MotionPlanRequest` goals by hand, with position constraints (spherical tolerance regions) and orientation constraints, and send them to the `/move_action` server. Speed is capped at 30% velocity and acceleration scaling.
- **Monitored execution:** every motion and gripper action has a timeout on both goal acceptance and result. If the result times out, the goal is actively cancelled so the arm never runs unsupervised.
- **Automatic recovery:** when a motion fails, the arm returns to a safe home pose and retries. If retries run out, the sequence stops cleanly with a clear log message instead of continuing from an unknown state.
- **Unit-tested without a robot:** a pytest suite mocks the action servers and the TF buffer. It covers success paths, rejected goals, timeouts, goal cancellation, retry logic and aborting mid-sequence.
- **Reproducible environment:** a Dockerfile installs ROS 2 Humble, MoveIt 2, the Panda MoveIt config and ros2_control.

## Architecture

```
┌──────────────────────┐   TF2 (world → table → platform / object)   ┌──────────────────────┐
│  scene_broadcaster   │ ──────────────────────────────────────────▶ │   pick_place_node    │
│  (stands in for      │                                             │   (state machine)    │
│   perception)        │                                             └──────────┬───────────┘
└──────────────────────┘                                                        │
                                       MoveGroup action (/move_action)          │  GripperCommand action
                                ┌───────────────────────────────────────────────┤  (/panda_hand_controller/gripper_cmd)
                                ▼                                               ▼
                       ┌─────────────────┐                             ┌─────────────────┐
                       │  MoveIt 2       │ ──── ros2_control ────────▶ │  Panda arm +    │
                       │  move_group     │                             │  gripper        │
                       └─────────────────┘                             └─────────────────┘
```

### Nodes

| Node | Purpose |
|------|---------|
| [`scene_broadcaster`](src/manipulation_pipeline/manipulation_pipeline/scene_broadcaster.py) | Publishes static `table` and `platform` frames and a live `object` frame over TF2, simulating a perception system. |
| [`pick_place_node`](src/manipulation_pipeline/manipulation_pipeline/pick_place_node.py) | Looks up the object and platform poses, then runs the monitored pick-and-place sequence through MoveIt 2 and the gripper controller. |

### Pick-and-place sequence

1. `approach-pick`: move 10 cm above the object, gripper pointing down
2. `descend-pick`: lower to grasp height
3. **grasp**: close the gripper
4. `retreat-pick`: lift back to approach height
5. `approach-place`: move across the workspace to above the platform
6. `descend-place`: lower onto the platform
7. **release**: open the gripper
8. `retreat-place`: lift clear
9. `return-home`: return to the home pose

Each motion step goes through `_move_with_recovery()`, which allows up to 2 attempts and returns to home between them. Each gripper step goes through `_monitored_gripper()`.

## Engineering challenge: an executor deadlock

In an early version, action results sometimes never arrived and motions timed out even though MoveIt had finished them. The cause was **double-spinning**: `rclpy.spin_until_future_complete()` was called inside a timer callback that an outer `rclpy.spin()` was already driving, so the same node was being spun by two executors at once. That silently starved action responses.

The fix was to remove the outer spin. A short `spin_once` loop lets the TF buffer fill up, and then the sequence runs synchronously, with each action future spun to completion by its own call. The reasoning is documented in `main()` in [`pick_place_node.py`](src/manipulation_pipeline/manipulation_pipeline/pick_place_node.py).

## Testing

[`test_pick_place_node.py`](src/manipulation_pipeline/test/test_pick_place_node.py) contains 13 unit tests in five groups:

| Area | What's verified |
|------|-----------------|
| Pose and constraint helpers | Pose construction, end-effector link and tolerance passed into the MoveIt constraints |
| TF2 lookup | Successful lookup; safe `None` return on `TransformException` |
| Execution monitor (`_send_and_wait`) | Success, rejected goal, timeout on acceptance, timeout on result with **goal cancellation** |
| Retry and recovery | Success on first try, one retry then success, giving up after `MAX_ATTEMPTS` with the right number of home recoveries |
| Sequencing | Abort when TF lookup fails, the full 9-step run (7 motions + 2 gripper actions), abort in the middle of the sequence |

The action clients are patched with `unittest.mock`, so the tests run without MoveIt or a simulator. The package also includes the standard ament `flake8`, `pep257` and copyright lint tests.

## Getting started

### 1. Build the Docker image

```bash
docker build -t ros2-manipulation .
docker run -it --rm -v $(pwd):/workspace ros2-manipulation
```

### 2. Build the workspace (inside the container)

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch MoveIt 2 with the Panda demo

```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

### 4. Run the pipeline (in separate terminals)

```bash
ros2 run manipulation_pipeline scene_broadcaster
ros2 run manipulation_pipeline pick_place_node
```

### 5. Run the tests

```bash
colcon test --packages-select manipulation_pipeline
colcon test-result --verbose
# or directly:
cd src/manipulation_pipeline && python3 -m pytest test/test_pick_place_node.py -v
```

## Tech stack

**ROS 2 Humble** · **MoveIt 2** · **TF2** · **ros2_control** · **rclpy action clients** · **Python 3** · **pytest / unittest.mock** · **Docker** · **colcon / ament_python**

## Project structure

```
ros2-manipulation-pipeline/
├── Dockerfile
└── src/manipulation_pipeline/
    ├── manipulation_pipeline/
    │   ├── pick_place_node.py      # state machine, MoveIt goals, monitoring, recovery
    │   └── scene_broadcaster.py    # TF2 scene / simulated perception
    ├── test/
    │   ├── test_pick_place_node.py # unit tests with mocked action servers
    │   └── conftest.py             # rclpy context fixture
    ├── package.xml
    └── setup.py
```

## Possible extensions

- Replace the simulated broadcaster with real perception (for example, ArUco markers or a point-cloud segmentation node)
- Add the object and platform to the MoveIt planning scene as collision objects
- Use Cartesian paths for the approach and descend moves
- Add a launch file that starts MoveIt, the scene and the pick-and-place node together
