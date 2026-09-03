# Third-party ROS source

`build_ros2.sh` calls `fetch_fairino_ros2.sh` when the vendor checkout is
missing. The fetch is pinned to official FAIRINO commit
`5bed0b0263c8f1e95f51aa45079f904d463c5c50` and uses sparse checkout for:

- `fairino_msgs`
- `fairino_hardware_v3_9_4`
- `fairino5_v6_moveit2_config`

The downloaded `frcobot_ros2-master/` directory is ignored by this repository.
The small, reviewable patches in `../patches/` fix installed SDK loading,
declare the missing MoveIt hardware dependency, and make the hardware plugin
honor the configured controller IP/ServoJ period while failing closed on SDK
errors. Scanner-specific MoveIt files live in `fr5_scanner_650`, not in this
downloaded tree.

Official upstream: <https://github.com/FAIR-INNOVATION/frcobot_ros2>
