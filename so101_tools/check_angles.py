# ruff: noqa: D103
"""Read both arms through their calibration files and print joint angles side by side.

    uv run python so101_tools/check_angles.py --follower-port ... --leader-port ... \
        --follower-id follower_1 --leader-id leader_1

Put both arms in the same pose first: any remaining difference is then a real
calibration offset.  Also shows where the follower will jump to when teleop starts.
"""

from __future__ import annotations

import argparse


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--follower-port", required=True)
    ap.add_argument("--leader-port", required=True)
    ap.add_argument("--follower-id", default="follower_1")
    ap.add_argument("--leader-id", default="leader_1")
    a = ap.parse_args()

    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

    leader = SO101Leader(SO101LeaderConfig(port=a.leader_port, id=a.leader_id))
    follower = SO101Follower(SO101FollowerConfig(port=a.follower_port, id=a.follower_id))
    try:
        leader.connect(calibrate=False)
        follower.connect(calibrate=False)
        act = leader.get_action()
        obs = follower.get_observation()
        print(f"{'joint':14s} {'leader':>9s} {'follower':>9s} {'diff':>8s}   (degrees; gripper 0-100)")
        for k, v in act.items():
            j = k.removesuffix(".pos")
            print(f"{j:14s} {v:9.1f} {obs[k]:9.1f} {v - obs[k]:8.1f}")
    finally:
        for d in (leader, follower):
            if d.is_connected:
                d.disconnect()


if __name__ == "__main__":
    main()
