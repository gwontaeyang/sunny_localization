"""Internal LIO wiring shared by launch and the effective-parameter tests."""

import argparse
import json
from pathlib import Path


def get_lio_pipeline_parameters():
    """Return fresh ROS parameter mappings; user files may override any key."""
    lio_outputs = {
        "odom_topic": "/localization_3d/lio/odom",
        "scan_odom_topic": "/localization_3d/lio/scan_odom",
        "deskewed_cloud_topic": "/localization_3d/lio/deskewed_points",
    }
    return {
        "slam_3d_lio_odometry": {"ros__parameters": lio_outputs},
        "slam_3d_localizer": {
            "ros__parameters": {
                "cloud_topic": lio_outputs["deskewed_cloud_topic"],
                "odom_topic": lio_outputs["odom_topic"],
                "registration_odom_topic": lio_outputs["scan_odom_topic"],
                "require_synced_odom_for_update": True,
                "require_exact_odom_for_update": True,
                "odom_max_interpolation_gap_s": 0.10,
                "input_scan_preprocessed": True,
                "cloud_subscription_reliable": True,
                "cloud_subscription_depth": 10,
            },
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    # JSON is valid YAML and preserves ROS scalar types without an extra parser
    # dependency. This generated file is used only by the C++ parameter tests.
    args.output.write_text(json.dumps(get_lio_pipeline_parameters(), indent=2) + "\n")


if __name__ == "__main__":
    main()
