import argparse

from src.ros2.r3_json_bridge import JsonToRos2Bridge


def main():
    parser = argparse.ArgumentParser(description="Compatibility JSON->ROS2 bridge for Ark Ranger R3.")
    parser.add_argument("--prompt", default="r3-json> ")
    parser.add_argument("--no-help", action="store_true", help="Disable help banner")
    args = parser.parse_args()

    bridge = JsonToRos2Bridge(prompt=args.prompt, show_help=not args.no_help)
    try:
        bridge.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()

