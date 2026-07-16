#!/usr/bin/env python3
"""
CLI tool for Discord integration: send images to a Discord channel.

Usage:
    python3 scripts/discord_tools.py send-image --path <image_path> --channel-id <channel_id> [--config <config_path>] [--delete]

Config file structure:
    {
        "channels": {
            "discord": {
                "token": "YOUR_DISCORD_BOT_TOKEN"
            }
        }
    }

Arguments:
    --config: Optional path to a Discord config file. When omitted, the tool
                        checks ~/.nanobot/config.json first and then
                        ~/.config/ros2-skill/discord_tools.json.
    --channel-id: Discord channel ID.
"""
import argparse
import json
import os
import sys
import requests

DEFAULT_NANOBOT_CONFIG = os.path.expanduser("~/.nanobot/config.json")
DEFAULT_STANDALONE_CONFIG = os.path.expanduser("~/.config/ros2-skill/discord_tools.json")


def _fail(message):
    print(f"Error: {message}", file=sys.stderr)
    sys.exit(1)


def resolve_config_path(config_path=None):
    """Return the first usable config path, or fail with a clear message."""
    if config_path:
        expanded = os.path.expanduser(config_path)
        if os.path.exists(expanded):
            return expanded
        _fail(f"Config file not found at {expanded}")

    for candidate in (DEFAULT_NANOBOT_CONFIG, DEFAULT_STANDALONE_CONFIG):
        if os.path.exists(candidate):
            return candidate

    _fail(
        "No Discord config file found. Tried ~/.nanobot/config.json and "
        "~/.config/ros2-skill/discord_tools.json. Create one with "
        '{"channels": {"discord": {"token": "YOUR_DISCORD_BOT_TOKEN"}}}'
    )

def load_config(config_path):
    """Load and validate the Discord config from the resolved config path."""
    config_path = resolve_config_path(config_path)
    
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        # Validate config structure: channels.discord.token
        if "channels" not in config:
            print("Error: Config missing 'channels' section", file=sys.stderr)
            sys.exit(1)
        
        if not isinstance(config["channels"], dict):
            print("Error: Config 'channels' section must be a dictionary", file=sys.stderr)
            sys.exit(1)
        
        if "discord" not in config["channels"]:
            print("Error: Config 'channels' section missing 'discord' key", file=sys.stderr)
            sys.exit(1)
        
        if not isinstance(config["channels"]["discord"], dict):
            print("Error: Config 'channels.discord' must be a dictionary", file=sys.stderr)
            sys.exit(1)
        
        if "token" not in config["channels"]["discord"]:
            print("Error: Config 'channels.discord' section missing 'token' key", file=sys.stderr)
            sys.exit(1)
        
        if not config["channels"]["discord"]["token"]:
            print("Error: Config 'channels.discord.token' is empty", file=sys.stderr)
            sys.exit(1)
        
        return config
        
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in config file: {e}", file=sys.stderr)
        sys.exit(1)
    except IOError as e:
        print(f"Error reading config file: {e}", file=sys.stderr)
        sys.exit(1)

def send_image(path, channel_id, config_path, delete_after):
    config = load_config(config_path)
    token = config["channels"]["discord"]["token"]
    
    if not channel_id:
        print("Error: --channel-id argument is required", file=sys.stderr)
        sys.exit(1)
    
    if not os.path.exists(path):
        print(f"Error: File not found: {path}", file=sys.stderr)
        sys.exit(1)
    
    url = f"https://discord.com/api/v10/channels/{channel_id}/messages"
    headers = {"Authorization": f"Bot {token}"}
    
    with open(path, "rb") as f:
        files = {"file": (os.path.basename(path), f)}
        response = requests.post(url, headers=headers, files=files)
    
    if response.status_code == 200 or response.status_code == 201:
        print(f"Image sent to Discord channel {channel_id} successfully.")
        if delete_after:
            os.remove(path)
            print(f"Deleted image: {path}")
    else:
        print(f"Error sending image: {response.status_code} {response.text}", file=sys.stderr)
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="Discord tools for ros2-skill")
    subparsers = parser.add_subparsers(dest="command")

    send_parser = subparsers.add_parser("send-image", help="Send image to Discord channel")
    send_parser.add_argument("--path", required=True, help="Path to image file")
    send_parser.add_argument("--channel-id", required=True, help="Discord channel ID (provided by agent)")
    send_parser.add_argument(
        "--config",
        default=None,
        help=(
            "Path to a Discord config file. If omitted, checks "
            "~/.nanobot/config.json then ~/.config/ros2-skill/discord_tools.json"
        ),
    )
    send_parser.add_argument("--delete", action="store_true", help="Delete image after sending")

    args = parser.parse_args()
    if args.command == "send-image":
        send_image(args.path, args.channel_id, args.config, args.delete)
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()
