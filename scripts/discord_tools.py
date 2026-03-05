#!/usr/bin/env python3
"""
CLI tool for Discord integration: send images to a Discord channel.

Usage:
  python3 scripts/discord_tools.py send-image --path <image_path> [--delete]

Environment/config required:
  DISCORD_BOT_TOKEN: Discord bot token
  DISCORD_CHANNEL_ID: Channel ID to send images to
"""
import argparse
import os
import sys
import requests

def send_image(path, delete_after):
    token = os.getenv("DISCORD_BOT_TOKEN")
    channel_id = os.getenv("DISCORD_CHANNEL_ID")
    if not token or not channel_id:
        print("Error: DISCORD_BOT_TOKEN and DISCORD_CHANNEL_ID must be set in environment.", file=sys.stderr)
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
    send_parser.add_argument("--delete", action="store_true", help="Delete image after sending")

    args = parser.parse_args()
    if args.command == "send-image":
        send_image(args.path, args.delete)
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()
