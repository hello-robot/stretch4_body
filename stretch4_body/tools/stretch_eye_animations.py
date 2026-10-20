#!/usr/bin/env python3
from __future__ import print_function

import argparse
import sys
import time

import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.power_periph import PowerPeriphDefn


COLOR_NAMES = {
    "white": (255, 255, 255),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "cyan": (0, 255, 255),
    "magenta": (255, 0, 255),
    "yellow": (255, 255, 0),
    "orange": (255, 80, 0),
    "purple": (128, 0, 255),
    "stretch": (40, 48, 60),
}


def _animation_items():
    return sorted(PowerPeriphDefn.EYE_ANIM_NAME_TO_IDX.items(),
                  key=lambda item: item[1])


def list_animations():
    print("Available eye animations:")
    for name, idx in _animation_items():
        print("  {:>2}: {}".format(idx, name))


def list_colors():
    print("Named colors:")
    for name in sorted(COLOR_NAMES):
        print("  {:<8} {}".format(name, COLOR_NAMES[name]))


def parse_animation(value):
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    if text.isdigit():
        idx = int(text)
    else:
        idx = PowerPeriphDefn.EYE_ANIM_NAME_TO_IDX.get(text.upper())
    if idx is None or idx < 0 or idx >= PowerPeriphDefn.EYE_ANIM_COUNT:
        names = ", ".join(name for name, _ in _animation_items())
        raise argparse.ArgumentTypeError(
            "unknown animation '{}'. Use one of: {}".format(value, names))
    return idx


def parse_channel(value):
    try:
        channel = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("{} is not an integer".format(value))
    if channel < 0 or channel > 255:
        raise argparse.ArgumentTypeError("{} is outside 0-255".format(value))
    return channel


def parse_color(value):
    if value is None:
        return None
    text = value.strip().lower()
    if text in COLOR_NAMES:
        return COLOR_NAMES[text]
    if text.startswith("#"):
        text = text[1:]
    if len(text) == 6:
        try:
            return tuple(int(text[i:i + 2], 16) for i in (0, 2, 4))
        except ValueError:
            pass
    raise argparse.ArgumentTypeError(
        "unknown color '{}'. Use a name, RRGGBB, or #RRGGBB".format(value))


def build_parser():
    parser = argparse.ArgumentParser(
        description="Control the Stretch eye LED animations")
    parser.add_argument("-d", "--direct", action="store_true",
                        help="Use direct serial API instead of the body server")
    parser.add_argument("--usb", default=None,
                        help="Power-periph serial device for --direct mode")
    parser.add_argument("--list", action="store_true",
                        help="List available animations and exit")
    parser.add_argument("--list-colors", action="store_true",
                        help="List named colors and exit")
    parser.add_argument("--animation", type=parse_animation,
                        help="Animation for both eyes, by name or index")
    parser.add_argument("--left", type=parse_animation,
                        help="Left eye animation, by name or index")
    parser.add_argument("--right", type=parse_animation,
                        help="Right eye animation, by name or index")
    parser.add_argument("--color", type=parse_color,
                        help="Named color, RRGGBB, or #RRGGBB")
    parser.add_argument("--rgb", nargs=3, type=parse_channel,
                        metavar=("R", "G", "B"),
                        help="RGB color channels, each 0-255")
    parser.add_argument("--intensity", type=parse_channel, default=255,
                        help="LED intensity 0-255")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds to keep the command active before exit")
    parser.add_argument("--interactive", action="store_true",
                        help="Prompt for animation and color values")
    return parser


def prompt_optional(prompt):
    text = input(prompt).strip()
    return text if text else None


def interactive_args():
    list_animations()
    print("")
    left = parse_animation(prompt_optional(
        "Left eye animation name/index, or Enter to skip: "))
    right = parse_animation(prompt_optional(
        "Right eye animation name/index, or Enter to skip: "))
    color_text = prompt_optional(
        "Color name or hex [stretch]: ") or "stretch"
    color = parse_color(color_text)
    intensity_text = prompt_optional("Intensity 0-255 [255]: ") or "255"
    intensity = parse_channel(intensity_text)
    return left, right, color, intensity


def get_power_periph(args):
    if args.direct:
        from stretch4_body.subsystem.power_periph import PowerPeriph
        kwargs = {}
        if args.usb:
            kwargs["usb"] = args.usb
        return PowerPeriph(**kwargs)
    from stretch4_body.robot.robot_client import PowerPeriphClient
    return PowerPeriphClient()


def main():
    hu.print_stretch_re_use()
    parser = build_parser()
    args = parser.parse_args()

    if args.list:
        list_animations()
        return 0
    if args.list_colors:
        list_colors()
        return 0

    if args.interactive or not any(
            (args.animation, args.left, args.right, args.color, args.rgb)):
        left_idx, right_idx, color, intensity = interactive_args()
    else:
        left_idx = args.left if args.left is not None else args.animation
        right_idx = args.right if args.right is not None else args.animation
        color = tuple(args.rgb) if args.rgb is not None else args.color
        if color is None:
            color = COLOR_NAMES["stretch"]
        intensity = args.intensity

    if left_idx is None and right_idx is None:
        print("No eye animation selected.")
        return 1

    p = get_power_periph(args)
    if not p.startup():
        print("Failed to start PowerPeriph")
        return 1

    try:
        r, g, b = color
        print("Sending left={}, right={}, intensity={}, rgb=({}, {}, {})".
              format(left_idx, right_idx, intensity, r, g, b))
        p.set_eye_animation(left_idx=left_idx, right_idx=right_idx,
                            intensity=intensity, r=r, g=g, b=b)
        if not p.push_command():
            print("Failed to push eye animation command")
            return 1
        if args.duration > 0:
            time.sleep(args.duration)
        return 0
    finally:
        p.stop()


if __name__ == "__main__":
    sys.exit(main())
