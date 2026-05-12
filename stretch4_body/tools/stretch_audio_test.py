#!/usr/bin/env python3
import os
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

import argparse
parser=argparse.ArgumentParser(description='Test the audio system')
args=parser.parse_args()

os.system("/usr/bin/canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/desktop-login.ogg")