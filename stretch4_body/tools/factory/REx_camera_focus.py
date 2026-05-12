#!/usr/bin/env python3
from stretch4_body.core.hello_utils import print_stretch_re_use
from stretch4_body.subsystem.cameras.calibrate_focus import calibrate_focus

print_stretch_re_use()

def main():
   calibrate_focus()

if __name__ == "__main__":
   main()