#!/usr/bin/env python3
from stretch4_body.core.hello_utils import print_stretch_re_use
from stretch4_body.subsystem.cameras.luxonis_list_devices import luxonis_list_devices

print_stretch_re_use()

def main():
   luxonis_list_devices()

if __name__ == "__main__":
   main()