#!/usr/bin/env python3

import sys
import tty
import termios


old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setraw(sys.stdin.fileno())

    print("Press keys. Press q to quit.")

    while True:

        key = sys.stdin.read(1)

        print(
            f"\nReceived: {repr(key)}"
        )

        if key == '\x1b':
            key2 = sys.stdin.read(1)
            print(f"Received: {repr(key2)}")

            if key2 == '[':
                key3 = sys.stdin.read(1)
                print(f"Received: {repr(key3)}")

        if key == 'q':
            break

finally:

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        old_settings
    )