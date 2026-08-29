#!/usr/bin/env python3
"""
Tiny helper process spawned by pick_and_place.py, dedicated to publishing
gz-transport attach/detach requests for the DetachableJoint gripper
mechanism (see pick_and_place.py's module docstring for why this is a
separate, early-spawned process rather than an in-process
gz.transport13 publish() call).

Protocol: one command per line on stdin ("attach" or "detach"), replies
"done" on stdout after publishing. Exits when stdin closes.
"""

import sys

from gz.transport13 import Node
from gz.msgs10.empty_pb2 import Empty

node = Node()
attach_pub = node.advertise("/gripper/attach", Empty)
detach_pub = node.advertise("/gripper/detach", Empty)

for line in sys.stdin:
    cmd = line.strip()
    if cmd == "attach":
        attach_pub.publish(Empty())
    elif cmd == "detach":
        detach_pub.publish(Empty())
    print("done", flush=True)
