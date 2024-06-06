#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import time
from fabrikSolver import FabrikSolver3D

arm = FabrikSolver3D()

arm.addSegment(0, 0, 0.2)
arm.addSegment(0, 0, 0.3)
arm.addSegment(0, 0, 0.3)
arm.addSegment(0, 0, 0.1)

arm.compute(0.2, 0.1, 0.1)

arm.plot()

for i, segment in enumerate(arm.segments):
    print(f"Segment {i+1} endpoint coordinates: {segment.point}")