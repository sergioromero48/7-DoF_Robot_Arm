#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import time
from fabrikSolver import FabrikSolver3D

arm = FabrikSolver3D()

arm.addSegment(100, 0, 0)
arm.addSegment(100, 0, 0)

arm.compute(100, 150, 50)

# arm.plot()

for i, segment in enumerate(arm.segments):
    print(f"Segment {i+1} endpoint coordinates: {segment.point}")