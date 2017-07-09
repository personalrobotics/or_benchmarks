#!/usr/bin/env python
pkg = 'prpy_benchmarks'
import logging
logger = logging.getLogger(pkg)

import numpy
import openravepy
from prpy.planning.snap import SnapPlanner
from prpy.planning.cbirrt import CBiRRTPlanner
from prpy.planning.logged import LoggedPlanner

## Setup copied from prpy/tests/planning/planning_helpers.py

active_dof_indices = range(7)

# Feasible start/goal pair.
config_feasible_start = numpy.array([
  +2.35061574,  0.61043555,  0.85000000,  1.80684444, -0.08639935,
  -0.69750474,  1.31656172
])

config_feasible_goal = numpy.array([
  -0.84085883,  1.44573701,  0.20000000,  1.72620231, -0.81124757,
  -1.39363597,  1.29233111
])

# Initialize openrave environment
openravepy.RaveInitialize(True)
openravepy.misc.InitOpenRAVELogging()
openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)
env = openravepy.Environment()

with env:
  # Load environment and setup robot  
  env.Load('data/wamtest2.env.xml')
  robot = env.GetRobot('BarrettWAM')
  manipulator = robot.GetManipulator('arm')
  env.Remove(env.GetKinBody('floor'))

  robot.SetActiveManipulator(manipulator)
  robot.SetActiveDOFs(active_dof_indices)

# Create logged planner around CBiRRT planner
planner = LoggedPlanner(CBiRRTPlanner())

# Run planning query
# This will automatically be logged in a timestamp-named file
robot.SetActiveDOFValues(config_feasible_start)
planner.PlanToConfiguration(robot, config_feasible_goal)
