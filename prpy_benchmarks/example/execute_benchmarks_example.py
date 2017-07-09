#!/usr/bin/env python
pkg = 'prpy_benchmarks'
import logging
logger = logging.getLogger(pkg)

import numpy
import openravepy
from prpy_benchmarks.benchmark_utils import execute_benchmark


if __name__ == '__main__':
    
  import argparse
  parser = argparse.ArgumentParser('Execute a single planning and collisioncheck benchmark')
  parser.add_argument("-l", "--logfile", type=str,
                      help="A yaml file logged from prpy.logger")
  parser.add_argument("-p", "--plannerfile", type=str,
                      help="The file with planner metadata")
  parser.add_argument("-o", "--outfile_plan", type=str,
                      help="The output file for the planner benchmark data")
  parser.add_argument("-c", "--outfile_checks", type=str,
                      help="The output file for the collision check benchmark data")
  
  args = parser.parse_args()

  ## Setup copied from prpy/tests/planning/planning_helpers.py
  active_dof_indices = range(7)
  openravepy.RaveInitialize(True)
  openravepy.misc.InitOpenRAVELogging()
  openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)

  env = openravepy.Environment()

  with env:
    env.Load('data/wamtest2.env.xml')
    robot = env.GetRobot('BarrettWAM')
    manipulator = robot.GetManipulator('arm')
    env.Remove(env.GetKinBody('floor'))

    robot.SetActiveManipulator(manipulator)
    robot.SetActiveDOFs(active_dof_indices)

# Execute planning benchmark
execute_benchmark(queryfile = args.logfile,
                  plannerfile = args.plannerfile,
                  log_collision_checks = False,
                  env = env,
                  robot = robot,
                  outfile = args.outfile_plan)

# Execute collision check benchmark
execute_benchmark(queryfile = args.logfile,
                  plannerfile = args.plannerfile,
                  log_collision_checks = True,
                  env = env,
                  robot = robot,
                  outfile = args.outfile_checks)