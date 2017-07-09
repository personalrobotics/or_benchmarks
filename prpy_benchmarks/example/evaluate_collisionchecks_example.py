#!/usr/bin/env python
pkg = 'prpy_benchmarks'
import logging
logger = logging.getLogger(pkg)

import numpy
import openravepy
from prpy_benchmarks.benchmark_utils import evaluate_collisioncheck_benchmark

if __name__ == '__main__':
    
  import argparse
  parser = argparse.ArgumentParser('Evaluate a collisioncheck benchmark')

  parser.add_argument("-b", "--benchmark", type=str,
                      help="A benchmark file for collision checking")
  parser.add_argument("-e", "--engine", type=str,
                      help="The collision checking engine")

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

  # Example arguments for env_outfile and self_outfile
  env_outfile = args.engine+'-benchmark-env.yaml'
  self_outfile = args.engine+'-benchmark-self.yaml'

  evaluate_collisioncheck_benchmark(engine=args.engine,
                                    collresultfile=args.benchmark,
                                    env=env,
                                    robot=robot,
                                    env_outfile=env_outfile,
                                    self_outfile=self_outfile)