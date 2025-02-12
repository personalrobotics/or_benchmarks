#!/usr/bin/env python

"""
Query format:

world: serialized via prpy.serialization
robot: string 
query:
  - planner_args
  - planner_kwargs
  - planning method
"""

"""
Planner format:

planner_name: string
parameters: 
   - dict of parameter name/value paris
"""

"""
Result format:

planner_file
query_file
time
success
result - raw result from the planner
"""

import logging
logger = logging.getLogger('prpy_benchmarks')

from rospkg import RosPack
ros_pack = RosPack()
package_path = ros_pack.get_path('prpy_benchmarks')

def get_planner(planner_name):
    """
    Convert a planner name to a planner
    """
    if planner_name == 'cbirrt':
        from prpy.planning import CBiRRTPlanner
        planner = CBiRRTPlanner()
    elif planner_name == 'chomp':
        from prpy.planning import CHOMPPlanner
        planner = CHOMPPlanner()
    elif planner_name == 'snap':
        from prpy.planning import SnapPlanner
        planner = SnapPlanner()
    elif planner_name == 'trajopt':
        from or_trajopt import TrajoptPlanner
        planner = TrajoptPlanner()
    else:
        raise Exception('Unrecognized planner name')

    return planner

def generate_configurations(num_samples = 1):
    logger.info('Generating %d valid configuration pairs...', num_samples)
    import random
    benchmark_configurations = []
    valid_samples = 0
    lower, upper = robot.GetDOFLimits()
    while valid_samples < num_samples:
        # Select a random arm
        with env:
            manipulators = robot.GetManipulators()
            m = random.sample(manipulators, 1)[0]
            robot.SetActiveManipulator(m)
            robot.SetActiveDOFs(m.GetArmIndices())
            l, u = robot.GetActiveDOFLimits()

        # sample random start - full configuration
        start_config = [float(random.uniform(lower[i], upper[i])) for i in range(len(lower))]
        with env:
            robot.SetDOFValues(start_config)
        if env.CheckCollision(robot) or robot.CheckSelfCollision():
            continue

        # sample a random goal for just the active dofs
        end_config = [float(random.uniform(l[i], u[i])) for i in range(len(l))]
        with env:
            robot.SetActiveDOFValues(end_config)
        if env.CheckCollision(robot) or robot.CheckSelfCollision():
            continue

        v = { 'manipulator': m.GetName(),
              'start': start_config,
              'end': end_config }
        benchmark_configurations.append(v)
        valid_samples += 1

    return benchmark_configurations

if __name__ == '__main__':

    import herbpy, numpy, os
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker') 

    from prpy.logger import initialize_logging
    initialize_logging()

    # Add the table to the world
    table_in_robot = numpy.array([[0., 0., 1., 0.945],
                                  [1., 0., 0., 0.   ],
                                  [0., 1., 0., 0.02 ],
                                  [0., 0., 0., 1.   ]])
    table_in_world = numpy.dot(robot.GetTransform(), table_in_robot)
    table_path = os.path.join('furniture', 'table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_path)
    table.SetTransform(table_in_world)
    env.Add(table)

    valid_configurations = generate_configurations()
    idx = 0
    query = {}

    from prpy.serialization import serialize_environment
    import yaml
    for v in valid_configurations:
        with env:
            robot.SetDOFValues(v['start'])
            m = robot.GetManipulator(v['manipulator'])
            robot.SetActiveManipulator(m)
            robot.SetActiveDOFs(m.GetArmIndices())
        planner_args = [ v['end'] ]
        planner_kwargs = dict()

        query['world'] = serialize_environment(env)
        query['args'] = planner_args
        query['kwargs'] = planner_kwargs
        query['planning_method'] = 'PlanToConfiguration'
        
        query_file = os.path.join(package_path, 'queries', 'table_query_%d.yaml' % idx)
        with open(query_file, 'w') as f:            
            f.write(yaml.dump(query))
        logger.info('Generated query file %s', query_file)
        
