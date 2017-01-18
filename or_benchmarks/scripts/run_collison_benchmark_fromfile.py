#!/usr/bin/env python
import argparse
import herbpy
import openravepy
import os
import yaml
import prpy.serialization
import time


def parse_checklog_file(line):

    # Return method_name, obj_name (optional), dof_vals

    log_info = line.split(' ')
    method_name = log_info[0]
    obj_name = None
    dof_vals = None
    n_tokens = len(log_info)

    if n_tokens >= 3:
        if method_name == 'CheckSelfCollision':
            # No object then
            dof_val_strs = log_info[1:n_tokens-1]

        else:
            #method_name is CheckCollision
            token_after_method = log_info[1]
            if token_after_method[0].isalpha():
                #Some object
                obj_name = log_info[1]
                dof_val_strs = log_info[2:n_tokens-1]
            else:
                dof_val_strs = log_info[1:n_tokens-1]

        dof_vals = [float(v) for v in dof_val_strs]

    return method_name,obj_name,dof_vals



def run_benchmark(engine, envfile=None, viewer=None):

    if envfile is None:
        raise Exception('No valid environment file specified')

    yamldict = yaml.safe_load(open(envfile))

    # Result - Dictionary with elapsed_ms, checks and data
    # Elapsed_ms is a double, checks an integer and data a list of doubles
    res_dict = dict()
    res_dict['elapsed_ms'] = 0.0
    res_dict['checks'] = 0
    res_dict['data'] = list()

    # Check that there are collision checks noted
    checks_log = yamldict['collision_log']
    if checks_log[0] != '':
        #Some checks done - so load environment and set checker

        env,robot = herbpy.initialize(sim=True,attach_viewer=viewer)
        prpy.serialization.deserialize_environment(yamldict['environment'],env=env,reuse_bodies = [robot])

        cc = openravepy.RaveCreateCollisionChecker(env,engine)
        if(engine == 'sdf'):
            cc.SendCommand('SetSubChecker ode')
        cc.SetCollisionOptions(0)
        if cc is None:
            raise Exception('Invalid collision engine. Failing.')
        env.SetCollisionChecker(cc)

        # now iterate over records in checks_log
        for record in checks_log:

            method_name,obj_name,dof_vals = parse_checklog_file(record)

            if dof_vals is not None:

                robot.SetDOFValues(dof_vals)
                single_check_time = 0.0

                if method_name == 'CheckSelfCollision':
                    start = time.clock()
                    robot.CheckSelfCollision()
                    tdiff = time.clock() - start
                else:
                    if obj_name is not None:
                        obj = env.GetKinBody(obj_name)
                        start = time.clock()
                        env.CheckCollision(robot,obj)
                        tdiff = time.clock() - start
                    else:
                        start = time.clock()
                        env.CheckCollision(robot)
                        tdiff = time.clock() - start

                single_check_time = tdiff

                #Update res_dict
                res_dict['checks'] += 1
                res_dict['elapsed_ms'] += single_check_time
                res_dict['data'].append(single_check_time)

    return res_dict


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Run collision check benchmarks after reading from file")
    parser.add_argument("--outfile", type=str, default=None,
        help="The output file to save results in, if none results are not saved")
    parser.add_argument("--env", type=str, default=None,
        help="The environment + collision-check file to load")
    parser.add_argument("--engine", type=str, default="ode",
        help="The underlying physics engine to use for collision checking")
    parser.add_argument("--viewer", type=str, default=None,
        help="The viewer to attach to the environment")

    args = parser.parse_args()

    res_dict = run_benchmark(args.engine,
                            envfile = args.env,
                            viewer=args.viewer)

    print res_dict