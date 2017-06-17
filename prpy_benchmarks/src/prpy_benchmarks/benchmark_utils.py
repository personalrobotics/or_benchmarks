import logging
logger = logging.getLogger('prpy_benchmarks')

class BenchmarkException(Exception):
    pass

def execute_benchmark(queryfile, plannerfile, log_collision_checks=False, env=None, robot=None, outfile=None):
    """
    @param queryfile The file containing the query to execute
    @param plannerfile The file containing metadata about the planner to use
    @param log_collision_checks Whether or not to log collision checks
    @param env The env to run the query against (if None a new env 
      is created)
    @param robot The robot to use (if None, the robot is loaded as 
      part of deserialization)
    @param outdir If not none, the file where the result should be written
    """
    import yaml
    import openravepy
    from prpy.planning import NamedPlanner
    import prpy.serialization


    # TODO Currently for a single planner only
    # TODO Responsibility for doing for multiple planners with higher level user

    # Load the query
    from .query import BenchmarkQuery
    query = BenchmarkQuery()
    
    query.from_yaml(queryfile, env=env, robot=robot)


    # Load the planner metadata
    from .planner import BenchmarkPlannerMetadata
    planner_metadata = BenchmarkPlannerMetadata()
    with open(plannerfile, 'r') as f:
        planner_metadata.from_yaml(yaml.load(f.read()))

    # Figure out which planner to use
    from .planner import get_planner
    planner = get_planner(planner_metadata.planner_module,
                          planner_metadata.planner_class_name,
                          **planner_metadata.planner_parameters)

    # Create a named planner 
    named_planner = NamedPlanner(delegate_planner=planner)


    # Get the planning method from the planner
    try:
        planning_method = getattr(named_planner, query.planning_method)
    except AttributeError:
        raise RuntimeError('That planner does not support planning method {}!'.format(query.planning_method))

    # Get robot from query args
    method_args = []
    for method_arg in query.args:
        method_args.append(prpy.serialization.deserialize(env,method_arg))
    method_kwargs = {}
    for key,value in query.kwargs.items():
        method_kwargs[key] = prpy.serialization.deserialize(env,value)

    #Set to stubchecker if collisions to be logged
    if log_collision_checks:
        stubchecker = openravepy.RaveCreateCollisionChecker(env,'stub_checker')
        env.SetCollisionChecker(stubchecker)

    # Execute with timing
    from prpy.planning import PlanningError
    from prpy.util import Timer
    with Timer() as timer:
        try:
            success = True
            path = planning_method(*method_args, **method_kwargs)
        except PlanningError as e:
            success = False
            path = None
    plan_time = timer.get_duration()

    if not log_collision_checks:
        # Create a result object
        from .result import BenchmarkResult
        from os.path import basename

        result = BenchmarkResult(queryfile=basename(queryfile), 
                                 query_md5=to_md5(queryfile),
                                 plannerfile=basename(plannerfile),
                                 planner_data_md5=to_md5(plannerfile),
                                 time=plan_time, 
                                 success=success, 
                                 path=path)

        # Save to yaml
        if outfile is not None:
            with open(outfile, 'w') as f:
                f.write(yaml.dump(result.to_yaml()))
            logger.info('Results written to file %s', outfile)

    else:
        from .result import BenchmarkCollisionResult
        import json

        # Request collision check logs from stubchecker
        check_info = stubchecker.SendCommand('GetLogInfo')
        check_info_dict = json.loads(check_info)
        stubchecker.SendCommand('Reset')


        # Save only the relevant DOF values used for self+env checks
        relevant_dof_list = get_relevant_DOF_list(check_info_dict)

        result = BenchmarkCollisionResult(environment=env,
                                          collision_log=relevant_dof_list)

        # Save to json
        if outfile is not None:
            with open(outfile,'w') as f:
                json.dump(result.to_dict(),f)
            logger.info('Results written to file %s',outfile)
    
    return result


def evaluate_collisioncheck_benchmark(engine,env=None,robot=None,collresultfile=None,
                                     option_dofs='active',env_first = True, 
                                     env_outfile=None, self_outfile=None): 
    """
    Runs the specified collision checking engine on the log from execute_benchmark.
    Expects a special format that has only the list of relevant DOF sets checked for
    self and env collisions, for a given environment. It only accepts the format
    output by the execute. The default order is env check first, 
    then self check, for both logging and evaluating
    @param engine The collision checker engine to use (ODE/PQP/FCL)
    @param env The OpenRAVe environment to use
    @param robot The OpenRAVe robot to use
    @param collresultfile The name of the file that has the environment and logged checks 
    @param option_dofs Whether to check with Active DOFs or all DOFs
    @param env_outfile The YAML file to save results to for environment-only checks 
    @param self_outfile The YAML file to save results to for self-only checks
    """
    import json
    import yaml
    import openravepy
    import atexit
    from .result import BenchmarkCollisionResult

    if collresultfile is None:
        raise Exception('No valid collision log file specified')

    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)

    with open(collresultfile,'r') as fin:
        collresultdict = json.load(fin)
        collresult = BenchmarkCollisionResult()
        collresult.from_dict(collresultdict,env,robot)


    # Lists for noting elapsed time and check result
    time_list = list()
    result_list = list()

    if not robot:
        robot = env.GetRobots()[0]

    # Lock openrave environment
    with env:

        # Create module to evaluate check time
        checkermodule = openravepy.RaveCreateModule(env,'CheckerResultModule')

        # Set collision checker to evaluate
        cc = openravepy.RaveCreateCollisionChecker(env,engine)
        if option_dofs == 'active':
            cc.SetCollisionOptions(openravepy.CollisionOptions.ActiveDOFs)
        else:
            cc.SetCollionOptions(0)

        if cc is None:
            raise Exception('Invalid collision engine. Failing.')
        env.SetCollisionChecker(cc)

        for dof_vals in collresult.collision_log:

            robot.SetDOFValues(dof_vals)
            single_check_time = 0.0

            # Specify env and self check methods
            standalone_checklog_dict = {'methodname': 'CheckStandaloneSelfCollision',
                                'body' : robot.GetName()}
            body_env_checklog_dict = {'methodname' : 'CheckCollision_body',
                                'body' : robot.GetName()}

            check_order = [body_env_checklog_dict,standalone_checklog_dict]

            for spec_dict in check_order:

                check_specs_str = json.dumps(spec_dict)
                
                string_command = 'EvaluateCheck '+check_specs_str
                check_and_time_str = checkermodule.SendCommand(string_command)
                check_and_time = check_and_time_str.split(' ')


                single_check_result = False
                if int(check_and_time[0]) == 1:
                    single_check_result = True
                single_check_time = float(check_and_time[1])

                # Update performance_dict
                time_list.append(single_check_time)
                result_list.append(single_check_result)

    #Reject the first pair as they are misleading
    time_list = time_list[2:]
    result_list = result_list[2:]

    # Save depending on env first or self first
    if env_first:
        env_times, self_times = split_by_env_first(time_list,result_list)
    else:
        self_times, env_times = split_by_self_first(time_list,result_list)

    # Separate dictionaries for env and self first
    env_performance_dict = {'checks':len(env_times),'elapsed_sec':sum(env_times),'data':{'times':env_times}}
    self_performance_dict = {'checks':len(self_times),'elapsed_sec':sum(self_times),'data':{'times':self_times}}

    # Save to file
    if env_outfile is not None and self_outfile is not None:
        with open(env_outfile,'w') as fout:
            yaml.dump(env_performance_dict,fout)
        with open(self_outfile,'w') as fout:
            yaml.dump(self_performance_dict,fout)

        logger.info('Collision Benchmarking Results written to files')


def get_relevant_DOF_list(check_info_dict):
    """
    From the output of stub_checker, get the unique sets of
    DOFs that are queried for self and env checks
    @ param check_info_dict The dictionary of collision check information
        obtained from a single run of or_stub_checker
    """
    n_keys = len(check_info_dict.keys())
    prev_method_name = ''
    relevant_DOF_list = list()

    for i in range(1,n_keys+1):

        check_key = str(i)
        if check_key in check_info_dict.keys():
            
            record = check_info_dict[check_key]
            method_name = record['methodname']
            
            if method_name == 'CheckStandaloneSelfCollision':
                if prev_method_name == 'CheckCollision_body':

                    # Relevant DOF set; record in list
                    relevant_DOF_list.append(check_info_dict[check_key]['dofvals'])
        
            prev_method_name = method_name

    return relevant_DOF_list


def split_by_env_first(time_list,result_list):
    """
    Split the elapsed time for checks and the result of the check,
    assuming the environment check is first
    @param time_list The list of elapsed times
    @param result_list The corresponding list of check results
    """

    self_times = list()
    env_times = list()

    n_vals = len(result_list)

    for i in range(n_vals/2):

        # Env time is every (2*i)th and self is (2*i+1)th
        env_times.append(time_list[2*i])
        if result_list[2*i] == False:
            # Needed to check self too to verify
            self_times.append(time_list[2*i+1])

    return env_times, self_times


def split_by_self_first(time_list,result_list):
    """
    Split the elapsed time for checks and the result of the check,
    assuming the self-collision check is first
    @param time_list The list of elapsed times
    @param result_list The corresponding list of check results
    """

    self_times = list()
    env_times = list()

    n_vals = len(result_list)

    for i in range(n_vals/2):

        # Env time is every (2*i)th and self is (2*i+1)th
        self_times.append(time_list[2*i]+1)
        if result_list[2*i+1] == False:
            # Needed to check env too to verify
            env_times.append(time_list[2*i])

    return self_times, env_times


def to_md5(filename):
    """
    Read in a file and compute md5 checksum
    """
    import hashlib
    hash = hashlib.md5()
    with open(filename, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash.update(chunk)
    return hash.hexdigest()
