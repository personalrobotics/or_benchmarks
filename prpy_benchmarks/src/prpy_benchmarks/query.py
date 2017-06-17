import yaml

class BenchmarkQuery(object):
        
    def __init__(self, planning_method=None, env=None, args=None, kwargs=None):
        """
        @param planning_method The planning method called by this query (string)
        @param env The OpenRAVE environment to run the query against
        @param args A list of arguments to pass to the planning method
        @param kwargs A dictionary of keyword arguments to pass to the planning method
        """
        from prpy.serialization import serialize_environment
        self.planning_method = planning_method
        self.env = env
        self.args = args if args is not None else list()
        self.kwargs = kwargs if kwargs is not None else dict()

    def to_yaml(self):
        """
        Serialize this query to a yaml dictionary
        """
        
        if self.env is None:
            raise BenchmarkException('The query has no serialized environment')
        if self.planning_method is None:
            raise BenchmarkException('The query has no planning method defined')

        from prpy.serialization import serialize, serialize_environment
        query = {}
        if self.env:
            query['env'] = serialize_environment(self.env)

        if self.args:
            query['args'] = serialize(self.args)

        if self.kwargs:
            query['kw_args'] = serialize(self.kwargs)

        if self.planning_method:
            query['method'] = serialize(self.planning_method)

        return query

    def from_yaml(self, query_yaml, env=None, robot=None):
        """
        @param query_yaml Yaml file containing query data
        @param env The environment to deserialize into, if None 
          a new environment is created
        @param robot The robot to use (if None, the robot is 
          loaded as part of deserialization)
        """

        from prpy.serialization import deserialize_environment, deserialize

        with open(query_yaml, 'r') as f:
            query = yaml.load(f)

        serialized_env = query.get('environment', None)
        if serialized_env:
            reuse_bodies = [ robot ] if robot is not None else list()
            self.env = deserialize_environment(serialized_env, env=env, reuse_bodies=reuse_bodies)

        self.planning_method = query.get('planning_method', None)

        request = query.get('request')
        self.args = request['args']
        self.kwargs = request['kw_args']
        self.planning_method = request['method']
