def get_planner(planner_module, planner_class_name, **kwargs):
    """
    Convert a planner name to a planner
    @param planner_module The module the planner can be found in
    @param planner_class_name The planner class to load from the module
    @param kwargs Additional keyword arguments to pass to the class constructor
    """
    import importlib
    m = importlib.import_module(planner_module)
    c = getattr(m, planner_class_name)
    planner = c(**kwargs)
    return planner

class BenchmarkPlannerMetadata(object):

    def __init__(self, planner_module=None, planner_class_name=None, planner_parameters=None):
        """
        @param planner_name The module to load the planner from (as string - i.e. 'prpy.planning')
        @param planner_class_name The name of the planner class (as string - i.e. 'CBiRRTPlanner')
        @param planner_parameters A dictionary of parameters to pass to the planner constructor
        """
        self.planner_module = planner_module
        self.planner_class_name = planner_class_name
        self.planner_parameters = planner_parameters if planner_parameters is not None else dict()

    def to_yaml(self):
        """
        Serialize this planner metadata to a yaml dictionary
        """
        import yaml
        p = { 'planner_module', self.planner_module,
              'planner_class_name', self.planner_class_name,
              'planner_parameters', yaml.dump(self.planner_parameters) }
        return p

    def from_yaml(self, planner_yaml):
        """
        @param planner_yaml The planner metadata in yaml format
        """
        self.planner_module = planner_yaml.get('planner_module', None)
        self.planner_class_name = planner_yaml.get('planner_class_name', None)
        planner_parameters = planner_yaml.get('planner_parameters', None)
        if planner_parameters is not None:
            import yaml
            self.planner_parameters = yaml.load(planner_parameters)
        else:
            self.planner_parameters = dict()
