# Prpy Benchmarks
This package provides a mechanism for running a set of benchmarks. The following sections define the three components of a single benchmark and example usage for this package.

## Planner Metadata
The planner metadata defines the information required to create an instance of the planner. It is comprised of 3 components:
* planner_module - The name of the python module containing the planner (i.e. prpy.planning)
* planner_class_name - The class name of the planner (i.e. SnapPlanner)
* planner_parameters - A dictionary of parameters to be passed to the planner constructor

## Query
A query defines the metadata for running a single planning instance. It is comprised of 4 components:
* planning_method - The planning method to execute (i.e. 'PlanToConfiguration')
* env - The OpenRAVE environment to run the planning instance against
* args - The arguments to pass to the planning method
* kwargs - The keyword arguments to pass to the planning method

## Result
A result contains metadata about the result of a benchmark. It is comprised of 7 components:
* queryfile - The name of the query file used to load the Query
* query_md5 - The md5 checksum of the queryfile
* plannerfile - The name of the planner metadata file used to load the Planner Metadata
* planner_data_md5 - The md5 checksum of the plannerfile
* time - The time it took to execute the benchmark
* success - True if the benchmark successfully found a path
* path - The path output by the planning call

## Usage
To generate planning queries, use `prpy.planning.logged.LoggedPlanner` to wrap an existing planner:
```python
import prpy.planning.logged
planner = prpy.planning.logged.LoggedPlanner(planner)
```

The following call can be used to execute the cartesian product of a set of queries and planner metadatas:
```shell
$ rosrun prpy_benchmarks run_benchmarks.py --queryfiles queries/table_query_0.yaml --plannerfiles \
planners/cbirrt_default.yaml planners/snap_default.yaml 
```
This executes two benchmarks.

Alternatively, you can define a yaml file with a list of queries and planners:
```shell
$ rosrun prpy_benchmarks run_benchmarks.py --benchmarkfiles benchmarks/table_benchmarks.yaml
```

The ```--outdir``` parameter can be passed to the ```run_benchmarks.py``` to indicate that results for each benchmark should be saved in a particular directory. 
