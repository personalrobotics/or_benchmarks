# Collision Checker Benchmarking
(Assumes that planner benchmarking has been covered and the `queryfile` and `plannerfile` concepts are understood)

There are two stages in the collision checker benchmarking pipeline:
* Log the collision check queries made by a planner during a planning call
* Evaluate a particular collision checker engine on a log of collision check queries

## Collision Check Logging
A `queryfile` and `plannerfile` should already exist to represent the planning call and the planner for which you wish to log the collision check queries. You need to call `execute_benchmark` which is in `prpy_benchmarks/src/prpy_benchmarks/benchmark_utils.py`, with the `log_collision_checks` flag set to `True`. Please provide a `JSON` file as the `outfile` argument to `execute_benchmark`. Note that you cannot simultaneously log the planner information as well as the collision check queries.

Internally, the method creates a logged collision checker via `or_stub_checker` which records all the information relevant to the query, such as the kind of check invoked, the relevant DOF values, and so on. This is then saved as a `dict` with the information necessary to recreate the collision check. The 1OpenRAVE` environment is also serialized using `prpy.serialization`

## Collision Checker Evaluation
Once you have a collision check query log from the previous stage, you may evaluate the collision checker engine of your choice on it. Refer to the `evaluate_collisioncheck_benchmark` method in `prpy_benchmarks/src/prpy_benchmarks/benchmark_utils.py`. For the `collresultfile` argument, provide a log file obtained from the previous step.

The evaluator notes the time and the result `(True/False)` for the collision check by the specified checker engine. It also separately notes the time for a self-collision and an env-collision check. By default, only the activeDOFs are considered, and the env-collision is checked first (as it is typically shorter).