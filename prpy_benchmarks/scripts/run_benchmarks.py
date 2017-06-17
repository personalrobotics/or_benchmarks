#!/usr/bin/env python
pkg = 'prpy_benchmarks'
import logging
logger = logging.getLogger(pkg)

if __name__ == '__main__':
    
    import argparse
    parser = argparse.ArgumentParser('Execute a single benchmark')
    parser.add_argument("-b", "--benchmarkfiles", type=str, nargs='+',
                        help="A yaml file defining a set of queries and planners to execute")
    parser.add_argument("-q", "--queryfiles", type=str, nargs='+', default=None,
                        help="The query file to execute")
    parser.add_argument("-p", "--plannerfiles", type=str, nargs='+', default=None,
                        help="The file containing planner metadata to use for this benchmark")
    parser.add_argument("-o", "--outdir", type=str, default=None,
                        help="The directory to write results to")

    args = parser.parse_args()

    querylist = args.queryfiles if args.queryfiles else list()
    plannerlist = args.plannerfiles if args.plannerfiles else list()
    benchmarkfiles = args.benchmarkfiles if args.benchmarkfiles else list()


    import herbpy #TODO: Remove
    from prpy_benchmarks.benchmark_utils import execute_benchmark
    import itertools, os
    from os.path import basename, join, splitext
    from prpy.util import FindCatkinResource

    for b in benchmarkfiles:
        import yaml
        with open(b, 'r') as f:
            data = yaml.load(f.read())
        for q in data['queries']:
            querylist.append(FindCatkinResource(pkg, join('queries', q)))
                                                   
        for p in data['planners']:
            plannerlist.append(FindCatkinResource(pkg, join('planners', p)))

    env, robot = herbpy.initialize(sim=True) # TODO: Remove

    # Make output directory, if necessary
    if args.outdir and not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    # Execute cartesian product of queries and planners
    outfile = None    
    for b in itertools.product(set(querylist), set(plannerlist)):
        qfile = b[0] # Query file
        pfile = b[1] # Planner metadata file

        # Compute a unique filename for this test
        if args.outdir:
            qname = splitext(basename(qfile))[0]
            pname = splitext(basename(pfile))[0]
            outfile = join(args.outdir, '%s_%s.result' % (qname, pname))

        # Go!
        result = execute_benchmark(qfile, pfile,
                                   env=env, robot=robot,
                                   outfile=outfile)
                        
