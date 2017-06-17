#!/usr/bin/env python
pkg = 'prpy_benchmarks'
import yaml
from prpy_benchmarks.query import BenchmarkQuery

if __name__ == '__main__':
    
    import argparse
    parser = argparse.ArgumentParser('Execute a single benchmark')
    parser.add_argument("-l", "--logfile", type=str,
                        help="A yaml file logged from prpy.logger")
    
    args = parser.parse_args()

    bq = BenchmarkQuery()
    bq.from_yaml(args.logfile)

    import IPython; 
    IPython.embed()

