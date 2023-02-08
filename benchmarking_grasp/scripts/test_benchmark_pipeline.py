#!/usr/bin/env python3

import rospy
from benchmarking_pipeline_module.benchmarking_pipeline import BenchmarkTest
from std_srvs.srv import Empty

def execute_benchmark(data):
    benchmark_test = BenchmarkTest()

if __name__ == "__main__":
    rospy.init_node("benchmark_test", log_level=rospy.INFO)
    benchmark_test = BenchmarkTest(use_cartesian=False, over_head=False, sim_mode=False)
    rospy.spin()
    