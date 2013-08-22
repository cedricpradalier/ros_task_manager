#! /usr/bin/env python
# Task Generic configuration

# PACKAGE='task_manager_lib'
# import roslib; roslib.load_manifest(PACKAGE)

from dynamic_reconfigure.parameter_generator_catkin import *

def TaskParameterGenerator():
    gen = ParameterGenerator()
    #       Name                    Type            Description               Default    Min   Max
    gen.add("task_rename",          str_t,    0,    "Task renaming at run-time",  "" )
    gen.add("foreground",           bool_t,    0,    "Task running in foreground",  True )
    gen.add("task_period",       double_t,    0,    "Task period",  -1. )
    gen.add("task_timeout",      double_t,    0,    "Task timeout",  -1. )

    return gen

