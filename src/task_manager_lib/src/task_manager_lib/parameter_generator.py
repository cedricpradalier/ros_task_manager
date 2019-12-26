#! /usr/bin/env python
# Task Generic configuration

# PACKAGE='task_manager_lib'
# import roslib; roslib.load_manifest(PACKAGE)

from dynamic_reconfigure.parameter_generator_catkin import *

class ParameterListAction:
    Clear=0
    Push=1
    Execute=2

def TaskParameterGenerator():
    gen = ParameterGenerator()
    #       Name                    Type            Description               Default    Min   Max
    gen.add("task_rename",          str_t,    0,    "Task renaming at run-time",  "" )
    gen.add("foreground",           bool_t,    0,    "Task running in foreground",  True )
    gen.add("task_period",       double_t,    0,    "Task period",  -1. )
    gen.add("task_timeout",      double_t,    0,    "Task timeout",  -1. )

    return gen

def TaskParameterListGenerator():
    PL=ParameterListAction()
    gen = TaskParameterGenerator()

    param_list_enum = gen.enum([ gen.const("Clear",      int_t, PL.Clear, "Clear parameter list"),
                           gen.const("Push",         int_t, PL.Push, "Push parameter to the list"),
                           gen.const("Execute",      int_t, PL.Execute, "Execute the parameter list")],
                         "parameter list action")

    gen.add("param_list_action",      int_t,   0,    "What should we do with this goal",  2, edit_method=param_list_enum) 

    return gen

