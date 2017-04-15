ros_task_manager
================

Generic Task Manager for ROS. 

The complete documentation is available on:
https://hal.archives-ouvertes.fr/hal-01435823

Developing a complete robotic system often requires combining multiple
behaviours into a complex decision grid, with elements running in sequence or
in parallel, eventually interrupting each others. To solve this 'age-old'
problem, ROS provides two main tools: Actionlib: a client-server architecture
that provides a way to specify results to be achieved. While the server works
on these results, it should report progresses and ultimately report when the
task is completed. Smach: a python API to define complex state machines. It can
interact with ROS services and actions to define a complex behaviour, including
nesting, interruptions and concurrence. Combining Smach and Actionlib, one
could build arbitrarily complex systems. Hence, why would another task
management system be necessary? The main argument in favour of our task
scheduler is the simplicity of its use, particularly in comparison with
Actionlib. As usual, simplicity is a trade-off against expressiveness.
Simplicity can be sacrificed by linking our task scheduler with Actionlib
and/or Smach to exploit the best of both worlds. This task scheduling framework
is the culmination of about 10 years of experience developing robotic
applications in the academic context. 

Most applications we designed could be handled using the task scheduling framework in this repository.
