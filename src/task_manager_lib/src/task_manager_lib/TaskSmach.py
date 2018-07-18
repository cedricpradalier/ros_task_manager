import roslib; roslib.load_manifest('task_manager_lib')
from task_manager_lib.TaskClient import *
import smach
import smach_ros
import signal
import sys

# # FIXME: What is the purpose of this class ?
# class MissionFailed(smach.State): 
#     def __init__(self):
#         smach.State.__init__(self, 
#                 outcomes=['MISSION_FAILED'])

# # FIXME: What is the purpose of this class ?
# class MissionCompleted(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, 
#                 outcomes=['MISSION_COMPLETED'])




class TaskState(smach.State):
    def __init__(self,mi,tc,name,**params):
        smach.State.__init__(self, 
                outcomes=['TASK_COMPLETED','TASK_INTERRUPTED',
                    'TASK_FAILED','TASK_TIMEOUT','MISSION_COMPLETED']) # TODO: see how to add new outcomes
        self.mi = mi
        self.tc = tc
        self.name = name
        self.params = params
        self.id = None

    def execute(self, userdata):
        if self.mi.is_shutdown():
            return 'MISSION_COMPLETED'
        # rospy.loginfo('Executing state '+self.name)
        try:
            self.id = self.tc.tasklist[self.name].start(**self.params)
            self.tc.waitTask(self.id)
            return 'TASK_COMPLETED'
        except TaskConditionException: #TODO: use exception condition to dynamically add new outcomes ?
            return 'TASK_INTERRUPTED'
        except TaskException, e:
            if e.status == TaskStatus.TASK_TIMEOUT:
                return 'TASK_TIMEOUT'
            elif e.status == TaskStatus.TASK_INTERRUPTED:
                return 'TASK_INTERRUPTED'
            return 'TASK_FAILED'

    def request_preempt(self):
        if self.id:
            # print "Preempting task %s:%d"%(self.name,self.id)
            self.tc.stopTask(self.id)

class MissionStateMachine:
    def __init__(self,tc=None, new_outcomes=[]):
        self.shutdown_requested = False
        self.pseudo_states={}
        server_node = rospy.get_param("~server","/turtlesim_tasks") # FIXME: why turtlesim_tasks appear here ?
        default_period = rospy.get_param("~period",0.2) # TODO: see if it could be usefull to set the period manually ?
        if tc:
            self.tc = tc
        else:
            self.tc = TaskClient(server_node,default_period)
        # self.tc.verbose = 2

        # Create list of default outcomes
        # ----------------------------------
        default_outcomes = ['TASK_COMPLETED','TASK_INTERRUPTED',
                    'TASK_FAILED','TASK_TIMEOUT','MISSION_COMPLETED']
        for outcome in new_outcomes:
            default_outcomes.append(outcome)
        self.default_outcomes = default_outcomes

    def is_shutdown(self):
        return self.shutdown_requested


    # Generate new name
    # --------------------
    def getLabel(self,name):
        state_name = "__"+name+"_0"
        if name in self.pseudo_states:
            state_name = "__" + name + "_" + str(self.pseudo_states[name])
        else:
            self.pseudo_states[name] = 0
        self.pseudo_states[name] += 1
        return state_name


    # Launch mission
    # -----------------
    class signal_handler:
        def __init__(self,mi,sm):
            self.mi = mi
            self.sm = sm

        def __call__(self,signal,frame):
            # print("Signal %s detected" % str(signal))
            self.mi.shutdown_requested = True
            self.sm.request_preempt()

    def run(self,sm):
        self.shutdown_requested = False
        sis = smach_ros.IntrospectionServer('mission_state_machine', sm, '/SM')
        sis.start()

        # Execute SMACH tree in a separate thread so that we can ctrl-c the script
        signal.signal(signal.SIGINT, self.signal_handler(self,sm))
        smach_thread = threading.Thread(target = sm.execute)
        smach_thread.start()

        while sm.is_running():
            rospy.rostime.wallsleep(0.5)

        sis.stop()


    #############################
    #####  Smash surcharge  #####
    #############################

    # State Machine
    # ----------------
    def StateMachine(self, outcomes = [], input_keys = [], output_keys = []):
        return self.StateMachineC(self, outcomes = outcomes, input_keys = input_keys, output_keys = output_keys)

    class StateMachineC(smach.StateMachine):
        """ Smash StateMachine surcharge

        bla bla
        """
        def __init__(self, mi, outcomes = [], input_keys = [], output_keys = []):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.StateMachine.__init__(self, outcomes = temp_outcomes, input_keys = input_keys, output_keys = output_keys)

        def add(self, label, state = None, transitions = None, remapping = None, **params):
            params['foreground'] = True
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.StateMachine.add(label, state, transitions, remapping)
            return label


    # Concurrence
    # --------------
    def Concurrence(self, outcomes = [], default_outcome = 'TASK_FAILED', input_keys = [], output_keys = [], 
                outcome_map = {}, outcome_cb = None, child_termination_cb = None, wait_for_all = True):
        return self.ConcurrenceC(self, outcomes = outcomes, default_outcome = default_outcome,
                    input_keys = input_keys, output_keys = output_keys, outcome_map = outcome_map,
                    outcome_cb = outcome_cb, child_termination_cb = child_termination_cb,
                    wait_for_all = wait_for_all)

    class ConcurrenceC(smach.Concurrence):
        """ Smash Concurrence surcharge

        bla bla
        """
        def __init__(self, mi, outcomes = [], default_outcome = 'TASK_FAILED', input_keys = [], output_keys = [], 
                    outcome_map = {}, outcome_cb = None, child_termination_cb = None, wait_for_all = True):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            if not outcome_cb:
                outcome_cb = self.concurrent_default_outcome_cb(self.mi)
            if not child_termination_cb:
                if wait_for_all:
                    new_termination_cb = lambda x:False
                else:
                    new_termination_cb = lambda x:True
            else:
                new_termination_cb = child_termination_cb

            smach.Concurrence.__init__(self, outcomes = temp_outcomes, default_outcome = default_outcome,
                    input_keys = input_keys, output_keys = output_keys, outcome_map = outcome_map,
                    outcome_cb = outcome_cb, child_termination_cb = new_termination_cb)

        def add(self, label, state = None, remapping = None, **params):
            # if not params['foreground']:
            if not 'foreground' in params:
                params['foreground'] = False # foreground default is False #TODO: check if all background works ?
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.Concurrence.add(label, state, remapping)
            return label

        class concurrent_default_outcome_cb:
            def __init__(self,mi):
                self.mi = mi
            def __call__(self,states):
                print states
                if self.mi.is_shutdown():
                    return 'TASK_INTERRUPTED'
                num_complete = sum([1 for x in states.values() if x == 'TASK_COMPLETED'])
                if len(states) == num_complete: 
                    return 'TASK_COMPLETED'
                return 'TASK_FAILED'


    # Sequence
    # -----------
    def Sequence(self, outcomes = [], connector_outcome = 'TASK_COMPLETED', input_keys = [], output_keys = []):
        return self.SequenceC(self, outcomes = outcomes, connector_outcome = connector_outcome, 
                    input_keys = input_keys, output_keys = output_keys)

    class SequenceC(smach.Sequence):
        """ Smash Sequence surcharge

        bla bla
        """
        def __init__(self, mi, outcomes = [], connector_outcome = 'TASK_COMPLETED', input_keys = [], output_keys = []):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.Sequence.__init__(self, outcomes = temp_outcomes, connector_outcome = connector_outcome,
                        input_keys = input_keys, output_keys = output_keys)

        def add(self, label, state = None, transitions = None, remapping = None, **params):
            params['foreground'] = True
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.Sequence.add(label, state, transitions, remapping)
            return label


    # Iterator
    # -----------
    def Iterator(self, outcomes = [], input_keys = [], output_keys = [], it = [], 
                it_label = 'it_data', exhausted_outcome = 'TASK_COMPLETED'):
        return self.IteratorC(self, outcomes = outcomes, input_keys = input_keys, output_keys = output_keys,
                    it = it, it_label = it_label, exhausted_outcome = exhausted_outcome)

    class IteratorC(smach.Iterator):
        """ Smash Iterator surcharge

        bla bla
        """
        def __init__(self, mi, outcomes = [], input_keys = [], output_keys = [], it = [], 
                    it_label = 'it_data', exhausted_outcome = 'TASK_COMPLETED'):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.Iterator.__init__(self, outcomes = temp_outcomes, 
                        input_keys = input_keys, output_keys = output_keys,
                        it = it, it_label = it_label, exhausted_outcome = exhausted_outcome)


    # Epsilon task
    # -----------
    class TaskEpsilon(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['TASK_COMPLETED','TASK_FAILED'])
        def execute(self, userdata):
            return 'TASK_COMPLETED'

    # TODO: see what happen if we want to wait for all task to reach epsilon before returning completed
    def epsilon_task(self,label=None,transitions=None):
        if not label:
            label=self.getLabel("Epsilon")
        if transitions:
            smach.Sequence.add(label, self.TaskEpsilon(),transitions)
        else:
            smach.Sequence.add(label, self.TaskEpsilon())
        return label



    #############################
    #####  Intern function  #####
    #############################

    # def getLabel(self,name):
    #     state_name = "__"+name+"_0"
    #     if name in self.pseudo_states:
    #         state_name = "__" + name + "_" + str(self.pseudo_states[name])
    #     else:
    #         self.pseudo_states[name] = 0
    #     self.pseudo_states[name] += 1
    #     return state_name






    # ########################
    # #####  Deprecated  #####
    # ######################## TODO: REMOVE

    # def createStateMachine(self):
    #     return smach.StateMachine(outcomes=['TASK_COMPLETED','TASK_INTERRUPTED',
    #                 'TASK_FAILED','TASK_TIMEOUT','MISSION_COMPLETED'])

    # def createSequence(self):
    #     return smach.Sequence(outcomes=['TASK_COMPLETED','TASK_INTERRUPTED',
    #                 'TASK_FAILED','TASK_TIMEOUT','MISSION_COMPLETED'],
    #             connector_outcome='TASK_COMPLETED')

    # class concurrent_outcome_cb:
    #     def __init__(self,mi,fg):
    #         self.mi = mi
    #         self.fg = fg
    #     def __call__(self,states):
    #         print states
    #         if self.mi.is_shutdown():
    #             return 'TASK_INTERRUPTED'
    #         num_complete = sum([1 for x in states.values() if x == 'TASK_COMPLETED'])
    #         if num_complete>=1: #FIXME: meaning it works if only one worked ?!
    #             return 'TASK_COMPLETED'
    #         return states[fg] #FIXME: what is fg ? fallback output ? -> front task

    # def createConcurrence(self,fg_state):
    #     # Create the sub SMACH state machine
    #     return smach.Concurrence(outcomes=['TASK_COMPLETED','TASK_INTERRUPTED',
    #                 'TASK_FAILED','TASK_TIMEOUT','MISSION_COMPLETED'], default_outcome='TASK_FAILED',
    #                 outcome_cb = self.concurrent_outcome_cb(self,fg_state),
    #                 child_termination_cb=lambda x:True) #FIXME: Lambda function not right

    # def task(self,name,**params):
    #     params['foreground']=True
    #     state_name = None
    #     if 'label' in params:
    #         state_name=params['label']
    #         del params['label']
    #     else:
    #         state_name = self.getLabel(name)
    #     T=params['transitions']
    #     del params['transitions']
    #     smach.StateMachine.add(state_name, TaskState(self,self.tc,name,**params),T)
    #     return state_name

    # def seq_task(self,name,**params):
    #     state_name = None
    #     params['foreground']=True
    #     if 'label' in params:
    #         state_name=params['label']
    #         del params['label']
    #     else:
    #         state_name = self.getLabel(name)
    #     if 'transitions' in params:
    #         T=params['transitions']
    #         del params['transitions']
    #         smach.Sequence.add(state_name, TaskState(self,self.tc,name,**params),T)
    #     else:
    #         smach.Sequence.add(state_name, TaskState(self,self.tc,name,**params))
    #     return state_name

    # def concurrent_task(self,name,**params):
    #     state_name = self.getLabel(name)
    #     foreground = params['foreground'] # This must be defined
    #     if 'label' in params:
    #         state_name=params['label']
    #         del params['label']
    #     smach.Concurrence.add(state_name, TaskState(self,self.tc,name,**params))
    #     return state_name
