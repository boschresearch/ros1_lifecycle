#!/usr/bin/env python

import actionlib
from lifecycle_msgs.msg import LifecycleGoal, LifecycleAction, LifecycleResult, Lifecycle

class State(object):
    # primary states
    UNCONFIGURED    = LifecycleGoal.PSTATE_UNCONFIGURED
    INACTIVE        = LifecycleGoal.PSTATE_INACTIVE
    ACTIVE          = LifecycleGoal.PSTATE_ACTIVE
    FINALIZED       = LifecycleGoal.PSTATE_FINALIZED
    # transition states
    ErrorProcessing = LifecycleGoal.TSTATE_ERROR_PROCESSING
    CleaningUp      = LifecycleGoal.TSTATE_CLEANING_UP
    Configuring     = LifecycleGoal.TSTATE_CONFIGURING
    Activating      = LifecycleGoal.TSTATE_ACTIVATING
    Deactivating    = LifecycleGoal.TSTATE_DEACTIVATING
    ShuttingDown    = LifecycleGoal.TSTATE_SHUTTING_DOWN
    
class Transition(object):
    CONFIGURE  = LifecycleGoal.EV_CONFIGURE
    CLEANUP    = LifecycleGoal.EV_CLEANUP
    ACTIVATE   = LifecycleGoal.EV_ACTIVATE
    DEACTIVATE = LifecycleGoal.EV_DEACTIVATE
    SHUTDOWN   = LifecycleGoal.EV_SHUTDOWN
    ERROR      = LifecycleGoal.EV_ERROR
    
class Result_Code(object):
    SUCCESS    = LifecycleGoal.EV_SUCCESS
    FAILURE    = LifecycleGoal.EV_FAILURE

class LifecycleModel(object):
    '''Contains all the dictionary & list items which define the model of the managed_node lifecycle'''
    
    '''Defines all the transitions between a primary state to a secondary state'''
    PRIMARY_STEPS = {
        (State.UNCONFIGURED, Transition.CONFIGURE)  : State.Configuring,
        (State.UNCONFIGURED, Transition.SHUTDOWN)   : State.ShuttingDown,


        (State.INACTIVE, Transition.CLEANUP)        : State.CleaningUp,
        (State.INACTIVE, Transition.ACTIVATE)       : State.Activating,
        (State.INACTIVE, Transition.SHUTDOWN)       : State.ShuttingDown,
        (State.INACTIVE, Transition.ERROR)          : State.ErrorProcessing,

        (State.ACTIVE, Transition.SHUTDOWN)         : State.ShuttingDown,
        (State.ACTIVE, Transition.DEACTIVATE)       : State.Deactivating,
        (State.ACTIVE, Transition.ERROR)            : State.ErrorProcessing
        }
    
    '''Defines all the transitions between a secondary state to a primary state'''
    SECONDARY_STEPS = {
        (State.Configuring, Result_Code.SUCCESS)     : State.INACTIVE,
        (State.Configuring, Result_Code.FAILURE)     : State.UNCONFIGURED,
        
        (State.CleaningUp, Result_Code.SUCCESS)      : State.UNCONFIGURED, # must not fail

        (State.Activating, Result_Code.SUCCESS)      : State.ACTIVE,
        (State.Activating, Result_Code.FAILURE)      : State.INACTIVE,

        (State.ShuttingDown, Result_Code.SUCCESS)    : State.FINALIZED, # must not fail

        (State.Deactivating, Result_Code.SUCCESS)    : State.INACTIVE, # must not fail

        (State.ErrorProcessing, Result_Code.SUCCESS) : State.UNCONFIGURED,
        (State.ErrorProcessing, Result_Code.FAILURE) : State.FINALIZED
        }
    
    '''For defining callback functions'''
    STATE_TRANSITION_VALIDPAIRS = {
        Transition.CONFIGURE   : [(State.UNCONFIGURED,  Transition.CONFIGURE    )],
        Transition.ACTIVATE    : [(State.INACTIVE,      Transition.ACTIVATE     )],
        Transition.DEACTIVATE  : [(State.ACTIVE,        Transition.DEACTIVATE   )],
        Transition.SHUTDOWN    : [(State.ACTIVE,        Transition.SHUTDOWN     ), 
                                  (State.INACTIVE,      Transition.SHUTDOWN     ),
                                  (State.UNCONFIGURED,  Transition.SHUTDOWN     )],
        Transition.CLEANUP     : [(State.INACTIVE,      Transition.CLEANUP      )],
        Transition.ERROR       : [(State.ACTIVE,        Transition.ERROR        ),
                                  (State.INACTIVE,      Transition.ERROR        )]
        }
    
    KNOWN_EVENTS = (LifecycleGoal.EV_SHUTDOWN, 
                    LifecycleGoal.EV_CONFIGURE,
                    LifecycleGoal.EV_CLEANUP,
                    LifecycleGoal.EV_DEACTIVATE,
                    LifecycleGoal.EV_ACTIVATE,
                    LifecycleGoal.EV_ERROR,
                    LifecycleGoal.EV_FAILURE,
                    LifecycleGoal.EV_SUCCESS)
    
    '''Sequence of events to transition into the final state from a given state'''
    EVENTS = {
        (Lifecycle.PSTATE_UNCONFIGURED, Lifecycle.PSTATE_INACTIVE): [LifecycleGoal.EV_CONFIGURE],
        (Lifecycle.PSTATE_UNCONFIGURED, Lifecycle.PSTATE_ACTIVE):   [LifecycleGoal.EV_CONFIGURE,
                                                                     LifecycleGoal.EV_ACTIVATE],
        (Lifecycle.PSTATE_INACTIVE, Lifecycle.PSTATE_UNCONFIGURED): [LifecycleGoal.EV_CLEANUP],
        (Lifecycle.PSTATE_INACTIVE, Lifecycle.PSTATE_ACTIVE):       [LifecycleGoal.EV_ACTIVATE],
        (Lifecycle.PSTATE_ACTIVE, Lifecycle.PSTATE_INACTIVE):       [LifecycleGoal.EV_DEACTIVATE],
        (Lifecycle.PSTATE_ACTIVE, Lifecycle.PSTATE_UNCONFIGURED):   [LifecycleGoal.EV_DEACTIVATE,
                                                                     LifecycleGoal.EV_CLEANUP],
        (Lifecycle.PSTATE_ACTIVE, Lifecycle.PSTATE_FINALIZED):      [LifecycleGoal.EV_SHUTDOWN],
        (Lifecycle.PSTATE_INACTIVE, Lifecycle.PSTATE_FINALIZED):    [LifecycleGoal.EV_SHUTDOWN],
        (Lifecycle.PSTATE_UNCONFIGURED, Lifecycle.PSTATE_FINALIZED):[LifecycleGoal.EV_SHUTDOWN]
        }
    
    '''A Dictionary to translate a State Number to equivalent String. To be used for debugging output'''
    STATE_TO_STR = {
        # primary states
        State.UNCONFIGURED    : 'UNCONFIGURED',
        State.INACTIVE        : 'INACTIVE',
        State.ACTIVE          : 'ACTIVE',
        State.FINALIZED       : 'FINALIZED',
        # transition states     
        State.ErrorProcessing : 'ErrorProcessing',
        State.CleaningUp      : 'CleaningUp',
        State.Configuring     : 'Configuring',
        State.Activating      : 'Activating',
        State.Deactivating    : 'Deactivating',
        State.ShuttingDown    : 'ShuttingDown',
        # Transitions
        Transition.CONFIGURE          : 'CONFIGURE',
        Transition.CLEANUP            : 'CLEANUP',
        Transition.ACTIVATE           : 'ACTIVATE',
        Transition.DEACTIVATE         : 'DEACTIVATE',
        Transition.SHUTDOWN           : 'SHUTDOWN',
        Transition.ERROR              : 'ERROR',
        # Result codes
        Result_Code.SUCCESS            : 'SUCCESS',
        Result_Code.FAILURE            : 'FAILURE'
        }
        
    '''A Dictionary to translate a String to equivalent State Number. To be used for friendly input of parameters'''
    STR_TO_STATE = {
        # primary states
        'UNCONFIGURED'      : State.UNCONFIGURED,
        'Unconfigured'      : State.UNCONFIGURED,
        'unconfigured'      : State.UNCONFIGURED,
        'INACTIVE'          : State.INACTIVE,
        'Inactive'          : State.INACTIVE,
        'inactive'          : State.INACTIVE,
        'ACTIVE'            : State.ACTIVE,
        'Active'            : State.ACTIVE,
        'active'            : State.ACTIVE,
        'FINALIZED'         : State.FINALIZED,
        'Finalized'         : State.FINALIZED,
        'finalized'         : State.FINALIZED,
        # transition states
        'ErrorProcessing'   : State.ErrorProcessing,
        'Errorprocessing'   : State.ErrorProcessing,
        'errorprocessing'   : State.ErrorProcessing,
        'errorProcessing'   : State.ErrorProcessing,
        'CleaningUp'        : State.CleaningUp,
        'Cleaningup'        : State.CleaningUp,
        'cleaningup'        : State.CleaningUp,
        'cleaningUp'        : State.CleaningUp,
        'Configuring'       : State.Configuring,
        'configuring'       : State.Configuring,
        'Activating'        : State.Activating,
        'activating'        : State.Activating,
        'Deactivating'      : State.Deactivating,
        'deactivating'      : State.Deactivating,
        'ShuttingDown'      : State.ShuttingDown,
        'Shuttingdown'      : State.ShuttingDown,
        'shuttingdown'      : State.ShuttingDown,
        'shuttingDown'      : State.ShuttingDown
        }
