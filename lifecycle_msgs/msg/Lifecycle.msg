# NOTE: These are duplicated in the action!
# Primary states
int8 PSTATE_UNCONFIGURED    = 0
int8 PSTATE_INACTIVE        = 1
int8 PSTATE_ACTIVE          = 2
int8 PSTATE_FINALIZED       = 3
# Transition states
int8 TSTATE_CONFIGURING     = 10
int8 TSTATE_CLEANING_UP     = 11
int8 TSTATE_SHUTTING_DOWN   = 12
int8 TSTATE_ERROR_PROCESSING= 13
int8 TSTATE_ACTIVATING      = 14
int8 TSTATE_DEACTIVATING    = 15
# Transitions
int8 EV_CONFIGURE           = 20
int8 EV_CLEANUP             = 21
int8 EV_ACTIVATE            = 22
int8 EV_DEACTIVATE          = 23
int8 EV_SHUTDOWN            = 24
int8 EV_ERROR               = 25
# Result codes
int8 EV_SUCCESS             = 26
int8 EV_FAILURE             = 27

# The requested transition
int8 transition

# The state that has been reached after finishing the transition
int8 end_state

# The result code from the transition state
int8 result_code 
