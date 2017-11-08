## A Monitor library for Lifecycle Manager : lm_monitor

### Concept
This library provides scripts to monitor the status of nodes configured by LifecycleManager.

#### Echo Script
This script prints the node status of the specified node name. If none specified then it prints all the nodes (configured with lifecycle manager) and their live transitions.For ex:

```
:~/catkin_ws$ rosrun lm_monitor echo Example_Node
Example_Node:	  1465568717232861042 CONFIGURE INACTIVE SUCCESS

Example_Node:	  1465568718253999948 CLEANUP UNCONFIGURED SUCCESS

Example_Node:	  1465568719273547887 CONFIGURE INACTIVE SUCCESS

Example_Node:	  1465568720280272960 ACTIVATE ACTIVE SUCCESS
```

or

```
~/catkin_ws$ rosrun lm_monitor echo
Example_Node:      1465568800460480928 CONFIGURE INACTIVE SUCCESS

Test_run:      1465568800896656036 CLEANUP UNCONFIGURED SUCCESS
Example_Node:     1465568800460480928 CONFIGURE INACTIVE SUCCESS

Test_run:      1465568800896656036 CLEANUP UNCONFIGURED SUCCESS
Example_Node:     1465568801476504087 CLEANUP UNCONFIGURED SUCCESS

Test_run:      1465568801897289037 CONFIGURE INACTIVE SUCCESS
Example_Node:     1465568801476504087 CLEANUP UNCONFIGURED SUCCESS
```