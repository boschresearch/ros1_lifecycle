## Automated Test Library for Lifecycle Manager

### Concept
This library provides methods to test your nodes configured by LifecycleManager. 

#### Transition Test
The user node is transitioned between various states multiple times to check the robustness of the state changes.

For example:

UNCONFIGURED -> INACTIVE -> ACTIVE -> INACTIVE -> UNCONFIGURED -> ACTIVE -> INACTIVE.

It tests all the states except the FINALIZED state since FINALIZED is the end state its tested only once at the end of the other test (behaviour test).

#### Behaviour Test
In this test the user node is transitioned to each state one after another and in each state the behaviour of the node is checked.
For example:

In the UNCONFIGURED state the node must not have any publishers, subscribers or services registered.

In the INACTIVE state the node has to register publishers, subscribers or services but the publishing rate has to be 0 and the service request should not be serviced

In the ACTIVE state the node has to start publishing messages and respond to the service requests

In the FINALIZED state the node must not have any publishers, subscribers or services registered.

### Using the Library
The library can be used inside unit tests for a particular node. Define an object of class LifecycleTestLibrary(component_fqn, pub, hz, sub, srv), and use the methods: test_transitions() and test_all_states for performing the tests.

A python script ("lifecycle_node_test.py") is provided to test a node as a standalone option. If you would like to test only the state transitions you could run (rosrun) this script with node name as parameter.

Syntax: rosrun lifecycle_test_library lifecycle_node_test.py node_name.

For performing the complete test (state transitions and behaviour of node in each state), configure a .yaml file and invoke the lifecycle_node_test.py from a launch file by loading the .yaml file parameters.

For example please see: lifecycle_test_library/examples/

### Configuration
The configuration file is a YAML file which looks as follows:

```yaml
node_name       : "Example_Node"
states          : ['INACTIVE', 'UNCONFIGURED', 'INACTIVE', 'ACTIVE', 'INACTIVE', 'UNCONFIGURED', 'ACTIVE', 'UNCONFIGURED']
timeout         : 3
delay           : 1
publications    : ['/Example_Node/chatter']
publishing_hz   : [10]
subscriptions   : ['/noise']
services        : ['/Example_Node/add_two_ints']
```

#### node_name
Name of your node that has to be tested

#### states
The sequence of the states that the node has to be tested for transitioning. For the state code lookup lifecycle_msg. 

#### timeout
Maximum duration in seconds the node has to take to make the transition

#### delay
The time in seconds between two consecutive transitions

#### publications
The names of the publications done by the node

#### publishing_hz
The rate of the publications

#### subscriptions
The names of the Subscriptions by the node

#### services
The services offered by the node

### Launch
An example launch file: 
```
<launch>
    <node pkg="lifecycle_test_library" type="lifecycle_node_test.py" name="Test_Node" output="screen" required="true">
        <rosparam command="load" file="$(find lifecycle_test_library)/examples/config/example_test.yaml"/>
    </node>

</launch>
```
