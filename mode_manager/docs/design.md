# Overall Mode Manager Design

The mode manager is responsible for knowing and switching the system's global mode. The basic
assumption is that "system modes" are implemented by de-/activating and reconfiguring nodes. 
There is no explicit concept of "component mode" in here. In particular, currently, if a node has
modes, they can only by changed by cycling through the "unconfigured" state.

Furthermore, it needs work well with auxiliary ROS features, such as diagnostics, robot 
descriptions, tf, etc.

In sum, the mode manager does
 * modify node's lifecycles
 * start/stop nodes
 * change ROS parameters

In the current implementation, it does so by delagating to roslaunch, i.e., there need
to be launch files that realize the changes.

# Requirements on managed nodes

Managed nodes need to implement either
 * the lifecycle protocol (see ros_rose/lifecycle)
 * nodelets (-> configuration/activation is mapped to loading the nodelet, deactivation to unloading)
 * we could also support dynamic_reconfigure, by setting a special parameter, but that is not spec'd, yet
 * as a last resort, we could kill and restart nodes

## Requirements on life-cycle nodes

Life-cycle users need to re-request the relevant rosparams from the master on going from unconfigured
to inactive.
