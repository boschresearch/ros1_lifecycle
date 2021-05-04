# ROS1 Lifecycle

## <a name="about"/>About ros1_lifecycle

ros1_lifecycle provides a component lifecycle implementation for ROSv1.

* **Lifecycle** provides the abstract classes for implementing Life-cycle mechanism to application nodes.  See [ROS2 Managed Nodes](http://design.ros2.org/articles/node_lifecycle.html) for a comprehensive description of the core concepts of Node life-cycle.
* **Lifecycle_python** is similar to Lifecycle but implemented in python.
* **Lifecycle_test_library** provides helper templates/classes to test a Life-cycle managed nodes.
* **Mode_manager** is responsible for knowing and switching the system's global mode.
* **Lm_monitor** provides scripts to monitor the status of the nodes configured using Lifecycle or Lifecycle_python


## <a name="license"/>License and Organization

bosch\_arch\_lifecycle is currently licensed under the Apache Software License v2, cf. [LICENSE](LICENSE).

bosch\_arch\_lifecycle is hosted at [https://github.com/bosch-robotics-cr/ros1_lifecycle]


## <a name="maintainers"/>Maintainers and Contributors

Maintainer:

* [Luetkebohle Ingo (CR/AEX3)](https://github.com/iluetkeb)

Authors:

* Yeshwanth Sampangi (Bosch Engineering India)
* [Luetkebohle Ingo (Bosch Germany)](https://github.com/iluetkeb)
* [Ralph Lange (Bosch Germany)](https://github.com/ralph-lange)

