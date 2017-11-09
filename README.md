# bosch_arch_Lifecycle

* [About the Lifecycle](#about)
* [License and Organization](#license)
* [Maintainers and Contributors](#maintainers)
* [Build Instructions](#build)
* [Dependencies on OSS Components](#dependencies)
* [Continuous Integration](#ci)


## <a name="about"/>About bosch_arch_lifecycle

ros1_lifecycle provides a component lifecycle implementation for ROSv1.

* **Lifecycle** provides the abstract classes for implementing Life-cycle mechanism to application nodes.  See [**lifecycle/docs/Managed_nodes.pdf**](http://design.ros2.org/articles/node_lifecycle.html) for a comprehensive description of the core concepts of Node life-cycle. This document has been copied from [http://design.ros2.org/articles/node_lifecycle.html] in May 2016.
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
 
Contributors:

* [Ralph Lange (CR/AEE1)](https://github.com/iluetkeb)


## <a name="build"/>Build Instructions

This repository can be build easily on Linux with [ROS-Indigo](http://wiki.ros.org/indigo) using [Catkin_Make](http://wiki.ros.org/catkin/commands/catkin_make). The Catkin_make tools recommends out-of-source builds, to not pollute the source tree with object files and other build artefacts. For example, for building the repository, proceed as follows:

* Create a [catkin workspace](http://wiki.ros.org/ROS/Tutorials).
* Clone repository (e.g., `git clone https://github.com/bosch-robotics-cr/ros1_lifecycle`) into the src folder
* In the catkin workspace folder call catkin_make.

For executing the unit tests simply call `catkin_make run_tests` within your workspace folder.


