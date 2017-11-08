# bosch_arch_Lifecycle

* [About the Lifecycle](#about)
* [License and Organization](#license)
* [Maintainers and Contributors](#maintainers)
* [Build Instructions](#build)
* [Dependencies on OSS Components](#dependencies)
* [Continuous Integration](#ci)


## <a name="about"/>About bosch_arch_lifecycle

bosch\_arch\_lifecycle provides a component lifecycle implementation for ROSv1. It has been developed as part of the [RoSe](https://inside-docupedia.bosch.com/confluence/display/ROSE/)-project. The repository consists of the following packages:

* **Lifecycle** provides the abstract classes for implementing Life-cycle mechanism to application nodes.  See [**lifecycle/docs/Managed_nodes.pdf**](lifecycle/docs/Managed_nodes.pdf) for a comprehensive description of the core concepts of Node life-cycle. This document has been copied from [http://design.ros2.org/articles/node_lifecycle.html] in May 2016.
* **Lifecycle_python** is similar to Lifecycle but implemented in python.
* **Lifecycle_test_library** provides helper templates/classes to test a Life-cycle managed nodes.
* **Mode_manager** is responsible for knowing and switching the system's global mode.
* **Lm_monitor** provides scripts to monitor the status of the nodes configured using Lifecycle or Lifecycle_python


## <a name="license"/>License and Organization

bosch\_arch\_lifecycle is currently licensed under the BIOS license, cf. LICENSE.txt, but will soon be re-licensed under the Apache Public License v2.0, in preparation for external open sourceing.

bosch\_arch\_lifecycle is hosted at [https://sourcecode.socialcoding.bosch.com/projects/BOSCH_ROSE/repos/bosch_arch_lifecycle/browse]


## <a name="maintainers"/>Maintainers and Contributors

Maintainer:

* [Luetkebohle Ingo (CR/AEA2)](https://connect.bosch.com/profiles/html/profileView.do?userid=20CD8DFB-55C3-4B2C-AD68-1C3819E3B831)
 
Contributors:

* [Yeshwanth Sampangi (RBEI/ETP)](https://connect.bosch.com/profiles/html/profileView.do?userid=7D03705F-2548-42E1-8504-BB4DEFD316D0)
* [Ralph Lange (CR/AEA1)](https://connect.bosch.com/profiles/html/profileView.do?key=2138c9c6-d3b3-41c6-a931-f52abde9f4bd)


## <a name="build"/>Build Instructions

This repository can be build easily on Linux with [ROS-Indigo](http://wiki.ros.org/indigo) using [Catkin_Make](http://wiki.ros.org/catkin/commands/catkin_make). The Catkin_make tools recommends out-of-source builds, to not pollute the source tree with object files and other build artefacts. For example, for building the repository, proceed as follows:

* Create a [catkin workspace](http://wiki.ros.org/ROS/Tutorials).
* Clone Bosch Arch repository (e.g., `git clone https://[MyNTUserName]@cr-scm01.de.bosch.com:8443/scm/rose/bosch-arch.git`) into the src folder
* In the catkin workspace folder call catkin_make.

For executing the unit tests simply call `catkin_make run_tests` within your workspace folder.


## <a name="dependencies"/>Dependencies on OSS Components

Bosch Arch depends on various OSS components (libraries, frameworks, ...), contained in the [external](external) folder. Please see [external/OSS_Licenses.md](external/OSS_Licenses.md) for a list of these components and their licenses.

Furthermore, Bosch Arch depends on external installations of

* [Boost 1.54](http://www.boost.org/doc/libs/1_54_0/) *(required)*
* [ROS](http://www.ros.org/) *(required)*
* [GoogleTest](https://github.com/google/googletest) *(optional, for unit testing)*


## <a name="ci"/>Continuous Integration

The dev-branch of this repository is build continuously using [Jenkins](https://jenkins-ci.org/) at http://si-zrose.si.de.bosch.com:8080/job/CR-ROSE-bosch-arch-devel/

[![Build Status](http://si-zrose.si.de.bosch.com:8080/buildStatus/icon?job=CR-ROSE-bosch-arch-devel)](http://si-zrose.si.de.bosch.com:8080/job/CR-ROSE-bosch-arch-devel/)
