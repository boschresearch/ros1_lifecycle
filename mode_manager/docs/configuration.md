The configuration file is a YAML file which looks as follows:

```yaml
modes:
  launched:
    component_configuration:
      param: inactive
  F1:
    inherits: launched
    component_configuration:
      param: active
initial_mode: launched
components:
  param:
    configs:
      inactive:
        entry:
          type: roslaunch
          launch_file: $(find mode_manager)/examples/launch/set_param_to_arg.launch
          args: value:=inactive
      active:
        entry:
          type: roslaunch
          launch_file: $(find mode_manager)/examples/launch/set_param_to_arg.launch
          args: value:=active
    initial_config: inactive
```

## Configuration Variables

### modes

Contains a *dictionary* that defines the modes of the system. The first-level keys are the names of the modes.

For each mode, we currently have one mandatory and one optional configuration element:

#### component_configuration

A dictionary that specifies the configuration for each component

This dictionary must specify the modes for *all* components known to the mode manager.

*Note* It is not currently checked that the configuration dictionary is complete, but this will be added in the near future.

#### inherits (optional): Specifies a mode whose component_configuration will be copied into this one

So, in the example above, the mode "F1" inherits the mode "launched", and then overwrites the configuration for the "param" component. 

When there is only one component, this doesn't give you a lot, but when there are many components, and modes modify only one or a few of them, it saves you from having to configure all of them for every mode.

### initial_mode

This specifies the mode that the mode manager activates on start-up.

### components

This specifies the component configurations. The first-level keys are the names of the components

There are two mandatory sub-elements

#### configs

The dictionary of configurations for this component. The only mandatory element is "type". Depending on type, other elements may also be mandatory. 

##### Type roslaunch

Mandatory: launch_file --> path to launch file, with the usual variable expressions known from roslaunch
Optional: args --> Arguments given to roslaunch.

#### initial_config

Currently specifies the initial component configuration. This duplicates information from the modes and will be removed. It is still necessary right now, however.

##### Type "none"

This is an empty configuration action, which simply does nothing. It can be useful to provide an "inactive" mode,
as something to switch out into.
