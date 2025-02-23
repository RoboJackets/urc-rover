# URC Behavior Tree

This package provides the central orchestrator for running the behavior tree.

## Launching

You should run the `urc_bt_orchestrator` node, which is registered as a plugin in this package. For an example, check out `bt.launch.py` under the `urc_bringup` package.

There are several parameters for the `urc_bt_orchestrator` node:

- `node_lib_dirs`: *array of strings*, specifying the path to dynamic library files needed for bt nodes. Will be loaded on runtime.
- `tree_file_dir`: *string*, to the path of the .xml file for the behavior tree. If not specified, the tree will not be loaded and started on launching.
- `start_bridge`: *bool*, whether to initialize some of the convenient variables in the root Blackboard of the current behavior tree.
  
## Starting, Stopping, Hot Swapping

To start or stop `bt_orchestrator`, call `~/start_bt` or `~/stop_bt`. Note that the orchestrator is still alive regardless of whether it is started or not; starting and stopping states determines whether the tree is going to tick the nodes.

`bt_orchestrator` enables hot-swapping by using service at runtime. After editing the tree, you do not need to restart everything. Instead, you have two options:

- Calling `~/update_tree` with service type `UpdateBehaviorTree` under `urc_msgs`. You can choose to update the tree using the path to an existing .xml BT tree file, or you can directly pass the content of the tree in the string.
- Calling `~/reload` with service type `Trigger` under `std_srvs`. In this case, you have to preload the tree with a tree_dir. The orchestrator will by-default load that file.

## Editing the Tree

You can specify path to your tree during launch. For an example, check out `strategies` folder under `urc_bringup` package, which contains `.btproj` containing definitions for all the nodes that is going to be used inside URC.

Note that to provide a bridge between ros and bt, upon launch, `urc_bt_orchestrator` writes several values to the root Blackboard of the current behavior tree by default. They are:

- `ros_nh`: *shared ptr*, node handle to a ros node.
- `ros_log`: *shared ptr*, a ros logger.

This functionality can be disabled by changing the settings.

## Creating Custom Nodes

### Program and Compile the New Node

There are generally two types of nodes: raw BTCPP nodes and BTCPP.ROS2 nodes.

`urc_bt` use dynamic libraries to enable hot load / unload of nodes at runtime, and thus this is the recommended way to extend the node library under `urc_bt_nodes`.

Note that two types of nodes have to be registered differently: raw BTCPP nodes can be registered in multiples in one plugin, while for BTCPP.ROS2 nodes, you have to create one plugin for one node, and manually register them inside the cmake file. Generally, you can follow the template created by existing nodes.

### Register the Node in Groot2

After creating your nodes, you have to manually register them inside Groot2. Note that having your nodes' dynamic library loaded does not mean Groot2 will automatically get the definition of the nodes: Groot2 is only an editor and does not know the actual node implementation.

You can register a new node by clicking the "plus" icon inside Groot2 and get all the input and output ports registered correctly.

For more info on the nodes available, check [Nodes Library](https://www.behaviortree.dev/docs/category/nodes-library) from the official BT.CPP website.
