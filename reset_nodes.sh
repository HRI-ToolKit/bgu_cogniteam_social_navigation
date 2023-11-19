#!/bin/bash

# List all running nodes
nodes=$(ros2 node list)

# Stop each node
for node in $nodes
do
  ros2 lifecycle set $node configure
  ros2 lifecycle set $node deactivate
done
