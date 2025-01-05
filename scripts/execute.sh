#!/bin/bash

# ROS-Umgebung laden
source /opt/ros/noetic/setup.bash
source ~/workspace_rosplan/devel/setup.bash

# Planner ausführen und Plan speichern
/home/jakob/workspace_rosplan/src/rosplan/rosplan_planning_system/common/bin/popf \
    /home/jakob/workspace_rosplan/src/project_package/domains/domain2.pddl \
    /home/jakob/workspace_rosplan/src/project_package/problems/problem2.pddl > plan.txt

# Python-Skript starten
python3 /home/jakob/workspace_rosplan/src/project_package/scripts/move_robot.py
