cmake_minimum_required(VERSION 2.8.3)
project(taurob_robots_description)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(
    PROGRAMS
        script/initial_joint_angles.py
        script/publish_flippers.py
        script/rqt_plot_data_publisher.py
        script/visualization_joints.py
        script/robot_footprint_publisher.py
        script/move_box_node.py
        script/filter_box.py
        script/move_box_circular.py
        script/move_box_triangular.py
        script/move_3_box.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
