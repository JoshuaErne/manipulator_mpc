#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String

header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
robot_name = "<robot name=\"obstacles\">\n"
end_robot = "</robot>"
indent = "  "


def processFile():
    # Get file location from ros param
    rospy.init_node('worldConverter', anonymous=True)
    file_loc = rospy.get_param('map_file')
    count = 0
    # get header for URDF
    urdf = header + robot_name + worldLink()
    # read the file and process each block
    with open(file_loc, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline()
        while line != '':  # The EOF char is an empty string
            line = reader.readline()
            if len(line) > 0 and line[0] == 'b':
                words = line.split()
                # Get the boundaries and origin for each block
                if words[0] == "block":
                    boundaries = [float(words[i]) / 1000 for i in range(1, len(words))]
                    origin = [average(boundaries[i], boundaries[i + 3]) for i in range(3)]
                    size = [abs(boundaries[i] - boundaries[i + 3]) for i in range(3)]
                    # Add the object to the urdf
                    urdf = urdf + convertTextString(origin, size, count)
                    count = count + 1
        urdf = urdf + end_robot
        # Export the urdf to the parameter server
        rospy.set_param('obstacle_urdf', urdf)


# adds the world object to urdf
def worldLink():
    output = indent + "<link name = \"world\">\n"
    output = output + indent + "</link> \n"
    return output


# adds inertial tag to a link
def inertial():
    output = indent + indent + "<inertial> \n"
    output = output + indent + indent + indent + "<mass value = \"0.3\"/>\n"
    output = output + indent + indent + indent + "<inertia ixx=\"0.001\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.001\" iyz=\"0.0\" izz=\"0.001\"/>\n"
    output = output + indent + indent + "</inertial> \n"
    return output


# adds link and joints to urdf
def convertTextString(origin, size, number):
    output = indent + "<link name = \"obstacle_" + str(number) + "\">\n"
    # Visual tag
    output = output + indent + indent + "<visual>\n"
    output = output + indent + indent + indent + "<origin xyz = \"" + str(origin[0]) + " " + str(origin[1]) + " " + str(
        origin[2]) + "\" "
    output = output + "rpy = \"0 0 0\" /> \n"
    output = output + indent + indent + indent + "<geometry> \n"
    output = output + indent + indent + indent + indent + "<box size=\"" + str(size[0]) + " " + str(
        size[1]) + " " + str(size[2]) + "\" />\n"
    output = output + indent + indent + indent + "</geometry> \n"

    output = output + indent + indent + "</visual> \n"

    # Collision tag
    output = output + indent + indent + "<collision>\n"
    output = output + indent + indent + indent + "<origin xyz = \"" + str(origin[0]) + " " + str(origin[1]) + " " + str(
        origin[2]) + "\" "
    output = output + "rpy = \"0 0 0\" /> \n"
    output = output + indent + indent + indent + "<geometry> \n"
    output = output + indent + indent + indent + indent + "<box size=\"" + str(size[0]) + " " + str(
        size[1]) + " " + str(size[2]) + "\" />\n"
    output = output + indent + indent + indent + "</geometry> \n"
    output = output + indent + indent + "</collision> \n"

    output = output + inertial()

    # Add joint connecting to world
    output = output + indent + "</link> \n"
    output = output + indent + "<joint name = \"joint" + str(number) + "\" type = \"fixed\">\n"
    output = output + indent + indent + "<parent link =\"world\"/>\n"
    output = output + indent + indent + "<child link =\"obstacle_" + str(number) + "\"/>\n"
    output = output + indent + "</joint> \n"

    output = output + indent + "<gazebo reference=\"obstacle_" + str(number) + "\"><material>Gazebo/Wood</material></gazebo>\n"

    return output


def average(in1, in2):
    return in1 / 2 + in2 / 2


if __name__ == '__main__':
    try:
        processFile()
    except rospy.ROSInterruptException:
        pass
