import numpy as np
import matplotlib.pyplot as plt
from .bvh_scanner import BVHScanner

class Joint():
    """
        This class is the representation of a joint
        It contains informations about its name, parent, positions according to its parent and index in skeleton hierarchy
    """

    def __init__(self, name, parent, index_in_hierarchy=None):
        """ Initialize the joint object with its name, parent joint and its index in the hierarchy (if known) """

        self.name = name
        self.parent = parent
        self.local_offset = np.zeros(3)
        self.global_offset = np.zeros(3)
        self.channels = []
        self.children = []
        self.index = index_in_hierarchy

    def add_child(self, joint):
        """
            Add child joint to its children list

            :param joint: Joint to add to children list
            :type joint: Joint Object
        """
        self.children.append(joint)

    def add_channels(self, channel):
        """
            Add channel to its channels list

            :param channel: Channel to add to channels list
            :type channel: string
        """
        self.channels.append(channel)


class BVHParser():
    """
        This class is in charge of parsing BVH files
    """

    def __init__(self):
        """ Initialize the BVHParser """

        self.parser = BVHScanner()
        self.joints = {}
        self.root = None
        self.frames = None
        self.nb_frames= 0
        self.fps = 0

    def parse(self, path):
        """
            Parse the given BVH file

            :param path: path of the bvh file we want to parse
            :type path: string
        """

        hierarchy, motion = self.parser.scan(path)
        self.parse_hierarchy(hierarchy)
        self.parse_motion(motion)

    def parse_hierarchy(self, bvh):
        """
            Parse the hierarchy of the previously scanned file and create joints object

            :param bvh: previously scanned hierarchy (first part of a BVH file)
            :type bvh: list of tuples
        """

        # this stack will temporarily contains Joint objects, when all infos about one of them is read, remove it from stack
        temp_stack = []
        
        nb_line = 0
        joint_created = 0
        
        # If first element of hierarchy part is not HIERARCHY keyword, stop the process, there is a deformity
        if bvh[nb_line] != ("IDENTIFIER", "HIERARCHY"):
            return None
        nb_line += 1

        while nb_line != len(bvh):
            
            # If current word is ROOT or JOINT
            if bvh[nb_line] == ("IDENTIFIER", "ROOT") or bvh[nb_line] == ("IDENTIFIER", "JOINT"):
                
                # If word isn't root, get the joint parent (last element of the stack)
                parent = temp_stack[-1] if bvh[nb_line][1] == "JOINT" else None
                
                # Read next element, name of joint or root's name and create corresponding Joint Object
                nb_line += 1
                joint = Joint(bvh[nb_line][1], parent, joint_created)
                joint_created += 1

                # Save the created joint in the joints dictionnary
                self.joints[bvh[nb_line][1]] = joint

                # If this joint is not the root, add this joint to its parent's children or set root
                if parent != None:
                    parent.add_child(joint)
                else:
                    self.root = joint
                
                temp_stack.append(joint)

            # If current word is CHANNELS
            elif bvh[nb_line] == ("IDENTIFIER", "CHANNELS"):
                
                # Read number of channels
                nb_line += 1
                if bvh[nb_line][0] != "DIGIT":
                    raise Exception
                
                # Read the given channel names and add them to the list of channels of the last element of the stack
                for i in range(int(bvh[nb_line][1])):
                    nb_line += 1
                    temp_stack[-1].add_channels(bvh[nb_line][1])
            
            # If current word is OFFSET
            elif bvh[nb_line] == ("IDENTIFIER", "OFFSET"):
                local_offset = np.zeros(3, dtype=np.float64)

                # Read the three given offset as local_offset
                for i in range(3):
                    nb_line += 1
                    local_offset[i] = np.float64(bvh[nb_line][1])
                temp_stack[-1].local_offset = local_offset

                # Compute global offset of the current joint with its parent one, except if this is root.
                # In that case, root's global offset is its local offset
                if temp_stack[-1].parent != None:
                    temp_stack[-1].global_offset = temp_stack[-1].parent.global_offset + local_offset
                else:
                    temp_stack[-1].global_offset = local_offset 
            
            # If current word is End
            elif bvh[nb_line] == ("IDENTIFIER", "End"):

                # Create an end site joint after current joint
                joint = Joint(temp_stack[-1].name+"_end", temp_stack[-1])
                temp_stack[-1].add_child(joint)
                temp_stack.append(joint)
                self.joints[joint.name] = joint
            
            # If current element is a }
            elif bvh[nb_line][0] == "CLOSE":

                # Parser read all infos on the current joint, so remove it from the stack
                temp_stack.pop()
            
            nb_line += 1

    def parse_motion(self, bvh):
        """
            Parse the motion part of the previously scanned BVH file and keep informations of each frame.

            :param bvh: previously scanned motion part (2nd part of a BVH file)
            :type bvh: string
        """

        motion_part = bvh.split("\n")

        frame = 0
        for line in motion_part:  
            if line == "":
                continue

            if "Frames" in line:
                self.nb_frames = int(line.split(" ")[1])
                continue

            if "Frame Time" in line:
                self.fps = round(1.0/float(self.nb_frames))
                continue

            digits = line.split(" ")

            if digits[-1] == "":
                del digits[-1]

            if self.frames is None:
                self.frames = np.empty((self.nb_frames, len(digits)), dtype=np.float32)

            for index_coordinates in range(len(digits)):
                self.frames[frame, index_coordinates] = float(digits[index_coordinates])

            frame += 1

    def get_joints_list(self):
        """
            Get the list of joints object representing usefull joints, exclude joint with _end if their names

            :return: list of joints
            :rtype: list of Joint Object
        """

        return [self.joints[value] for _, value in enumerate(self.joints) if not value.endswith("_end")]

    def print_hierarchy(self):
        """ Print hierarchy recursively, start with the root joint """
        
        print("┌ " + str(self.root.index) + " - " + self.root.name + '(' +  str(self.root.global_offset) + ")")
        self.print_joint(self.root, 0)

    def print_joint(self, joint, depth):
        """ 
            Print current joint and go through its children

            :param joint: current joint to print
            :type joint: Joint Object
            :param depth: current depth, just for the indentation in terminal when printing
            :type depth: int
        """

        for child in joint.children:
            if not child.children and child.name.endswith("_end"): #End site joint
                continue
            print(depth*"|   ", end="")
            print("└ "+ str(child.index) + " - " + child.name + '(' + str(child.global_offset) + ")")
            self.print_joint(child, depth+1)

    def plot_hierarchy(self):
        """
            Plot the skeleton hierarchy in 3D
            (use of recursive method to add local offset to each joint)
        """

        self.coords = []
        self.coords.append(self.root.global_offset)
        for child in self.root.children:
            self.coords.append(child.global_offset)
            self.plot_joint(child)
        self.coords = np.array(self.coords)

        plt.scatter(self.coords[:, 0], self.coords[:, 1])
        plt.show()


    def plot_joint(self, joint):
        """
            Get informations about given joint's children and append them to the list of coordinates to plot
        
            :param joint: joint to plot
            :type joint: Joint Object
        """
        for child in joint.children:
            if child.name.endswith("_end"):
                continue
            self.coords.append(child.global_offset)
            self.plot_joint(child)

    def delete_joint(self, name, inclusive=False):
        """
            Delete joint from the list of joints
            If the joint we want to delete has children, delete them too

            :param name: name of the targeted joint
            :type name: string
            :param inclusive: True if we also want to delete the targeted joint, False if we want to delete its children
            :type inclusive: bool
        """

        self.search_joint(self.root, name, inclusive)


    def search_joint(self, joint, name, inclusive):
        """
            Search recursively for the targeted joint to delete

            :param joint: current joint to explore
            :type joint: Joint Object
            :param name: name of the targeted joint
            :type name: string
            :param inclusive: whether we want to keep targeted joint or not
            :type inclusive: bool
        """

        joint_to_delete= None

        for child in joint.children:
            if not child.children:
                continue
            if child.name == name:
                self.delete_children(child)
                joint_to_delete = child
                break
            else:
                self.search_joint(child, name, inclusive)
        
        if joint_to_delete != None and inclusive == True:
            joint.children.remove(child)

    def delete_children(self, joint):
        """
            Delete children of the specified joint

            :param joint: joint which we want to delete its children
            :type joint: Joint Object
        """

        for child in joint.children:
            if not child.children:
                continue
            else:
                self.delete_children(child)
                childrens_to_delete = [child for child in joint.children]
                for child in childrens_to_delete:
                    joint.children.remove(child)
    
    def get_BVH_type(self):
        """ Get the type of the BVH file with the number of channels contained in it """

        if len(self.root.children[0].channels) == 6:
            return "TR" # 6 Channels for translation and rotation coordinates
        else:
            return "R" # 3 Channels for rotation coordinates

    def get_informations(self):
        """ Retrieve informations about the parsed bvh file and print them """

        joints = [joint for _, joint in enumerate(self.joints) if not joint.endswith("_end")]
        print("[+] - BVH Information:")
        print("Number of joints: ", len(joints))
        print("Number of frames: ", self.nb_frames)