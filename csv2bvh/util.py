import numpy as np
import transforms3d
from BVHToolkit import BVHAnimationReader, Animation, Pose, Bone

def read(csvfile):
    print ("Load csv")
    print ("Assumes 1st row is the header")
    print ("Data must contain (rotation, translation), with rotation in quaternion format in (x,y,z,w)")
    data = np.genfromtxt(csvfile, skip_header=1, delimiter=",")
    header = open(csvfile, "r").readline().replace("\n","").replace("\r","").split(",")
    if "time" in header[0].lower():
        start_idx = 1
        header = header[1:]
    else:
        start_idx = 0
    if (len(data.shape) == 1):
        data = data.reshape(1,-1)
    if np.isnan(data[0,-1]):
        data = data[:,start_idx:-1]
    else:
        data = data[:,start_idx:]

    return data, header

def get_joint_structure(joints, node, structure):
    import copy
    if node.name.startswith("End"):
        idx = structure[-1] + 1
        structure += [idx]
        return structure
    else:
        structure += [joints.index(node.name)]

    children = node.children
    root_structure = []

    for child in children:
        st = get_joint_structure(joints, child, copy.copy(structure))
        root_structure.append(st)

    flattend_root_structure = []
    for elt in root_structure:

        if isinstance(elt[0], list):
            for j in elt:
                flattend_root_structure.append(j)
        else:
            flattend_root_structure.append(elt)

    return flattend_root_structure

def read_bvh(bvhfile):
    print ("Load bvh")
    animation = Animation.from_bvh(bvhfile)
    num_frames = len(animation.frames)
    pose = animation.get_pose(0)
    joints = [node.name for node in pose.bone.node_list]

    joints_structure = get_joint_structure(joints, pose.bone.node_list[0], [])
    return animation, joints, joints_structure


def create_header(joint_names):
    postfix = ["_rx","_ry","_rz","_rw","_tx","_ty","_tz"]
    header = []
    for j in joint_names:
        names = [j+p for p in postfix]
        header += names
    return header