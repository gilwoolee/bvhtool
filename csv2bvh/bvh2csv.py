import numpy as np
import transforms3d
from BVHToolkit import BVHAnimationReader, Animation, Pose, Bone
from util import *

def extract_motion(motion):
    num_frames = int(motion[1].split(" ")[-1])
    frame_rate = float(motion[2].split(" ")[-1])

    motion = motion[3:]
    all_data = []
    for line in motion:
        data = [float(x) for x in line.split(" ")]
        all_data += [data]

    all_data = np.array(all_data)
    assert(num_frames == all_data.shape[0])
    timestamp = np.arange(0, frame_rate*num_frames, frame_rate)
    return timestamp, all_data

def bvh2csv(bvhfile):
    animation, joints, joints_structure. _ = read_bvh(bvhfile)

    all_transforms = []
    all_rotations = []

    num_frames = len(animation.frames)
    for frame in range(num_frames):
        print ("Frame: {}".format(frame))
        pose = animation.get_pose(frame)
        transforms = []
        rotations = []
        for local, node in zip(pose.matrixes_local, pose.bone.node_list):
            mat = np.array(local).transpose()
            rotation = mat[:3,:3]
            translation = mat[:3,-1]
            if node.isRoot():
                translation = mat[:3, -1]
            else:
                translation = np.array(node.offset)
            quat = transforms3d.quaternions.mat2quat(rotation).tolist()
            quat = quat[[1,2,3,0]]
            rotations += [quat]
            transforms += [np.concatenate([quat, translation])]

        all_transforms += [np.hstack(transforms)]
        all_rotations += [np.array(rotations)]

    all_transforms = np.array(all_transforms)
    all_rotations = np.array(all_rotations)
    header = ["Timestamp"] + create_header(joints)

    lines = open(bvhfile, 'r').readlines()
    for i, l in enumerate(lines):
        if "frame time" in l.lower():
            frame_rate = float(l.split(" ")[-1].split("\t")[-1].strip())
            break
    timestamp = np.arange(0, frame_rate*num_frames, frame_rate).reshape(-1,1)
    data = np.concatenate([timestamp, all_transforms], axis=1)

    csvfilename = bvhfile[:-4] + ".csv"
    npzfilename = bvhfile[:-4] + ".npz"
    np.savetxt(csvfilename, data, fmt='%10.5f', delimiter=",", header=",".join(header), comments="")
    np.savez(npzfilename, data=data, joints_structure=joints_structure, joints=joints)

    print ("Wrote {}".format(csvfilename))
    print ("Wrote {}".format(npzfilename))



if __name__ == "__main__":
    bvh2csv("../skeleton.bvh")
