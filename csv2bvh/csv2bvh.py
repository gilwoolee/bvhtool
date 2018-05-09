import numpy as np
import transforms3d
from util import read
from BVHToolkit import BVHAnimationReader, Animation, Pose, Bone

def csv2bvh(csvfile, skeleton_bvh):

    data, header = read(csvfile)
    new_data = []
    for i in range(0, data.shape[1], 7):
        quaternions = data[:,i:i+4]
        quaternions = quaternions[:,[3,0,1,2]]
        eulers = []
        for quat in quaternions:
            euler = np.rad2deg(transforms3d.euler.quat2euler(quat, axes='rzxy'))
            eulers += [euler]

        eulers = np.array(eulers)
        new_data += [np.concatenate([data[:,i+4:i+7], eulers], axis=1)]

    data = np.concatenate(new_data, axis=1)

    frame_time = 1.0/90.0

    print ("Load skeleton_bvh")
    bvh_file = open(skeleton_bvh, "r").readlines()
    skeleton_def = []
    joint_ordering = ["body_world"]
    endsite_offsets = dict()
    for i in range(len(bvh_file)):
        if bvh_file[i].startswith("MOTION"):
            break
        if bvh_file[i].lstrip().startswith("JOINT"):
            joint_ordering += [bvh_file[i].lstrip().split(" ")[1].replace("\r","").replace("\n","")]
        if "End Site" in bvh_file[i]:
            offset = [float(x) for x in bvh_file[i+2].lstrip().split(" ")[1:]]
            endsite_offsets[joint_ordering[-1]] = offset
            joint_ordering += ["End Site"]

    skeleton_def = bvh_file[0:i]

    csv_joints = []
    for i in range(1, len(header), 7):
        csv_joints += [header[i][:-3]]

    new_data = []

    for i, joint in enumerate(joint_ordering):
        if joint.startswith("End Site"):
            pass
        else:
            idx = csv_joints.index(joint)
            new_data += [data[:,idx*6:idx*6+6]]

    data = np.around(np.concatenate(new_data, axis=1), 3)

    frame_count = data.shape[0]
    motion = ["MOTION"]
    motion += ["Frames: {}".format(frame_count)]
    motion += ["Frame Time: " + str(frame_time)]
    for i in range(frame_count):
        d = data[i,:].tolist()
        d = ["{:.1f}".format(x) for x in d]
        d = ' '.join(d)
        motion += [d]

    outfilename = csvfile[:-4] + ".bvh"
    with open(outfilename, "w") as f:
        f.write("".join(skeleton_def))
        f.write("\n".join(motion))
    print ("Wrote {}".format(outfilename))


def subset_csv2bvh(csvfile, skeleton_bvh, quaternion_only=True):
    """
    Parses csv with subset of skeleton definition
    """
    print ("Load skeleton_bvh")
    bvh_file = open(skeleton_bvh, "r").readlines()
    skeleton_def = []
    joint_ordering = ["body_world"]
    endsite_offsets = dict()
    for i in range(len(bvh_file)):
        if bvh_file[i].startswith("MOTION"):
            break
        if bvh_file[i].lstrip().startswith("JOINT"):
            joint_ordering += [bvh_file[i].lstrip().split(" ")[1].replace("\r","").replace("\n","")]
        if "End Site" in bvh_file[i]:
            offset = [float(x) for x in bvh_file[i+2].lstrip().split(" ")[1:]]
            endsite_offsets[joint_ordering[-1]] = offset
            joint_ordering += ["End Site"]

    skeleton_def = bvh_file[0:i]
    # default_skeleton = np.array([float(x) for x in bvh_file[i+3].replace("\r\n","").split(" ")])
    # import IPython; IPython.embed()

    # Load csv
    data, header = read(csvfile)
    if quaternion_only:
        dims = 4
    else:
        dims = 7

    csv_joints = []
    for i in range(1, len(header), dims):
        csv_joints += [header[i][:-3]]

    frame_time = 1.0/90.0

    # Get default pose
    animation = Animation.from_bvh(skeleton_bvh)
    pose = animation.get_pose(0)


    new_data = []
    for joint_idx, i in enumerate(range(0, data.shape[1], dims)):
        quaternions = data[:,i:i+4]
        quaternions = quaternions[:,[3,0,1,2]]
        eulers = []
        for quat in quaternions:
            euler = np.rad2deg(transforms3d.euler.quat2euler(quat, axes='rzxy'))
            eulers += [euler]

        eulers = np.array(eulers)

        if not quaternion_only:
            new_data += [np.concatenate([data[:,i+4:i+7], eulers], axis=1)]
        else:
            joint_name = csv_joints[joint_idx]

            if joint_name in joint_ordering:
                joint_idx_bvh = joint_ordering.index(joint_name)
                translation = np.array(pose.matrixes_local[joint_idx_bvh]).transpose()[:3,-1]
            else:
                if "null" in joint_name and csv_joints[joint_idx-1] == joint_name[:-5] and csv_joints[joint_idx-1] in joint_ordering:
                    joint_idx_bvh = joint_ordering.index(csv_joints[joint_idx-1]) + 1
                    if pose.bone.node_list[joint_idx_bvh].name == "End Site":
                        translation = np.array(pose.matrixes_local[joint_idx_bvh]).transpose()[:3,-1]
                    else:
                        translation = np.zeros(3)
                else:
                    translation = np.zeros(3)
            translation = np.tile(translation, (eulers.shape[0], 1))
            new_data += [np.concatenate([translation, eulers], axis=1)]

    data = np.concatenate(new_data, axis=1)


    new_data = []

    for i, joint in enumerate(joint_ordering):
        if joint.startswith("End Site"):
            pass
        else:
            if joint in csv_joints:
                idx = csv_joints.index(joint)
                new_data += [data[:,idx*6:idx*6+6]]
            else:
                print ("Read {} from skeleton bvh".format(joint))
                transform = np.array(pose.matrixes_local[i]).transpose()
                translation = transform[:3, -1]
                euler = np.rad2deg(transforms3d.euler.mat2euler(transform[:3,:3], axes='rzxy'))
                transform = np.concatenate([translation, euler])
                transform = np.tile(transform, (data.shape[0], 1))
                new_data += [transform]

    data = np.around(np.concatenate(new_data, axis=1), 3)

    print ("Joints should be in the follwing order: ", joint_ordering)
    print ("Copy the skeleton definition from skeleton_bvh")
    print (data.shape)

    frame_count = data.shape[0]
    motion = ["MOTION"]
    motion += ["Frames: {}".format(frame_count)]
    motion += ["Frame Time: " + str(frame_time)]
    for i in range(frame_count):
        d = data[i,:].tolist()
        d = ["{:.1f}".format(x) for x in d]
        d = ' '.join(d)
        motion += [d]

    outfilename = csvfile[:-4] + ".bvh"
    with open(outfilename, "w") as f:
        f.write("".join(skeleton_def))
        f.write("\n".join(motion))
    print ("Wrote {}".format(outfilename))


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description='Convert CSV to BVH')
    parser.add_argument('--rootdir', '-r', type=str, default="/home/gilwoo/Oculus_Workspace/KalmanOutput/",
                        help='CSV file to parse')
    parser.add_argument('--skeleton', type=str, default="skeleton.bvh",
                        help='Skeleton definition')

    args = parser.parse_args()

    import glob
    files = glob.glob(args.rootdir + "*local*.csv")

    for f in files:
        print ("File: {}".format(f))
        try:
            subset_csv2bvh(f, args.skeleton, True)
        except:
            print ("Failed to parse {}".format(f))

