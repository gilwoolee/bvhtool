import numpy as np
import transforms3d
from util import read

def csv2bvh(csvfile, skeleton_bvh):

    data, header = read(csvfile)
    new_data = []
    converter = transforms3d.euler.TBZYX()
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
            # t = np.array(endsite_offsets[joint_ordering[i-1]] + [0,0,0])
            # t = np.tile(t, (data.shape[0], 1))
            # new_data += [t]
            pass
        else:
            idx = csv_joints.index(joint)

            if not idx*6 + 6 <= data.shape[1]:
                print (joint)
                import IPython; IPython.embed()
            new_data += [data[:,idx*6:idx*6+6]]

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

    csv2bvh("../test_shugao.csv", "../skeleton.bvh")
