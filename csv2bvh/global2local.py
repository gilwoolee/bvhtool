import numpy as np
import transforms3d
from util import read
from util import *

def get_transforms(translations, quaternions):
    Ts = np.array([np.eye(4,4)]*translations.shape[0])
    for i, (t, q) in enumerate(zip(translations, quaternions)):
        mat = np.eye(4,4)
        mat[:3,:3] = transforms3d.quaternions.quat2mat(q)
        mat[:3,-1] = t
        Ts[i] = mat
    return Ts

def get_local_transform(parent_Ts, Ts):
    local_Ts = np.array([np.eye(4,4)]*Ts.shape[0])
    for i, (parentT, T) in enumerate(zip(parent_Ts, Ts)):
        local_Ts[i] = np.dot(np.linalg.inv(parentT), T)
    return local_Ts

def get_parent(joint, joint_names, joints_structure):
    idx = joint_names.index(joint)
    for js in joints_structure:
        if idx in js:
            js_names = [joint_names[i] for i in js]

            return joint_names[js[js.index(idx) - 1]]

def to_quaternions(local_Ts):
    transforms = []
    for lt in local_Ts:
        transform = np.zeros((lt.shape[0], 7))
        for i, t in enumerate(lt):
            quat = transforms3d.quaternions.mat2quat(t[:3,:3]).tolist()
            trans = t[:3,-1]
            quat = quat[1:] + [quat[0]]
            transform[i, :4] = quat
            transform[i,4:] = trans
        transforms += [transform]

    return transforms

def global2local(csvfile, bvhfile):
    data, header = read(csvfile)
    animation, _, joints, joints_structure, _ = read_bvh(bvhfile)
    data = data.reshape(data.shape[0], int(data.shape[1]/7), 7)

    Ts = []
    joint_names = []
    for i in range(0, data.shape[1]):
        quaternions = data[:,i,0:4]
        quaternions = np.concatenate([quaternions[:,[-1]], quaternions[:,:3]], axis=1)
        translations = data[:,i,4:]
        Ts += [get_transforms(translations, quaternions)]
        joint_names += [header[i*7][:-3]]
    local_Ts = []

    for joint in joints[1:]:    
        if joint.startswith("End"):
            continue
        print (joint)
        parent_joint = joint_names.index(get_parent(joint, joints, joints_structure))
        local_Ts += [get_local_transform(parent_Ts = Ts[parent_joint], Ts = Ts[joint_names.index(joint)])]
    transform = to_quaternions(local_Ts)
    
    local_transforms = np.concatenate(transform, axis=1)
    transforms = np.concatenate([data[:,1], local_transforms], axis=1)

    joints = [j for j in joints if not j.startswith("End")]
    header = create_header(joints)
    for j in joints: 
        header += []

    np.savetxt(csvfile[:-4]+"_local.csv", transforms, fmt='%10.5f', delimiter=",", header=",".join(header), comments="")

    
if __name__ == "__main__":
    global2local("../test_shugao_global.csv", "../skeleton_simple.bvh")