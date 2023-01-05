import torch
from scipy.io import loadmat
from scipy.io import savemat
from dynamics import PhysicsOptimizer
import numpy

if __name__ == '__main__':

    # load data from saved matlab file
    D = loadmat("./data/DataForOptz.mat")
    print(D.keys())
    pose = D['p']
    joint_velocity = D['v']
    contact = D['c']
    glb_acc = D['a']

    dynamics_optimizer = PhysicsOptimizer(debug=True)

    pose_opt, tran_opt = [], []

    for p, v, c, a in zip(pose, joint_velocity, contact, glb_acc):
        p, t = dynamics_optimizer.optimize_frame(p, v, c, a)
        pose_opt.append(p)
        tran_opt.append(t)
    pose_opt, tran_opt = numpy.stack(pose_opt), numpy.stack(tran_opt)
    savemat('./results/phyOptzedData.mat', {'pose_opt': pose_opt, 'tran_opt': tran_opt})
    #return pose_opt, tran_opt
