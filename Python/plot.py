import numpy as np
import matplotlib.pyplot as plt
import sys
from scipy.linalg import expm,logm,block_diag
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R


bsp_path = sys.argv[1]
gt_path = sys.argv[2]
bsp_data = np.loadtxt(bsp_path, delimiter=' ', dtype = np.float)
gt_data = np.loadtxt(gt_path, delimiter=' ', dtype = np.float)


def inv_so3(C):
    return np.array([C[2,1],C[0,2],C[1,0]])

def batch_inv_so3(C):
    res = []
    for i in range(C.shape[0]):
        res.append(inv_so3(logm(C[i,:,:].T)))
    return np.array(res)


def batch_quat2rot(Qt):
    res = []
    for i in range(Qt.shape[0]):
        r = R.from_quat(Qt[i,:])
        #print(Qt[i,:])
        temp = r.as_matrix()
        res.append(temp)
    return np.array(res)

# C_op_batch_gt = batch_quat2rot(gt_data[10:2480,5:])
# print(C_op_batch_gt)
# plt.figure()
# plt.title('plot')
# plt.plot(data[:,0], data[:, 1], 'r')
# plt.plot(data[:,0], data[:, 2], 'g')
# plt.plot(data[:,0], data[:, 3], 'b')

C_op_batch = bsp_data[:,[1,2,3,5,6,7,9,10,11]].reshape(2470,3,3)
quat_gt = gt_data[10:2480,5:]

phi_gt = batch_inv_so3(batch_quat2rot(quat_gt))
print(phi_gt.shape,bsp_data.shape)

phi_batch = batch_inv_so3(C_op_batch)
#print(phi_batch)
plt.figure()
plt.title('plot')
# plt.plot(bsp_data[600:700:10,0], phi_gt[600:700:10,0],'r.')
# plt.plot(bsp_data[600:700,0], phi_batch[600:700, 0], 'r')
plt.plot(bsp_data[:,0], phi_batch[:, 0], 'r')
plt.plot(bsp_data[::100,0], phi_gt[::100, 0], 'r.')
plt.plot(bsp_data[:,0], phi_batch[:, 1], 'g')
plt.plot(bsp_data[::100,0], phi_gt[::100, 1], 'g.')
plt.plot(bsp_data[:,0], phi_batch[:, 2], 'b')
plt.plot(bsp_data[::100,0], phi_gt[::100, 2], 'b.')
plt.show()

