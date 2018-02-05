from scipy import io
import numpy as np
from helper import vec2quat,quatMulti,acc2rp,rpy2rot,vecNormorlize,quat2matrix



#
# data=io.loadmat('./vicon/viconRot1.mat')
# rots, time= data['rots'], data['ts']
# rots*=scale
def caldQ(w,t_i):

    dAngle = np.linalg.norm(w) * t_i
    dAxis = w / np.linalg.norm(w)
    return np.append(np.cos(dAngle / 2), np.multiply(dAxis, np.sin(dAngle / 2)))


def processA(A):
    return rpy2rot(acc2rp(A)) # return 3*3*n

def processW(W,ts):
    _,n_data=W.shape
    rots_W=np.zeros(3,3,n_data)
    ti= np.append(0, ts[1:] - ts[:-1])
    dq=caldQ(W,ti)
    q0=np.array([1,0,0,0])
    for i in range(n_data):
        rots_W[:,:,i]=quat2matrix(vecNormorlize(quatMulti(q0,dq[i])))

    return rots_W








def impData():
    sensitivity_A = 0.33  # unit:mv/g
    scale_A = 3.3 / 1023 / sensitivity_A
    Az_addbias =1/scale_A

    sensitivity_W = 3.33   # unit:mv/deg/s
    scale_W = 3300 / 1023 / (sensitivity_W*180/np.pi)


    data = io.loadmat('./imu/imuRaw1.mat')
    raw_vals, ts = data['vals'], data['ts']
    A,W=raw_vals[:3,:],raw_vals[3:,:]
    A_bias=np.mean(A[:,:100],axis=1)
    A_bias[-1]=A_bias+Az_addbias
    A=(A-A_bias)*scale_A
    A[0:1]*=-1






    W_bias = np.mean(W[:, :100], axis=1)
    W = (W - W_bias) * scale_W
    W[[0,1,2]]=W[[1,2,0]]

    return A,W,ts


def gtData():


    data=io.loadmat('./vicon/viconRot1.mat')
    rots, ts_gt= data['rots'], data['ts']


    return rots,ts_gt





def ukf(rots_A,rots_W,ts):
    #init pare
    P = np.eye(6)#State Cov
    Q=np.eye(6)
    Q[:3, :3]+=np.ones(3)
    Q[3:, 3:] += np.ones(3)
    Q*=5e-8

    X=np.zeros([12,7])




    n=6
    x=np.array([1,0,0,0,0,0,0])
    g = np.array([0, 0, 0, 1])

    t_interval=np.append(0,ts[1:] - ts[:-1])

    _, n_measure=rots.shape

    for i in range(n_measure):

        #calulate dq
        dq=caldQ(x[4:],t_interval[i])
        # dq[np.isnan(dq)]=0

        #extract sigma points
        S=np.linalg.cholesky(P+Q) #S=6*6
        W=np.sqrt(2*n)*S
        W=np.append(W,-W,axis=1)#W=6*12
            #to Quater
        X[:,:4]=quatMulti(x,vec2quat(W))
        X[:,4:]=W[:,3:]
        X=X.T

        #Tansformation of sigma pts X












        P=np.eye(6)
        S=np.linalg.cholesky(P)


    return rots_filtered

if __name__=='__main__':
    A,W,ts_imu=impData()
    rots_A=processA(A)
    rots_W=processW(W,ts_imu)

    rots,ts_gt=gtData()

    rots_ukf=ukf(rots_A,rots_W,ts_imu)







