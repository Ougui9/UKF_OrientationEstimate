from scipy import io
import numpy as np
from helper import vec2quat,quatMulti,acc2rp,rpy2rot,\
    vecNormorlize,quat2matrix,quaternion_conjugate,quat2vec
from utils import averageQuaternions


#
# data=io.loadmat('./vicon/viconRot1.mat')
# rots, time= data['rots'], data['ts']
# rots*=scale
def caldQ(w,t_i):
    '''
    :param w: 3,n,
    :param t_i: 1,n
    :return: dq: 4,1
    '''
    w_norm=np.linalg.norm(w,axis=0)
    dAngle = w_norm * t_i
    dAxis = w / w_norm
    dq=np.append(np.cos(dAngle / 2), np.multiply(dAxis, np.sin(dAngle / 2)),axis=0)
    dq[np.isnan(dq)]=0
    return dq


def processA(A):
    r,p,y=acc2rp(A)
    return rpy2rot(r,p,y) # return 3*3*n

def processW(W,ts):
    _,n_data=W.shape
    rots_W=np.zeros([3,3,n_data])
    ti= np.append([[0]], ts[:,1:] - ts[:,:-1],axis=1)
    dq=caldQ(W,ti)
    q0=np.array([1,0,0,0])
    for i in range(n_data):
        rots_W[:,:,i]=quat2matrix(vecNormorlize(quatMulti(q0.reshape(4,-1),dq[:,i].reshape(4,-1)))).reshape(3,3)

    return rots_W








def impData():
    sensitivity_A = 0.33  # unit:mv/g
    scale_A = 3.3 / 1023 / sensitivity_A
    Az_addbias =-1/scale_A

    sensitivity_W = 3.33   # unit:mv/deg/s
    scale_W = 3300 / 1023 / (sensitivity_W*180/np.pi)


    data = io.loadmat('./imu/imuRaw1.mat')
    raw_vals, ts = data['vals'], data['ts']
    A,W=raw_vals[:3,:],raw_vals[3:,:]
    A_bias=np.mean(A[:,:100],axis=1)
    A_bias[-1]=A_bias[-1]+Az_addbias
    A=(A-A_bias.reshape(3,1))*scale_A
    A[0:1]*=-1






    W_bias = np.mean(W[:, :100], axis=1)
    W = (W - W_bias.reshape(3,1)) * scale_W
    W[[0,1,2]]=W[[1,2,0]]

    return A,W,ts


def gtData():


    data=io.loadmat('./vicon/viconRot1.mat')
    rots, ts_gt= data['rots'], data['ts']


    return rots,ts_gt





def ukf(rots_A,rots_W,ts):
    #init para
    P = np.eye(6)#State Cov
    Q=np.eye(6)
    Q[:3, :3]+=np.ones(3)
    Q[3:, 3:] += np.ones(3)
    Q*=5e-8

    X=np.zeros([12,7])
    Y=X.copy()





    n=6
    x=np.array([1,0,0,0,0,0,0])
    xk = x.copy()
    g = np.array([0, 0, 0, 1])

    t_interval=np.append([[0]], ts[:,1:] - ts[:,:-1],axis=1)

    _, _,n_measure=rots.shape

    for i in range(n_measure):

        #calulate dq
        w=x[4:]
        dq=caldQ(w.reshape(3,1),t_interval[0,i].reshape(1,-1))


        #extract sigma points
        S=np.linalg.cholesky(P+Q) #S:(6, 6)
        W=np.sqrt(2*n)*S#W: (6, 6)
        W=np.append(W,-W,axis=1)#W=6*12
            #to Quater
        X[:,:4]=quatMulti(x[:4].reshape(4,-1),vec2quat(W[:3,:])).T
        X[:,4:]=W[3:,:].T  #X: (12, 7)
        print(1)

        #Tansformation of sigma pts X
        Y[:,:4] = quatMulti(X[:, 0: 4].T, dq).T
        Y[:,4:] =X[:,4:]+w

        q2k=averageQuaternions(Y[:,:4])
        print(1)
        xk[:4],xk[4:]=q2k,np.mean(Y[:,4:],axis=0)

        Wp[:, 0: 3] = quat2vec(quatMulti(Y[:, 0: 4], quaternion_conjugate(xk[0: 4])))













        #
        #
        # P=np.eye(6)
        # S=np.linalg.cholesky(P)


    return rots_filtered

if __name__=='__main__':
    A,W,ts_imu=impData()
    rots_A=processA(A)
    rots_W=processW(W,ts_imu)

    rots,ts_gt=gtData()

    rots_ukf=ukf(rots_A,rots_W,ts_imu)







