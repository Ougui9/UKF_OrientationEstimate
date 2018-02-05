from scipy import io
import numpy as np



# def deg2rad(deg):
#     return deg/180*np.pi

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

    W_bias = np.mean(W[:, :100], axis=1)
    W = (W - W_bias) * scale_W



    return A,W


def gtData():


    data=io.loadmat('./vicon/viconRot1.mat')
    rots, ts_gt= data['rots'], data['ts']


    return rots,ts_gt

def Aprocess(A_rawData):
