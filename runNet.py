import numpy as np
import pickle
from collections import defaultdict
from scipy.io import loadmat
from scipy.io import savemat
import tensorflow as tf
import keras.backend as K

### accCost 0: none ### 1: acc cost as angle vector ### 2: acc cost as MAE
acc = 2


def get_data(Xi, Yi, N_TIME_STEPS = 100, STEP = 1): #, LEN, N_TIME_STEPS, STEP
    LEN = np.size(Xi[:,0])
    data = defaultdict(lambda: [])
    for j in range(0, LEN - N_TIME_STEPS, STEP):
          inp = Xi[j: j + N_TIME_STEPS]
          out = Yi[j + N_TIME_STEPS]
          data['input'].append(inp) # input
          data['output'].append(out) # output
    for key in data.keys():
        data[key] = np.array(data[key])
    return data


def mae_acc_fn(y_true, y_pred):
    abs_difference = tf.abs(y_true[:,13:] - y_pred[:,13:])
    return tf.reduce_mean(abs_difference, axis=-1)


def mean_angle_btw_vectors(v1, v2):
    dot_product = tf.reduce_sum(v1*v2, axis=-1)
    cos_a = dot_product / (tf.norm(v1, axis=-1) * tf.norm(v2, axis=-1))
    eps = 1e-8
    cos_a = tf.clip_by_value(cos_a, -1 + eps, 1 - eps)
    angle_dist = tf.math.acos(cos_a) / np.pi * 180.0
    return tf.reduce_mean(angle_dist)


# Load vector NyQaut with Yaw rate test data from mocap dataset
D = loadmat("./ExtData/nYQTestData.mat")
Xt = D['X_t']
Yt = D['Y_t']

if acc >= 1:
    Xt = D['X_t'][:,0:20] # last 5 features are yaw rates
    Yt = D['Y_t']

    # laod acc data
    D = loadmat("./ExtData/accTestData.mat")
    Xat = D['X_t']
    Yat = D['Y_t']

    # stack acc data with NyQuat data
    Xt = np.hstack((Xt, Xat))
    Yt = np.hstack((Yt, Xat))


# process data
data = get_data(N_TIME_STEPS=100, STEP=1, Xi=Xt, Yi=Yt)

# load trained model weights
if acc==0:
        # model w/o acc
        print("Running net without acceleration ...")
        model = tf.keras.models.load_model('./results/model_nyQuat_w100_s1',
                                           custom_objects={'mean_angle_btw_vectors': mean_angle_btw_vectors})

elif acc==1:
        # model w acc with vector angle cost func
        print("Running net with acceleration and vector angle cost function...")
        model = tf.keras.models.load_model('./results/model_nyQuatAccPlus_w100_s1',
                                           custom_objects={'mean_angle_btw_vectors': mean_angle_btw_vectors})

else:
        # model w acc with MAE cost func
        print("Running net with acceleration and MAE around each axis cost function...")
        model = tf.keras.models.load_model('./results/model_nyQuatAccPlusCost_w100_s1',
                                           custom_objects={'mean_angle_btw_vectors': mean_angle_btw_vectors,
                                                           'mae_acc_fn': mae_acc_fn})


# run model
results = model.predict(x = data['input'], batch_size=64, verbose=1)

# save vector predictions
savemat('./results/myTestPredict.mat', {'Predict': results})

# push data to Matlab


if acc==0:
        resultsE = model.evaluate(x = data['input'], y=data['output'].reshape(-1, 13, 3), batch_size=64, verbose=1)
else:
        resultsE = model.evaluate(x=data['input'], y=data['output'].reshape(-1, 18, 3), batch_size=64, verbose=1)

