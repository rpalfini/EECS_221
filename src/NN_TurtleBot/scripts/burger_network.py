#!/usr/bin/env python3.9

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import re
from scipy.spatial.transform import Rotation as R
import bisect
from datetime import datetime
import tensorflow as tf
from tensorflow import keras as K
import plot_utils
import nn_utils
import pickle


def import_data(input_fpath, ref_fpath, output_fpath):
    # import and parse the output data, the cmd vel
    output_df = nn_utils.import_data_file(
        output_fpath, 'time', '.linear.x', '.angular.z')
    # output_data = process_cmd_vel_data(output_df)
    output_data = output_df.to_numpy()

    # import and parse the model_states data
    model_df = nn_utils.import_data_file(input_fpath, 'time', '.name', '.pose')
    model_data = parse_model_states_df(model_df, 'turtlebot3_burger')

    # downsample the model_states data to match the cmd_vel
    model_data = downsample_model_states(output_data[:, 0], model_data)

    # import and parse the reference point data
    move_base_df = nn_utils.import_data_file(
        ref_fpath, '.goal.target_pose.pose.position.x', '.goal.target_pose.pose.position.y')
    move_base_data = np.array(move_base_df)
    move_base_data = move_base_data.astype(float)

    input_data = combine_model_goal_data(model_data, move_base_data)

    for_output = output_data[:, 1:]
    for_output = for_output.astype(float)

    return input_data, for_output


def combine_model_goal_data(model_data, move_base_data):
    row_num = 0
    first_index = 0
    num_data = model_data.shape[0]
    ref_data = np.zeros((num_data, 2))
    for row_num, row in enumerate(move_base_data):
        last_idx = find_matching_idx(model_data, move_base_data, row_num)
        ref_data[first_index:last_idx+1] = move_base_data[row_num, :]
        first_index = last_idx+1
        # move_base_rep = np.tile(move_base_data[row_num, :], (last_idx+1, 1))
        # ref_data = np.vstack((ref_data, move_base_rep))

    print(model_data.shape)
    print(ref_data.shape)

    input_data = np.column_stack((model_data, ref_data))
    return input_data


def find_matching_idx(model_data, move_base_data, row_num):
    not_found = True
    amount = 0.01
    tolerance = 0.05
    while not_found:
        indices = np.where(np.isclose(model_data[:, 0], move_base_data[row_num, 0], atol=tolerance) & np.isclose(
            model_data[:, 1], move_base_data[row_num, 1], atol=tolerance))[0]
        if len(indices) == 0:
            tolerance += amount
        else:
            first_idx = indices[0]
            last_idx = indices[-1]
            index_diff = last_idx - first_idx
            if index_diff <= 0:
                index_diff = 1
            not_found = False

    return last_idx


def downsample_model_states(ref_timestamps, model_data):
    ref_data_ts = [parse_and_round(ts) for ts in ref_timestamps]
    new_model_data = np.zeros((ref_timestamps.shape[0], 3))
    model_data_ts = [parse_and_round(ts) for ts in model_data[:, 0]]
    for ii, timestamp in enumerate(ref_timestamps):
        idx = search_timestamps(model_data_ts, parse_and_round(timestamp))
        new_model_data[ii, :] = model_data[idx, 1:]
        # new_model_data.append(model_data[idx, :])

    # new_model_data = np.array(new_model_data)
    return new_model_data


def search_timestamps(timestamps, target_timestamp):
    index = bisect.bisect_left(timestamps, target_timestamp)
    # and timestamps[index] == target_timestamp: # not checking that timestamp is the same as there isn't always a measurement and I am running out of time :)
    if index < len(timestamps):
        # Timestamp found at index
        return index
    else:
        # Timestamp not found
        return None


def parse_and_round(timestamp_string):
    dt_time = parse_timestamp(timestamp_string)
    rounded_dt = round_datetime_to_hundredth(dt_time)
    return rounded_dt


def round_datetime_to_hundredth(dt):
    # Round datetime to the hundredth of a second
    rounded_microseconds = round(dt.microsecond, -4)
    if rounded_microseconds == 1000000:
        rounded_microseconds = 0
    rounded_dt = dt.replace(microsecond=rounded_microseconds)
    return rounded_dt


def parse_timestamp(timestamp):
    format_string = '%Y/%m/%d/%H:%M:%S.%f'
    return datetime.strptime(timestamp, format_string)


def parse_model_states_df(input_df, model_name):
    model_idx = get_model_index(input_df, model_name)

    positions = []
    orientations = []

    for row in input_df['.pose']:
        # Extract position data
        position_matches = re.findall(
            r'x: ([0-9e.-]+)\n\s+y: ([0-9e.-]+)\n\s+z: ([0-9e.-]+)\n(?=orientation:)', row)
        x, y, z = position_matches[model_idx]

        # Extract orientation data
        orientation_matches = re.findall(
            r'x: ([0-9e.-]+)\n\s+y: ([0-9e.-]+)\n\s+z: ([0-9e.-]+)\n\s+w: ([0-9e.-]+)', row)
        qx, qy, qz, qw = orientation_matches[model_idx]
        r = R.from_quat([qx, qy, qz, qw])
        angles = r.as_euler('xyz')

        positions.append((x, y))  # robot is always on ground so we dont need z
        orientations.append(angles[2])  # only care about the yaw angle

    positions = np.array(positions)
    orientations = np.array(orientations)

    time_data = input_df['time'].to_numpy()

    pos_ori_data = np.column_stack((positions, orientations))
    pos_ori_data = pos_ori_data.astype(float)
    input_data = np.column_stack((time_data, pos_ori_data))
    return input_data


def get_model_index(input_df, model_name):
    model_list_string = input_df.loc[0, '.name']
    model_names = eval(model_list_string)
    if model_name in model_names:
        return model_names.index(model_name)
    else:
        return None


def main():
    input_fpath = '/home/rpalfini/catkin_ws/Recorded_Robot-gazebo-model_states.csv'
    ref_fpath = '/home/rpalfini/catkin_ws/Recorded_Robot-move_base-goal.csv'
    output_fpath = '/home/rpalfini/catkin_ws/Recorded_Robot-cmd_vel.csv'
    # first three columns of input_data is the model state x,y,theta and the last two columns are the goal_state
    input_data, output_data = import_data(input_fpath, ref_fpath, output_fpath)

    train_ratio = 0.8
    train_samples = int(train_ratio * len(input_data))
    x_train = input_data[:train_samples]
    x_test = input_data[train_samples:]
    y_train = output_data[:train_samples]
    y_test = output_data[train_samples:]

    num_layers = 20
    input_neurons = 5
    hidden_neurons = 128
    output_neurons = 2
    hidden_act = 'relu'
    output_act = 'linear'

    epochs = 20
    batch_size = 32
    opti = 'adam'
    loss = 'mse'
    metrics = ['accuracy']

    # num_layers = 20
    # neurons = 128

    model = nn_utils.make_dense_NN(
        num_layers, input_neurons, hidden_neurons, output_neurons, hidden_act, output_act)
    # model.add(K.layers.Dense(neurons, activation='relu', input_shape=(5,)))
    # for ii in range(num_layers):
    #     model.add(K.layers.Dense(neurons, activation='relu'))
    #     # default activation is linear which is needed for regression
    # model.add(K.layers.Dense(2))


    model.compile(optimizer=opti, loss=loss, metrics=metrics)

    result = model.fit(x_train, y_train, epochs=epochs,
                       batch_size=batch_size, validation_data=(x_test, y_test))
    mname = 'model_e%d_b%d_l%d_n%d' % (epochs,batch_size,num_layers,hidden_neurons)
    model.save(mname+'.h5')
    # model.save('trained_model_gazebo.h5')

    plot_utils.plot_hist(result.history)
    plt.savefig(mname+'.png')
    plt.show()



if __name__ == "__main__":
    main()
