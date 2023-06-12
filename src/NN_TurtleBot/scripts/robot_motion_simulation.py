#!/usr/bin/env python3.9

import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow import keras as K
import plot_utils


def main():

    # options
    plot_data = True
    train_model = True  # not implemented yet, keep as True
    shuffle_data = True

    # NN variables
    input_data = []
    output_data = []

    # Simulation Variables
    # sim_length = 6000
    sim_length = 30
    q_values = []  # List to store computed values of q
    u_values = []  # List to store values of u used]
    Ts = 0.033  # Sampling time
    t = np.arange(0, sim_length + Ts, Ts)  # Simulation time
    q = np.array([1.1, 0.8, 0])  # Initial robot pose
    freq = 2 * np.pi / 30
    xRef = 1.1 + 0.7 * np.sin(freq * t)
    yRef = 0.9 + 0.7 * np.sin(2 * freq * t)
    dxRef = freq * 0.7 * np.cos(freq * t)
    dyRef = 2 * freq * 0.7 * np.cos(2 * freq * t)
    ddxRef = - freq ** 2 * 0.7 * np.sin(freq * t)
    ddyRef = -4 * freq ** 2 * 0.7 * np.sin(2 * freq * t)
    # Reference trajectory
    qRef = np.array([xRef, yRef, np.arctan2(dyRef, dxRef)])
    vRef = np.sqrt(dxRef ** 2 + dyRef ** 2)
    wRef = (dxRef * ddyRef - dyRef * ddxRef) / (dxRef ** 2 + dyRef ** 2)
    uRef = np.array([vRef, wRef])  # Reference inputs

    # MPC Loop
    for k in range(len(t) - 4):
        e = np.array([[np.cos(q[2]), np.sin(q[2]), 0],
                      [-np.sin(q[2]), np.cos(q[2]), 0],
                      [0, 0, 1]]) @ (qRef[:, k] - q)  # error vector

        # wrapToPi function equivalent
        e[2] = np.arctan2(np.sin(e[2]), np.cos(e[2]))

        A0 = np.array([[1, Ts * uRef[1, k], 0],
                       [- Ts * uRef[1, k], 1, Ts * uRef[0, k]],
                       [0, 0, 1]])
        A1 = np.array([[1, Ts * uRef[1, k + 1], 0],
                       [- Ts * uRef[1, k + 1], 1, Ts * uRef[0, k + 1]],
                       [0, 0, 1]])
        A2 = np.array([[1, Ts * uRef[1, k + 2], 0],
                       [- Ts * uRef[1, k + 2], 1, Ts * uRef[0, k + 2]],
                       [0, 0, 1]])
        A3 = np.array([[1, Ts * uRef[1, k + 3], 0],
                       [- Ts * uRef[1, k + 3], 1, Ts * uRef[0, k + 3]],
                       [0, 0, 1]])
        A4 = np.array([[1, Ts * uRef[1, k + 4], 0],
                       [- Ts * uRef[1, k + 4], 1, Ts * uRef[0, k + 4]],
                       [0, 0, 1]])

        B = np.array([[Ts, 0],
                      [0, 0],
                      [0, Ts]])

        C = np.eye(3)

        Z = np.zeros((3, 2))

        Hm = np.block([[C @ A0 @ B, Z, Z, Z],
                       [C @ A0 @ A1 @ B, C @ A0 @ B, Z, Z],
                       [C @ A0 @ A1 @ A2 @ B, C @ A0 @ A1 @ B, C @ A0 @ B, Z],
                       [C @ A0 @ A1 @ A2 @ A3 @ B, C @ A0 @ A1 @ A2 @ B, C @ A0 @ A1 @ B, C @ A0 @ B]])

        Fm = np.block([[C @ A0 @ A1, C @ A0 @ A1 @ A2, C @ A0 @
                      A1 @ A2 @ A3, C @ A0 @ A1 @ A2 @ A3 @ A4]]).T

        ar = 0.65
        Ar = np.eye(3) * ar  # Reference error dynamics
        H = 0
        Fr = np.block([[np.linalg.matrix_power(Ar, H + 1), np.linalg.matrix_power(Ar, H + 2),
                        np.linalg.matrix_power(Ar, H + 3), np.linalg.matrix_power(Ar, H + 4)]]).T

        Qt = np.diag(np.tile([1, 40, 0.1], 4))
        Rt = np.diag(np.tile([0.001, 0.001], 4))

        KKgpc = np.linalg.inv(Hm.T @ Qt @ Hm + Rt) @ (Hm.T @ Qt @ (- Fm))
        KK = KKgpc[:2, :]  # Take current control gains

        v = - KK @ e
        uF = np.array([uRef[0, k] * np.cos(e[2]), uRef[1, k]])
        u = v + uF

        vMAX = 1
        wMAX = 15  # Max velocities

        if abs(u[0]) > vMAX:
            u[0] = np.sign(u[0]) * vMAX
        if abs(u[1]) > wMAX:
            u[1] = np.sign(u[1]) * wMAX

        dq = np.array([u[0] * np.cos(q[2]), u[0] * np.sin(q[2]), u[1]])
        noise = 0.00  # Set to experiment with noise(e.g., 0.001)
        q = q + Ts * dq + np.random.randn(3) * noise  # Euler integration
        q_values.append(q)  # Add q to the list
        q[2] = np.arctan2(np.sin(q[2]), np.cos(q[2]))  # angle to[-pi, pi]
        u_values.append(u)

    if plot_data:
        plot_utils.plot_traj(q_values)
        plot_utils.plot_columns_against_time(np.array(u_values), Ts)
        plot_utils.plot_columns_against_time(np.array(q_values), Ts)

    input_data = np.array(q_values)  # this should be q values
    output_data = np.array(u_values)  # this should be u values
    if shuffle_data:
        combined_data = np.concatenate((input_data, output_data), axis=1)
        np.random.shuffle(combined_data)
        input_data = combined_data[:, :3]
        output_data = combined_data[:, 3:]

    if train_model:
        train_ratio = 0.8
        train_samples = int(train_ratio * len(input_data))
        x_train = input_data[:train_samples]
        y_train = output_data[:train_samples]
        x_test = input_data[train_samples:]
        y_test = output_data[train_samples:]

        num_layers = 30
        neurons = 128
        model = K.Sequential()
        model.add(K.layers.Dense(neurons, activation='relu', input_shape=(3,)))
        for ii in range(num_layers):
            model.add(K.layers.Dense(neurons, activation='relu'))
        # default activation is linear which is needed for regression
        model.add(K.layers.Dense(2))

        # model.compile(optimizer='adam', loss='mse', run_eagerly=True)
        model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])

        result = model.fit(x_train, y_train, epochs=20,
                           batch_size=32, validation_data=(x_test, y_test))
        model.save('trained_model_MPC.h5')

    plot_utils.plot_hist(result.history, logscale=0)
    plt.show()


if __name__ == "__main__":
    main()
