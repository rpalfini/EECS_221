#!/usr/bin/env python3.9

import numpy as np
from matplotlib import pyplot as plt
import tensorflow as tf
from tensorflow import keras


def main():
    loaded_model = keras.models.load_model('trained_model_MPC.h5')
    q_values = []  # List to store computed values of q
    Ts = 0.033  # Sampling time
    t = np.arange(0, 33 + Ts, Ts)  # Simulation time 33
    q = np.array([1.1, 0.8, 0])  # Initial robot pose

    for k in range(len(t) - 4):
        u = loaded_model.predict(np.array([q]))[0]
        # Robot motion simulation
        dq = np.array([u[0] * np.cos(q[2]), u[0] * np.sin(q[2]), u[1]])
        noise = 0.00  # Set to experiment with noise(e.g., 0.001)
        q = q + Ts * dq + np.random.randn(3) * noise  # Euler integration
        q_values.append(q)  # Add q to the list
        # Map orientation angle to[-pi, pi]
        q[2] = np.arctan2(np.sin(q[2]), np.cos(q[2]))
        # Extracting the first and second elements from each vector
        x = [vector[0] for vector in q_values]
        y = [vector[1] for vector in q_values]
    # Plotting x and y
    plt.plot(x, y, 'o')
    plt.ioff()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('X-Y Plot')
    plt.show()


if __name__ == "__main__":
    main()
