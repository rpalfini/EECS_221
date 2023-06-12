#!/usr/bin/env python3.9

import numpy as np
import tensorflow as tf
from tensorflow import keras as K
import plot_utils
import pandas as pd


def make_dense_NN(num_layers, input_neurons, hidden_neurons, output_neurons, hidden_act, output_act):
    model = K.Sequential()
    model.add(K.layers.Dense(hidden_neurons,
              activation=hidden_act, input_shape=(input_neurons,)))
    for ii in range(num_layers-1):
        model.add(K.layers.Dense(hidden_neurons, activation=hidden_act))
    model.add(K.layers.Dense(output_neurons, activation=output_act))
    return model


def import_data_file(file_path, *headers):
    data_frame = pd.read_csv(file_path)
    selected_data = data_frame[headers]
    return selected_data
