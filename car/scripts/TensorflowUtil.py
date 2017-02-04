#!/usr/bin/env python
"""
Copyright (c) 2017, Ryan Dellana
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Ryan Dellana nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL Ryan Dellana BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


import tensorflow as tf
import numpy as np

def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)

def bias_variable(shape):
    initial = tf.constant(0.0, shape=shape)
    return tf.Variable(initial)

def conv2d(x, W, stride):
    return tf.nn.conv2d(x, W, strides=[1, stride, stride, 1], padding='SAME')

def conv_layer(x, conv=(3, 3), stride=1, n_filters=32, use_bias=False):
    W = weight_variable([conv[0], conv[1], x.get_shape()[-1].value, n_filters])
    if use_bias:
        b = bias_variable([n_filters])
        return (tf.nn.relu(conv2d(x, W, stride=stride) + b), W)
    else:
        return (tf.nn.relu(conv2d(x, W, stride=stride)), W)

def conv_layer_(x, conv=(3, 3), stride=1, n_filters=32, use_bias=False, weights=None):
    return tf.nn.relu(conv2d(x, weights, stride=stride))

def fc_layer(x, n_neurons, activation=tf.tanh, use_bias=True, dropout=False):
    W = weight_variable([x.get_shape()[-1].value, n_neurons])
    h, b = None, None
    if use_bias:
        b = bias_variable([n_neurons])
        h = activation(tf.matmul(x, W) + b)
    else:
        h = activation(tf.matmul(x, W))
    if dropout:
        keep_prob = tf.placeholder(tf.float32)
        h_drop = tf.nn.dropout(h, keep_prob)
        return (h_drop, W, b, keep_prob)
    else:
        return (h, W, b, None)

def fc_layer_(x, n_neurons, activation=tf.tanh, use_bias=True, dropout=False, weights=None, bias=None):
    h = None
    if use_bias and bias != None:
        h = activation(tf.matmul(x, weights) + bias)
    else:
        h = activation(tf.matmul(x, weights))
    return h

# h_identity_in, W_in = identity_in(x)
def identity_in(x):
    shp = x.get_shape()
    W = tf.ones([shp[1].value, shp[2].value, shp[3].value])
    return tf.mul(x, W), W

def flattened(x):
    product = 1
    for d in x.get_shape():
        if d.value is not None:
            product *= d.value
    return tf.reshape(x, [-1, product])

# Define negative log-likelihood and gradient
def normal_log(X, mu=np.float32(1), sigma=np.float32(1), left=-np.inf, right=np.inf):
    val = -tf.log(tf.constant(np.sqrt(2 * np.pi), dtype=tf.float32) * sigma) - \
           tf.pow(X - mu, 2) / (tf.constant(2, dtype=tf.float32) * tf.pow(sigma, 2))
    return val

def negative_log_likelihood(X):
    return -tf.reduce_sum(normal_log(X))

def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
