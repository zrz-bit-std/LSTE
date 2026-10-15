"""Sparse Gaussian-process model used by the legacy GP frontier node.

This is deliberately independent of ROS.  The node owns incoming point clouds
and output topics; this module owns only GPFlow model construction and fitting.
"""

import numpy as np
import tensorflow as tf
import gpflow
from gpflow import set_trainable


gpflow.config.set_default_float(np.float32)
np.random.seed(0)
tf.random.set_seed(0)


class SGP2D:
    """Create a two-dimensional sparse GP for spherical occupancy samples."""

    def __init__(self):
        self.model = None
        self.data = None
        self.kernel1 = None
        self.kernel2 = None
        self.kernel = None
        self.indpts = None
        self.meanf = gpflow.mean_functions.Constant(0)

    def set_kernel_param(self, ls1, ls2, var, alpha, noise, noise_var):
        self.kernel1 = gpflow.kernels.RationalQuadratic(lengthscales=[ls1, ls2])
        self.kernel1.variance.assign(var)
        self.kernel1.alpha.assign(alpha)
        self.kernel2 = gpflow.kernels.White(noise)
        self.kernel2.variance.assign(noise_var)
        self.kernel = self.kernel1 + self.kernel2

    def set_empty_data(self):
        inputs, outputs = np.zeros((0, 2)), np.zeros((0, 1))
        self.data = (
            tf.Variable(inputs, shape=(None, 2), dtype=tf.float32),
            tf.Variable(outputs, shape=(None, 1), dtype=tf.float32),
        )

    def set_training_data(self, inputs, outputs):
        self.data = (
            tf.Variable(inputs, dtype=tf.float32),
            tf.Variable(outputs, dtype=tf.float32),
        )

    def set_empty_indpts(self):
        self.indpts = tf.Variable(
            np.zeros((0, 2)), shape=(None, 2), dtype=tf.float32
        )

    def set_indpts_from_training_data(self, indpts_size, inputs):
        data_size = np.shape(inputs)[0]
        stride = int(data_size / indpts_size)
        indices = range(0, data_size, stride)
        self.indpts = inputs[[index for index in indices], :]

    def set_init_mean(self, init_mean):
        self.meanf = gpflow.mean_functions.Constant(init_mean)

    def set_sgp_model(self):
        self.model = gpflow.models.SGPR(
            self.data, self.kernel, self.indpts, mean_function=self.meanf
        )

    def select_trainable_param(self):
        set_trainable(self.kernel1.variance, False)
        set_trainable(self.kernel1.lengthscales, False)
        set_trainable(self.kernel2.variance, False)
        set_trainable(self.model.likelihood.variance, False)

    def minimize_loss(self):
        self.model.training_loss_closure()

    def adam_optimize_param(self):
        optimizer = tf.optimizers.Adam()
        optimizer.minimize(self.model.training_loss, self.model.trainable_variables)
