"""
Lightweight copy of the Sparse GP (SGP2D) helper from pre_ws/gp_subgoal.

Only保留核心建模/训练接口，供 lste_topo_access 的 GP 子目标节点复用。
原节点包装（订阅点云/发布子目标）放在 lste_topo_access/scripts/gp_subgoals_sim_topo.py。
"""

import numpy as np
import tensorflow as tf
import gpflow
from gpflow import set_trainable


class SGP2D:
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
        in_init, out_init = np.zeros((0, 2)), np.zeros((0, 1))  # input dim:2
        mdl_in = tf.Variable(in_init, shape=(None, 2), dtype=tf.float32)
        mdl_out = tf.Variable(out_init, shape=(None, 1), dtype=tf.float32)
        self.data = (mdl_in, mdl_out)

    def set_training_data(self, d_in, d_out):
        mdl_in = tf.Variable(d_in, dtype=tf.float32)
        mdl_out = tf.Variable(d_out, dtype=tf.float32)
        self.data = (mdl_in, mdl_out)

    def set_empty_indpts(self):
        indpts_init = np.zeros((0, 2))
        self.indpts = tf.Variable(indpts_init, shape=(None, 2), dtype=tf.float32)

    def set_indpts_from_training_data(self, indpts_size, in_data):
        data_size = np.shape(in_data)[0]
        pts_idx = range(0, data_size, int(data_size / indpts_size))
        self.indpts = in_data[[idx for idx in pts_idx], :]

    def set_init_mean(self, init_mean):
        self.meanf = gpflow.mean_functions.Constant(init_mean)

    def set_sgp_model(self):
        self.model = gpflow.models.SGPR(self.data, self.kernel, self.indpts, mean_function=self.meanf)

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
