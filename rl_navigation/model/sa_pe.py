import math
import numpy as np
import torch
import torch.nn as nn
from torch.nn import init
from torch.nn import functional as F

from model.utils import log_normal_density


# 定义位置编码模块
class PositionalEncoding(nn.Module):
    def __init__(self, embed_dim, max_len=5000):
        super(PositionalEncoding, self).__init__()
        # 预先生成一个位置编码矩阵
        self.smooth_factor = 0.25  # 可调节的平滑因子，减小频率的作用
        pe = torch.zeros(max_len, embed_dim)
        position = torch.arange(0, max_len).unsqueeze(1).float()
        div_term = torch.exp(torch.arange(0, embed_dim, 2).float() * -(math.log(10000.0) / embed_dim))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0)  # 增加batch维度
        self.register_buffer('pe', pe)

    def forward(self, x):
        """
        将位置编码加到输入上
        x: 输入数据，形状为 (batch_size, seq_len, embed_dim)
        """
        batch_size, seq_len, _ = x.size()
        return x + self.smooth_factor * self.pe[:, :seq_len]



# 定义多头自注意力模块
class MultiHeadSelfAttention(nn.Module):
    def __init__(self, embed_dim, num_heads):
        super(MultiHeadSelfAttention, self).__init__()
        self.num_heads = num_heads
        self.head_dim = embed_dim // num_heads

        self.query = nn.Linear(embed_dim, embed_dim)
        self.key = nn.Linear(embed_dim, embed_dim)
        self.value = nn.Linear(embed_dim, embed_dim)
        self.fc_out = nn.Linear(1536, 256)

        self.positional_encoding = PositionalEncoding(embed_dim)

        # self.apply(self.init_weights)

    def init_weights(self, module):
        if isinstance(module, nn.Linear):
            # nn.init.kaiming_normal_(module.weight, mode='fan_out', nonlinearity='leaky_relu')
            nn.init.xavier_normal_(module.weight)
            if module.bias is not None:
                nn.init.constant_(module.bias, 0)

    def forward(self, x):

        batch_size, seq_len, embed_dim = x.size()

        x = self.positional_encoding(x)

        q = self.query(x).reshape(batch_size, seq_len, self.num_heads, self.head_dim).transpose(1, 2)
        k = self.key(x).reshape(batch_size, seq_len, self.num_heads, self.head_dim).transpose(1, 2)
        v = self.value(x).reshape(batch_size, seq_len, self.num_heads, self.head_dim).transpose(1, 2)
        # 计算注意力得分
        attn_weights = torch.matmul(q, k.transpose(-2, -1)) / math.sqrt(self.head_dim)
        attn_weights = torch.softmax(attn_weights, dim=-1)

        # 应用注意力权重
        attended_values = torch.matmul(attn_weights, v).transpose(1, 2).contiguous().reshape(batch_size, seq_len, embed_dim)
        # print(attended_values.shape)
        attended_values = attended_values.reshape(x.shape[0], 1, -1)
        # 经过线性变换和残差连接
        attended_values = self.fc_out(attended_values)
        attended_values = torch.tanh(attended_values)
        # x = self.norm1(attended_values + x)
        x = attended_values
        return x


class MLPPolicy(nn.Module):
    def __init__(self, obs_space, action_space):
        super(MLPPolicy, self).__init__()
        # action network

        #self.act_fea_cv1 = nn.Conv1d(in_channels=3, out_channels=32, kernel_size=5, stride=2, padding=1)
        #self.act_fea_cv2 = nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1)
        self.act_goal_fc1 = nn.Linear(2, 128)  # 2维目标信息全连接层1，输入2，输出128
        self.act_goal_fc2 = nn.Linear(128, 128)  # 2维目标信息全连接层1，输入2，输出128
        self.act_speed_fc1 = nn.Linear(2, 128)  # 2维速度
        self.act_speed_fc2 = nn.Linear(128, 128)
        self.act_self_attention_lidar = MultiHeadSelfAttention(512, 8)  # 处理激光雷达数据
        # self.act_fc1 = nn.Linear(512 + 512 + 512, 64)  # 3帧激光雷达，输入512+512+512，输出64
        # self.act_att_fc1 = nn.Linear(64, 64)  # 注意力机制全连接层，处理激光雷达数据
        self.act_att_fc2 = nn.Linear(128, 128)  # 处理速度
        self.act_att_fc3 = nn.Linear(128, 128)  # 处理目标
        self.act_fc2 = nn.Linear(256, 128)  # 雷达全连接层2
        self.mu = nn.Linear(128 + 128 + 128, 2)  # 输出层，合并3个128维的特征，输出2层动作空间
        self.mu.weight.data.mul_(0.1)  #初始化权重，*0.1
        # torch.log(std)
        self.logstd = nn.Parameter(torch.zeros(action_space))  # 初始化对数标准差参数，用于生成动作

        # value network
        self.crt_goal_fc1 = nn.Linear(2, 128)
        self.crt_goal_fc2 = nn.Linear(128, 128)
        self.crt_speed_fc1 = nn.Linear(2, 128)
        self.crt_speed_fc2 = nn.Linear(128, 128)
        # self.crt_fc1 = nn.Linear(512 + 512 + 512, 64)
        # self.crt_att_fc1 = nn.Linear(64, 64)
        self.crt_self_attention_lidar = MultiHeadSelfAttention(512, 8)
        self.crt_att_fc2 = nn.Linear(128, 128)
        self.crt_att_fc3 = nn.Linear(128, 128)
        self.crt_fc2 = nn.Linear(256, 128)
        self.crt_mu = nn.Linear(128 + 128 + 128, 1)  # 输出层，合并特征，输出1维价值
        self.crt_mu.weight.data.mul_(0.1)

    def forward(self, x, goal, speed):
        """
            returns value estimation, action, log_action_prob
        """
        # action
        # print(f"x.shape={x.shape}")
        # x = x.reshape(1,-1)
        # print(f"x.shape={x.shape}")
        # x = x.view(x.shape[0], -1)  # 将3帧雷达输入展平
        # act = self.act_self_attention_lidar(x.view(x.shape[0], 1, -1)).view(x.shape[0], -1)
        act = self.act_self_attention_lidar(x)
        act = self.act_fc2(act)  # 第二个全连接层，64---128
        act = torch.tanh(act)
        rlspeed = self.act_speed_fc1(speed)  # 2---128
        rlspeed = torch.tanh(rlspeed)
        speed_attention = self.act_att_fc2(rlspeed)  # 128---128
        rlspeed = torch.mul(rlspeed, speed_attention)  # 通道注意力
        rlspeed = self.act_speed_fc2(rlspeed)  # 128---128
        rlspeed = torch.tanh(rlspeed)
        rlgoal = self.act_goal_fc1(goal)
        rlgoal = torch.tanh(rlgoal)
        goal_attention = self.act_att_fc3(rlgoal)  # 通道注意力
        rlgoal = torch.mul(rlgoal, goal_attention)
        rlgoal = self.act_goal_fc2(rlgoal)
        rlgoal = torch.tanh(rlgoal)

        # 展平处理后的3个状态特征
        act = act.reshape(act.shape[0], -1)
        rlgoal = rlgoal.reshape(rlgoal.shape[0], -1)
        rlspeed = rlspeed.reshape(rlspeed.shape[0], -1)

        act = torch.cat((act, rlgoal, rlspeed), dim=-1)  # 合并特征
        mean = self.mu(act)  # 后并后的全连接层
        logstd = self.logstd.expand_as(mean)  # 扩展对数标准差到均值的维度
        std = torch.exp(logstd)  # 计算标准差
        action = torch.normal(mean, std)  # 生成动作

        # value
        # v = self.crt_self_attention_lidar(x.reshape(x.shape[0], 1, -1)).reshape(x.shape[0], -1)
        v = self.crt_self_attention_lidar(x)
        v = self.crt_fc2(v)
        v = torch.tanh(v)
        v_rlspeed = self.crt_speed_fc1(speed)
        v_rlspeed = torch.tanh(v_rlspeed)
        v_speed_attention = self.crt_att_fc2(v_rlspeed)
        v_rlspeed = torch.mul(v_rlspeed, v_speed_attention)
        v_rlspeed = self.crt_speed_fc2(v_rlspeed)
        v_rlspeed = torch.tanh(v_rlspeed)
        v_rlgoal = self.crt_goal_fc1(goal)
        v_rlgoal = torch.tanh(v_rlgoal)
        v_goal_attention = self.crt_att_fc3(v_rlgoal)
        v_rlgoal = torch.mul(v_rlgoal, v_goal_attention)
        v_rlgoal = self.crt_goal_fc2(v_rlgoal)
        v_rlgoal = torch.tanh(v_rlgoal)
        v = v.reshape(v.shape[0], -1)
        v = torch.cat((v, v_rlgoal, v_rlspeed), dim=-1)
        v = self.crt_mu(v)  # 后并后的全连接层，输出价值

        # action prob on log scale，计算动作的对数概率
        logprob = log_normal_density(action, mean, std=std, log_std=logstd)
        return v, action, logprob, mean
        # 返回价值，采样得到的动作，对数概率，均值

    def evaluate_actions(self, x, goal, speed, action):
        v, _, _, mean = self.forward(x, goal, speed)        # 前向传播计算状态值和动作均值
        logstd = self.logstd.expand_as(mean)
        std = torch.exp(logstd)
        # evaluate评估动作
        logprob = log_normal_density(action, mean, log_std=logstd, std=std)
        dist_entropy = 0.5 + 0.5 * math.log(2 * math.pi) + logstd   # 计算分布的熵
        dist_entropy = dist_entropy.sum(-1).mean()      # 计算分布熵的平均值
        return v, logprob, dist_entropy     # 返回状态值，动作的概率密度对数，分布熵

if __name__ == '__main__':
    net = MLPPolicy(3, 2)

    observation = torch.randn(2, 3)
    v, action, logprob, mean = net.forward(observation)
    print(v)
