#include "eskf.h"

eskf::eskf()
{
}

eskf::eskf(int dim)
{
    this->P = MatrixXd::Identity(dim, dim);
    this->dim = dim;
}

eskf::~eskf()
{
}

void eskf::eskf_init_P(VectorXd p)
{
    for (int i = 0; i < dim; i++)
    {
        P(i, i) = p(i); //初始化赋值操作
    }
}

void eskf::predict(MatrixXd Phi, MatrixXd X, MatrixXd P, MatrixXd G, MatrixXd Q)
{
    this->x = Phi * X;
    this->P = Phi * P * Phi.transpose() + G * Q * G.transpose();
}

//传入的参数为：观测误差（Z - HX），量测矩阵：H， 噪声增益R
void eskf::update(VectorXd dz, MatrixXd H, MatrixXd R)
{
    MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    this->dx = K * dz; //dx为状态估计误差
    MatrixXd I = MatrixXd::Identity(dim, dim);
    P = (I - K * H) * P;
}
//状态更新：P：k-1时刻到k时刻误差协方差阵预测，H是k时刻观测方程，R是k时刻观测方差阵
//X：k-1时刻到k时刻状态估计预测，K是k时刻卡尔曼增益阵，Phi是k-1时刻状态转移矩阵
//Z：k-1时刻到k时刻状态观测值
void eskf::update(MatrixXd P, MatrixXd H, MatrixXd R, MatrixXd X, MatrixXd K, VectorXd Z)
{
    this->x = X + K * (Z - H * X);
    MatrixXd I = MatrixXd::Identity(dim, dim);
    this->P = (I - K * H) * P;
}