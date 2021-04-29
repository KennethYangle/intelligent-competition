#include "img_imu_ekf.h"

Matrix3d _skew_symmetric3(Vector3d v) //斜对称矩阵
{
    Matrix3d m; //给m矩阵赋值
    m << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return m;
}

img_imu_ekf::img_imu_ekf()
{
    is_init_done = false;
    is_atti_init_done = false;
    this->q.setIdentity(); //四元数初始化为0
    this->pos.setZero();
    this->vel.setZero();
    this->img.setZero();
    this->acc_bias.setZero();
    this->gyro_bias.setZero();
}

img_imu_ekf::~img_imu_ekf()
{
}

void img_imu_ekf::set_P_matrix()
{
    //获取从向量的第i个元素开始的n个元素：vector.segment(i,n)
    VectorXd p = VectorXd::Ones(dim);
    p.segment(0, 4) = Vector4d::Ones() * gyro_noise * gyro_noise;
    p.segment(4, 3) = Vector3d::Ones() * gps_pos_noise * gps_pos_noise;
    p.segment(7, 3) = Vector3d::Ones() * gps_vel_noise * gps_vel_noise;
    p.segment(10, 2) = Vector2d::Ones() * img_noise * img_noise;
    p.segment(12, 3) = Vector3d::Ones() * 0.01 * 0.01;
    p.segment(15, 3) = Vector3d::Ones() * 0.01 * 0.01;
    // kf

    kf.eskf_init_P(p);
}

//定义各常熟矩阵   Q:过程噪声，G：系统噪声阵   R：测量噪声
void img_imu_ekf::set_Q_matrix(double dt)
{
    Q = MatrixXd::Zero(dim, dim);
    Matrix2d I2 = Matrix2d::Identity(); //定义单位矩阵
    Matrix3d I3 = Matrix3d::Identity(); //定义单位矩阵
    Matrix4d I4 = Matrix4d::Identity(); //定义单位矩阵
    //状态的顺序为：q,p,v,img,b_gyr,b_acc
    //其中，四元数用欧拉角表示
    //目前这个是随意设置的，不一定正确,四元数直接从mavros中读取
    Q.block(0, 0, 4, 4) = I4 * gyro_noise * gyro_noise;             //q
    Q.block(4, 4, 3, 3) = I3 * gps_vel_noise * gps_vel_noise;       //pos
    Q.block(7, 7, 3, 3) = I3 * acc_noise * acc_noise;               //v
    Q.block(10, 10, 2, 2) = I2 * img_noise * img_noise;             //img
    Q.block(12, 12, 3, 3) = I3 * gyro_bias_noise * gyro_bias_noise; //b_gyr
    Q.block(15, 15, 3, 3) = I3 * acc_bias_noise * acc_bias_noise;   //b_acc
    Q = 100* Q;

    //定义测量方差阵
    R = MatrixXd::Zero(dim, dim);
    R.block(0, 0, 4, 4) = I4 * gyro_noise * gyro_noise;             //q
    R.block(4, 4, 3, 3) = I3 * gps_vel_noise * gps_vel_noise;       //pos
    R.block(7, 7, 3, 3) = I3 * acc_noise * acc_noise;               //v
    R.block(10, 10, 2, 2) = 0.001 * I2;//100 * I2 * img_noise * img_noise;             //img
    R.block(12, 12, 3, 3) = I3 * gyro_bias_noise * gyro_bias_noise * dt; //b_gyr
    R.block(15, 15, 3, 3) = I3 * acc_bias_noise * acc_bias_noise * dt;   //b_acc
    R = 0.001 * R;
}

void img_imu_ekf::sensor_init(Vector4d q, Vector3d pos, Vector3d vel, Vector2d img)
{
    kf = eskf(dim);

    // this->q = q;
    // Eigen::Quaterniond q2(Vector4d(x, y, z, w)); // 第二种方式
    //mavros读取四元数的顺序是(q.w, q.x, q.y, q.z)？？？这个需要确认，目前是按照这种方式读取的
    // Quaterniond vec_q(q(0), q(1), q(2), q(3));
    // this->q = vec_q;
    this->q.w() = q(0);
    this->q.x() = q(1);
    this->q.y() = q(2);
    this->q.z() = q(3);
    this->pos = pos;
    this->vel = vel;
    this->img = img;
    is_init_done = true;
    // kf
    //p.segment(0, 4) = Vector4d::Ones() * gyro_noise * gyro_noise;
    //将q，pos，vel，img，b_gyr，b_acc转换成状态变量x,这里即为初始化状态向量x
    kf.x = VectorXd::Ones(18);
    kf.x.segment(0, 4) = q;
    kf.x.segment(4, 3) = pos;
    kf.x.segment(7, 3) = vel;
    kf.x.segment(10, 2) = img;
    kf.x.segment(12, 3) = Vector3d::Ones() * 0.01 * 0.01;
    kf.x.segment(15, 3) = Vector3d::Ones() * 0.01 * 0.01;

    Matrix2d I2 = Matrix2d::Identity(); //定义单位矩阵
    kf.H = MatrixXd::Zero(dim, dim);
    kf.H.block(10,10,2,2) = I2;

    this->set_Q_matrix(1.0 / imu_freq);
    this->set_P_matrix();
}

//参数: w:为机体坐标系下的角速度  vel：为世界坐标系下的速度  dt：为时间间隔
//predict函数输入的参数为第k时刻的参数，而最原始的this->参数为第k-1时刻
//系统噪声阵G的更新
void img_imu_ekf::update_Phi(Vector3d w, Vector3d vel, Vector3d acc, double dt)
{
    Matrix4d I4 = Matrix4d::Identity();
    Matrix3d I3 = Matrix3d::Identity();
    Matrix2d I2 = Matrix2d::Identity();

    MatrixXd R_bc;
    R_bc = MatrixXd::Zero(3, 3);
    // R_bc << 1,0,0,
    //         0,0,1,
    //         0,-1,0;
    R_bc << 0,1,0,
            0,0,-1,
            1,0,0;

    //predict q
    //R_eb:机体系到世界系下的旋转矩阵，F_q：四元数到四元数的状态方程，F_q_gyr：陀螺仪到四元数的旋转矩阵
    MatrixXd dMq_pre, F_q, F_q_gyr, R_eb;
    dMq_pre = MatrixXd::Zero(4, 4);
    F_q_gyr = MatrixXd::Zero(4, 3);
    R_eb = MatrixXd::Zero(3, 3);
    // w:为传入进来的角速度值，this->w：为角度变化值
    Vector3d pre_w(this->w(0), this->w(1), this->w(2));
    // Vector3d pre_w(this->w(0) * dt - kf.x(12), this->w(1) * dt - kf.x(13), this->w(2) * dt - kf.x(14));
    // Vector3d pre_w_b(kf.x(12), kf.x(13), kf.x(14));
    dMq_pre << 1, -pre_w(0) / 2, -pre_w(1) / 2, -pre_w(2) / 2,
        pre_w(0) / 2, 1, pre_w(2) / 2, -pre_w(1) / 2,
        pre_w(1) / 2, -pre_w(2) / 2, 1, pre_w(0) / 2,
        pre_w(2) / 2, pre_w(1) / 2, -pre_w(0) / 2, 1;
    this->w = w;

    F_q = dMq_pre;
    // Vector4d pre_q(this->q.w(), this->q.x(), this->q.y(), this->q.z());
    //q的状态是机体坐标系到世界坐标系下的关系，但是mavros获取到的都是世界坐标系到机体坐标系下的关系，因为需要学会转换，在赋值的时候
    //pre_q:上一时刻，机体坐标系到世界坐标系下的旋转矩阵,状态x：0——3为四元数前四个
    Quaterniond pre_q;
    pre_q.w() = kf.x(0);
    pre_q.x() = kf.x(1);
    pre_q.y() = kf.x(2);
    pre_q.z() = kf.x(3);
    R_eb = pre_q.toRotationMatrix();

    F_q_gyr << pre_q.x() / 2, pre_q.y() / 2, pre_q.z() / 2,
        -pre_q.w() / 2, pre_q.z() / 2, -pre_q.y() / 2,
        -pre_q.z() / 2, -pre_q.w() / 2, pre_q.x() / 2,
        pre_q.y() / 2, -pre_q.x() / 2, -pre_q.w() / 2;

    // this->q = F_q * this->q + F_q_gyr * this->gyro_bias;

    //predict pos
    // pos += this->vel * dt;

    //predict v
    //mavros读取四元数的顺序是(q.w, q.x, q.y, q.z)？？？这个需要确认，目前是按照这种方式读取的
    MatrixXd Phi_1, Phi_2, Phi_3;

    Phi_1 = MatrixXd::Zero(4, 3);
    Phi_2 = MatrixXd::Zero(4, 3);
    Phi_3 = MatrixXd::Zero(4, 3);
    Phi_1 << pre_q.w(), -pre_q.z(), pre_q.y(),
        pre_q.x(), pre_q.y(), pre_q.z(),
        -pre_q.y(), pre_q.x(), pre_q.w(),
        -pre_q.z(), -pre_q.w(), pre_q.x();

    Phi_2 << pre_q.z(), pre_q.w(), -pre_q.x(),
        pre_q.y(), -pre_q.x(), -pre_q.w(),
        pre_q.x(), pre_q.y(), pre_q.z(),
        pre_q.w(), -pre_q.z(), pre_q.y();

    Phi_3 << -pre_q.y(), pre_q.x(), pre_q.w(),
        pre_q.z(), pre_q.w(), -pre_q.x(),
        -pre_q.w(), pre_q.z(), -pre_q.y(),
        pre_q.x(), pre_q.y(), pre_q.z();

    
    // 方法一：直接从mavros读取速度值获取速度增量？？
    // Vector3d pre_vel(kf.x(7), kf.x(8), kf.x(9));
    // Vector3d dv = vel - pre_vel;

    //方法二：步骤一：从mavros读取加速度，然后乘以dt，获得及体系下的速度增量
    //      步骤二：将机体系下的速度增量转移到世界系下（东北天坐标系）
    Vector3d vel_acc(kf.x(15), kf.x(16), kf.x(17));
    Vector3d pre_vel(kf.x(7), kf.x(8), kf.x(9));
    Vector3d dv = R_eb * (acc * dt - vel_acc);
    MatrixXd F_vel_q;
    F_vel_q = MatrixXd::Zero(3, 4);
    F_vel_q << (Phi_1 * dv)(0), (Phi_1 * dv)(1), (Phi_1 * dv)(2), (Phi_1 * dv)(3),
        (Phi_2 * dv)(0), (Phi_2 * dv)(1), (Phi_2 * dv)(2), (Phi_2 * dv)(3),
        (Phi_3 * dv)(0), (Phi_3 * dv)(1), (Phi_3 * dv)(2), (Phi_3 * dv)(3);
    MatrixXd F_v_dv;
    F_v_dv = MatrixXd::Zero(3, 3);
    F_v_dv = -R_eb;
    // Vector2d pre_vel = this->vel;
    // this->vel = this->vel + F_vel_q * q + F_v_dv * this->acc_bias;

    //predict img
    MatrixXd F_img_q, F_img_v, F_img_gyr, F_img, F_img_q_process;
    F_img_q = MatrixXd::Zero(2, 4);
    F_img_q_process = MatrixXd::Zero(4, 2);
    F_img_v = MatrixXd::Zero(2, 3);
    F_img_gyr = MatrixXd::Zero(2, 3);
    F_img = MatrixXd::Zero(2, 2);
    Vector2d pre_img(kf.x(10) / img_f, kf.x(11) / img_f); // img_f为相机焦距,pre_img为归一化后的像素坐标

    //状态x：3——5为分别为位置变量
    //P_c(2)为P_cz
    Vector3d P_pre(kf.x(3), kf.x(4), kf.x(5));
    Vector3d P_c = R_bc.transpose() * R_eb.transpose()* P_pre;

    F_img_v << -1 / P_c(2), 0, pre_img(0) / P_c(2),
        0, -1 / P_c(2), pre_img(1) / P_c(2);
    F_img_v = F_img_v*R_bc.transpose()*R_eb.transpose();

    F_img_gyr << pre_img(0) * pre_img(1), -(1 + pre_img(0) * pre_img(0)), pre_img(1),
        1 + pre_img(1) * pre_img(1), -pre_img(0) * pre_img(1), -pre_img(0);
    
    F_img_gyr = - F_img_gyr*R_bc.transpose();

    
    F_img_q_process << (pre_img(0) * pre_q.w() + pre_q.z()) * pre_vel(0) + (pre_img(0) * pre_q.z() - pre_q.w()) * pre_vel(1) + (-pre_img(0) * pre_q.y() - pre_q.x()) * pre_vel(2),
        (pre_img(1) * pre_q.w() - pre_q.y()) * pre_vel(0) + (pre_img(1) * pre_q.z() + pre_q.x()) * pre_vel(1) + (-pre_img(1) * pre_q.y() - pre_q.w()) * pre_vel(2),
        (pre_img(0) * pre_q.x() - pre_q.y()) * pre_vel(0) + (pre_img(0) * pre_q.y() + pre_q.x()) * pre_vel(1) + (pre_img(0) * pre_q.z() - pre_q.w()) * pre_vel(2),
        (pre_img(1) * pre_q.x() - pre_q.z()) * pre_vel(0) + (pre_img(1) * pre_q.y() + pre_q.w()) * pre_vel(1) + (pre_img(1) * pre_q.z() + pre_q.x()) * pre_vel(2),
        (pre_img(0) * pre_q.y() - pre_q.x()) * pre_vel(0) + (pre_img(0) * pre_q.x() - pre_q.y()) * pre_vel(1) + (-pre_img(0) * pre_q.w() - pre_q.z()) * pre_vel(2),
        (pre_img(1) * pre_q.y() - pre_q.w()) * pre_vel(0) + (pre_img(1) * pre_q.x() - pre_q.z()) * pre_vel(1) + (-pre_img(1) * pre_q.w() + pre_q.y()) * pre_vel(2),
        (pre_img(0) * pre_q.z() + pre_q.w()) * pre_vel(0) + (pre_img(0) * pre_q.w() + pre_q.z()) * pre_vel(1) + (pre_img(0) * pre_q.x() - pre_q.y()) * pre_vel(2),
        (pre_img(1) * pre_q.z() - pre_q.x()) * pre_vel(0) + (pre_img(1) * pre_q.w() - pre_q.y()) * pre_vel(1) + (pre_img(1) * pre_q.x() - pre_q.z()) * pre_vel(2);

    F_img_q = 2 * dt * F_img_q_process.transpose() / P_c(2);

    //w_c:为相机坐标系下的角度变化
    Vector3d w_c,v_c;
    // w_c = R_bc.transpose()* this->w;
    w_c = R_bc.transpose() * pre_w;//2021.04.28
    v_c = R_bc.transpose() * R_eb.transpose() * pre_vel;
    F_img << v_c(0)/P_c(2) + pre_img(1)*w_c(0)-2*pre_img(0)*w_c(1), pre_img(0)*w_c(0) + w_c(2),
            -pre_img(1)*w_c(1) - w_c(2), v_c(2)/P_c(2) + 2*pre_img(1)*w_c(0) - pre_img(0)*w_c(1);
    F_img = I2 + F_img * dt;
    // F_img << dt * v_c(0) / P_c(2) + pre_img(1) * w_c(0) - 2 * pre_img(0) * w_c(1), pre_img(0) * w_c(0) + w_c(2),
    //     -pre_img(1) * w_c(1) - w_c(2), dt * v_c(2) / P_c(2) + 2 * pre_img(1) * w_c(0) - pre_img(0) * w_c(1);
    // F_img = I2 + F_img;

    // F_img = F_img_v * R_be.transpose() * this->vel + R_be.transpose() * w;
    // this->img = F_img_v * pre_vel + F_img * this->img + F_img_gyr * this->gyro_bias;

    Phi = MatrixXd::Identity(dim, dim);

    //quaternioned part
    Phi.block(0, 0, 4, 4) = F_q;
    Phi.block(0, 12, 4, 3) = F_q_gyr;

    //position part
    Phi.block(4, 4, 3, 3) = I3;
    Phi.block(4, 7, 3, 3) = I3 * dt;

    //veolicity part
    Phi.block(7, 0, 3, 4) = F_vel_q;
    Phi.block(7, 7, 3, 3) = I3;
    Phi.block(7, 15, 3, 3) = F_v_dv;

    //image part  这个正确性有待确认
    Phi.block(10, 0, 2, 4) = F_img_q;
    Phi.block(10, 7, 2, 3) = F_img_v;
    Phi.block(10, 10, 2, 2) = F_img;
    Phi.block(10, 12, 2, 3) = F_img_gyr;
    //byro part
    Phi.block(12, 12, 3, 3) = I3;
    //acc part
    Phi.block(15, 15, 3, 3) = I3;

    //定义系统噪声阵，目前认为为单位阵
    // G = MatrixXd::Identity(dim, dim);
    G = MatrixXd::Zero(dim, dim);
    G.block(0, 12, 4, 3) = F_q_gyr;
    G.block(7, 15, 3, 3) = F_v_dv;
    G.block(10, 12, 2, 3) = F_img_gyr;
}
