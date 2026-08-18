
//* ROS action server */
#include "rm_control.h"
/* 三次样条插补 */
#include "cubicSpline.h"
#include <atomic>
#include <array>
#include <cmath>

using namespace std;
vector<double> time_from_start_;
vector<double> p_joint1_;
vector<double> p_joint2_;
vector<double> p_joint3_;
vector<double> p_joint4_;
vector<double> p_joint5_;
vector<double> p_joint6_;
vector<double> p_joint7_;
vector<double> v_joint1_;
vector<double> v_joint2_;
vector<double> v_joint3_;
vector<double> v_joint4_;
vector<double> v_joint5_;
vector<double> v_joint6_;
vector<double> v_joint7_;
vector<double> a_joint1_;
vector<double> a_joint2_;
vector<double> a_joint3_;
vector<double> a_joint4_;
vector<double> a_joint5_;
vector<double> a_joint6_;
vector<double> a_joint7_;

/* 存储的结构体 p2*/
struct vel_data
{
    int vector_len;    //toatal num
    int vector_cnt;    //current num
};

/* 数据收发结构体 */
struct vel_data p2;

/* action 服务端声明 */
// typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

/* 初始化输入输出速度加速度 */
double acc = 0, vel = 0;
double x_out = 0, y_out = 0;

/* 判断路点数据是否改变 */
std::atomic_bool point_changed{false};
std::atomic_uint32_t movej_canfd_error{0};
std::atomic_uint64_t movej_canfd_result_count{0};

/*三次样条插值后周期*/
double rate = 0.005;    // 5 ms: RM75 high-follow CANFD transmission period

float min_interval = 20;  //透传周期,单位:秒
float wait_move_finish_time = 1.5;  //等待运动到位时间,单位:秒
int count_keep_send = 0;
int count_final_joint = 0;

/* 三次样条无参构造 */
cubicSpline::cubicSpline()
{
}
/* 析构 */
cubicSpline::~cubicSpline()
{
    releaseMem();
}
/* 初始化参数 */
void cubicSpline::initParam()
{
    x_sample_ = y_sample_ = M_ = NULL;
    sample_count_ = 0;
    bound1_ = bound2_ = 0;
}
/* 释放参数 */
void cubicSpline::releaseMem()
{
    delete x_sample_;
    delete y_sample_;
    delete M_;

    initParam();
}
/* 加载关节位置数组等信息 */
bool cubicSpline::loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type)
{
    if ((NULL == x_data) || (NULL == y_data) || (count < 3) || (type > BoundType_Second_Derivative) || (type < BoundType_First_Derivative))
    {
        return false;
    }
    initParam();

    x_sample_ = new double[count];
    y_sample_ = new double[count];
    M_        = new double[count];
    sample_count_ = count;

    memcpy(x_sample_, x_data, sample_count_*sizeof(double));
    memcpy(y_sample_, y_data, sample_count_*sizeof(double));

    bound1_ = bound1;
    bound2_ = bound2;

    return spline(type);
}
/* 计算样条插值 */
bool cubicSpline::spline(BoundType type)
{
    if ((type < BoundType_First_Derivative) || (type > BoundType_Second_Derivative))
    {
        return false;
    }

    //  追赶法解方程求二阶偏导数
    double f1=bound1_, f2=bound2_;

    double *a=new double[sample_count_];                //  a:稀疏矩阵最下边一串数
    double *b=new double[sample_count_];                //  b:稀疏矩阵最中间一串数
    double *c=new double[sample_count_];                //  c:稀疏矩阵最上边一串数
    double *d=new double[sample_count_];

    double *f=new double[sample_count_];

    double *bt=new double[sample_count_];
    double *gm=new double[sample_count_];

    double *h=new double[sample_count_];

    for(int i=0;i<sample_count_;i++)
        b[i]=2;                                //  中间一串数为2
    for(int i=0;i<sample_count_-1;i++)
    {
        if (x_sample_[i+1] == x_sample_[i])
        {
            h[i] = 0.005;
        }
        else
        {
            h[i]=x_sample_[i+1]-x_sample_[i];      // 各段步长
        }
    }
    for(int i=1;i<sample_count_-1;i++)
        a[i]=h[i-1]/(h[i-1]+h[i]);
    a[sample_count_-1]=1;

    c[0]=1;
    for(int i=1;i<sample_count_-1;i++)
        c[i]=h[i]/(h[i-1]+h[i]);

    for(int i=0;i<sample_count_-1;i++)
    {
        if (x_sample_[i+1] == x_sample_[i])
        {
            f[i]=(y_sample_[i+1]-y_sample_[i])/0.005;
        }
        else
        {
            f[i]=(y_sample_[i+1]-y_sample_[i])/(x_sample_[i+1]-x_sample_[i]);
        }
    }

    for(int i=1;i<sample_count_-1;i++)
        d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);

    //  追赶法求解方程
    if(BoundType_First_Derivative == type)
    {
        d[0]=6*(f[0]-f1)/h[0];
        d[sample_count_-1]=6*(f2-f[sample_count_-2])/h[sample_count_-2];

        bt[0]=c[0]/b[0];
        for(int i=1;i<sample_count_-1;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[0]=d[0]/b[0];
        for(int i=1;i<=sample_count_-1;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        M_[sample_count_-1]=gm[sample_count_-1];
        for(int i=sample_count_-2;i>=0;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];
    }
    else if(BoundType_Second_Derivative == type)
    {
        d[1]=d[1]-a[1]*f1;
        d[sample_count_-2]=d[sample_count_-2]-c[sample_count_-2]*f2;

        bt[1]=c[1]/b[1];
        for(int i=2;i<sample_count_-2;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[1]=d[1]/b[1];
        for(int i=2;i<=sample_count_-2;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        M_[sample_count_-2]=gm[sample_count_-2];
        for(int i=sample_count_-3;i>=1;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];

        M_[0]=f1;
        M_[sample_count_-1]=f2;
    }
    else
        return false;

    delete a;
    delete b;
    delete c;
    delete d;
    delete gm;
    delete bt;
    delete f;
    delete h;

    return true;
}
/* 得到速度和加速度数组 */
bool cubicSpline::getYbyX(double &x_in, double &y_out)
{
    int klo,khi,k;
    klo=0;
    khi=sample_count_-1;
    double hh,bb,aa;

    //  二分法查找x所在区间段
    while(khi-klo>1)
    {
        k=(khi+klo)>>1;
        if(x_sample_[k]>x_in)
            khi=k;
        else
            klo=k;
    }
    hh=x_sample_[khi]-x_sample_[klo];

    aa=(x_sample_[khi]-x_in)/hh;
    bb=(x_in-x_sample_[klo])/hh;

    y_out=aa*y_sample_[klo]+bb*y_sample_[khi]+((aa*aa*aa-aa)*M_[klo]+(bb*bb*bb-bb)*M_[khi])*hh*hh/6.0;

    //test
    acc = (M_[klo]*(x_sample_[khi]-x_in) + M_[khi]*(x_in - x_sample_[klo])) / hh;
    vel = M_[khi]*(x_in - x_sample_[klo]) * (x_in - x_sample_[klo]) / (2 * hh)
          - M_[klo]*(x_sample_[khi]-x_in) * (x_sample_[khi]-x_in) / (2 * hh)
          + (y_sample_[khi] - y_sample_[klo])/hh
          - hh*(M_[khi] - M_[klo])/6;

    return true;
}


Rm_Control::Rm_Control(std::string name) : Node(name)
{

    using namespace std::placeholders;
 

    this->declare_parameter<int>("arm_type", arm_type_);
    this->get_parameter("arm_type", arm_type_);

    this->declare_parameter<bool>("follow", follow_);
    this->get_parameter("follow", follow_);

    if((arm_type_ == 75)||(arm_type_ == 72))
    {
        joint_msg.joint.resize(7);
        joint_msg.dof = 7;
        arm_type_ = 75;
    }
    else
    {
        joint_msg.joint.resize(6);
        joint_msg.dof = 6;
    }

    State_Timer = this->create_wall_timer(std::chrono::milliseconds(5),
        std::bind(&Rm_Control::timer_callback,this));

    this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
                this, "rm_group_controller/follow_joint_trajectory",
                std::bind(&Rm_Control::handle_goal, this, _1, _2),
                std::bind(&Rm_Control::handle_cancel, this, _1),
                std::bind(&Rm_Control::handle_accepted, this, _1));

    rclcpp::QoS qos(10);

    joint_pos_publisher = this->create_publisher<rm_ros_interfaces::msg::Jointpos>("rm_driver/movej_canfd_cmd", qos);

    Get_Move_Stop_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/move_stop_cmd",rclcpp::ParametersQoS(),
        std::bind(&Rm_Control::get_move_stop_callback,this,std::placeholders::_1));

    Movej_CANFD_Result = this->create_subscription<std_msgs::msg::UInt32>(
        "rm_driver/movej_canfd_result", rclcpp::ParametersQoS(),
        std::bind(&Rm_Control::movej_canfd_result_callback, this, std::placeholders::_1));

    Joint_State_Sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        std::bind(&Rm_Control::joint_state_callback, this, std::placeholders::_1));

}

rclcpp_action::GoalResponse Rm_Control::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    std::cout << "---handle goal:" << goal->trajectory.joint_names.size() << std::endl;
    std::cout << goal->trajectory.header.frame_id.c_str() << goal->trajectory.header.stamp.sec << goal->trajectory.header.stamp.nanosec << std::endl;

    int pointSize = goal->trajectory.points.size();
    if(pointSize > 0)
    {
        for(int i = 0; i < pointSize; i++)
        {
            auto point = goal->trajectory.points.at(i);
        }
    }

    (void)uuid;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Rm_Control::handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    return rclcpp_action::CancelResponse::ACCEPT;
}

void Rm_Control::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle) 
{
    using std::placeholders::_1;

    std::thread{std::bind(&Rm_Control::execute_move, this, _1), goal_handle}
    .detach();
}

/* 收到action的goal后调用的回调函数 */
void Rm_Control::execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    int i = 0;
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    int point_num = goal->trajectory.points.size();
    RCLCPP_INFO(this->get_logger(), "First Move_group give us %d points",point_num);
    point_changed = false;
    movej_canfd_error = 0;
    movej_canfd_result_count = 0;

    if (point_num <= 0)
    {
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "Trajectory contains no points";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
        return;
    }

    const int dof = (arm_type_ == 75) ? 7 : 6;
    const std::array<std::string, 7> expected_joint_names = {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    std::array<size_t, 7> joint_index{};

    for (int joint = 0; joint < dof; ++joint)
    {
        const auto found = std::find(
            goal->trajectory.joint_names.begin(), goal->trajectory.joint_names.end(),
            expected_joint_names[joint]);
        if (found == goal->trajectory.joint_names.end())
        {
            result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
            result->error_string = "Trajectory is missing " + expected_joint_names[joint];
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
            return;
        }
        joint_index[joint] = static_cast<size_t>(
            std::distance(goal->trajectory.joint_names.begin(), found));
    }

    for (const auto & point : goal->trajectory.points)
    {
        if (point.positions.size() < goal->trajectory.joint_names.size())
        {
            result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
            result->error_string = "Trajectory point has fewer positions than joint_names";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
            return;
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Validated trajectory joint mapping: joint1..joint%d", dof);

    const auto value_or_zero = [](const std::vector<double> & values, size_t index) {
        return index < values.size() ? values[index] : 0.0;
    };

    // High-follow streaming must start from the physical robot state.  A stale
    // MoveIt start state produces a discontinuous first CANFD point and can be
    // interpreted by the controller as a collision/protection event.
    std::vector<double> start_actual_positions(dof, 0.0);
    bool complete_start_feedback = true;
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (!joint_state_received_)
        {
            complete_start_feedback = false;
        }
        else
        {
            for (int joint = 0; joint < dof; ++joint)
            {
                const auto found = std::find(
                    latest_joint_names_.begin(), latest_joint_names_.end(),
                    expected_joint_names[joint]);
                if (found == latest_joint_names_.end())
                {
                    complete_start_feedback = false;
                    break;
                }
                const size_t feedback_index = static_cast<size_t>(
                    std::distance(latest_joint_names_.begin(), found));
                if (feedback_index >= latest_joint_positions_.size())
                {
                    complete_start_feedback = false;
                    break;
                }
                start_actual_positions[joint] = latest_joint_positions_[feedback_index];
            }
        }
    }

    if (!complete_start_feedback)
    {
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "Cannot validate trajectory start: incomplete /joint_states feedback";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
        return;
    }

    const auto & first_positions = goal->trajectory.points.front().positions;
    double max_start_error = 0.0;
    int worst_start_joint = 0;
    for (int joint = 0; joint < dof; ++joint)
    {
        const double error = std::abs(
            start_actual_positions[joint] - first_positions[joint_index[joint]]);
        if (error > max_start_error)
        {
            max_start_error = error;
            worst_start_joint = joint;
        }
    }

    constexpr double maximum_start_discontinuity = 0.15;  // rad, about 8.6 degrees
    if (max_start_error > maximum_start_discontinuity)
    {
        result->error_code = FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP;
        result->error_string =
            "Trajectory start differs from the physical robot; " +
            expected_joint_names[worst_start_joint] + " error=" +
            std::to_string(max_start_error) + " rad. Re-plan from the current state.";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
        return;
    }

    /***********Start gubs 2021/9/16 修复Moveit在同一位姿重复规划执行导致rm_contorl异常停止的Bug***********/
    if  (point_num > 3) //判断当moveit规划的路点数大于3时为有效规划并进行三次样条插值
    {

        /* 各个关节位置 */
        double p_joint1[point_num];
        double p_joint2[point_num];
        double p_joint3[point_num];
        double p_joint4[point_num];
        double p_joint5[point_num];
        double p_joint6[point_num];
        double p_joint7[point_num];

        /* 各个关节速度 */
        double v_joint1[point_num];
        double v_joint2[point_num];
        double v_joint3[point_num];
        double v_joint4[point_num];
        double v_joint5[point_num];
        double v_joint6[point_num];
        double v_joint7[point_num];

        /* 各个关节加速度 */
        double a_joint1[point_num];
        double a_joint2[point_num];
        double a_joint3[point_num];
        double a_joint4[point_num];
        double a_joint5[point_num];
        double a_joint6[point_num];
        double a_joint7[point_num];

        /* 时间数组 */
        double time_from_start[point_num];
        double timens_from_start[point_num];

        for (i = 0; i < point_num; i++) 
        {
            p_joint1[i] = goal->trajectory.points[i].positions[joint_index[0]];
            p_joint2[i] = goal->trajectory.points[i].positions[joint_index[1]];
            p_joint3[i] = goal->trajectory.points[i].positions[joint_index[2]];
            p_joint4[i] = goal->trajectory.points[i].positions[joint_index[3]];
            p_joint5[i] = goal->trajectory.points[i].positions[joint_index[4]];
            p_joint6[i] = goal->trajectory.points[i].positions[joint_index[5]];
            if(arm_type_ == 75)
            {
                p_joint7[i] = goal->trajectory.points[i].positions[joint_index[6]];
            }
            

            v_joint1[i] = value_or_zero(goal->trajectory.points[i].velocities, joint_index[0]);
            v_joint2[i] = value_or_zero(goal->trajectory.points[i].velocities, joint_index[1]);
            v_joint3[i] = value_or_zero(goal->trajectory.points[i].velocities, joint_index[2]);
            v_joint4[i] = value_or_zero(goal->trajectory.points[i].velocities, joint_index[3]);
            v_joint5[i] = value_or_zero(goal->trajectory.points[i].velocities, joint_index[4]);
            v_joint6[i] = value_or_zero(goal->trajectory.points[i].velocities, joint_index[5]);
            if(arm_type_ == 75)
            {
                v_joint7[i] = value_or_zero(goal->trajectory.points[i].velocities, joint_index[6]);
            }

            a_joint1[i] = value_or_zero(goal->trajectory.points[i].accelerations, joint_index[0]);
            a_joint2[i] = value_or_zero(goal->trajectory.points[i].accelerations, joint_index[1]);
            a_joint3[i] = value_or_zero(goal->trajectory.points[i].accelerations, joint_index[2]);
            a_joint4[i] = value_or_zero(goal->trajectory.points[i].accelerations, joint_index[3]);
            a_joint5[i] = value_or_zero(goal->trajectory.points[i].accelerations, joint_index[4]);
            a_joint6[i] = value_or_zero(goal->trajectory.points[i].accelerations, joint_index[5]);
            if(arm_type_ == 75)
            {
                a_joint7[i] = value_or_zero(goal->trajectory.points[i].accelerations, joint_index[6]);
            }

            time_from_start[i] = goal->trajectory.points[i].time_from_start.sec + goal->trajectory.points[i].time_from_start.nanosec/1e9;
            timens_from_start[i] = goal->trajectory.points[i].time_from_start.nanosec;
            
        }
        cubicSpline spline;
        double max_time = time_from_start[point_num-1];
        RCLCPP_INFO(this->get_logger(), "Second Move_group max_time is %f", max_time);
        time_from_start_.clear();

        // joint1
        if (spline.loadData(time_from_start, p_joint1, point_num, 0, 0, cubicSpline::BoundType_First_Derivative))
        {
            p_joint1_.clear();
            v_joint1_.clear();
            a_joint1_.clear();
            x_out = -rate;
            while(x_out < max_time) {
                x_out += rate;
                spline.getYbyX(x_out, y_out);
                time_from_start_.push_back(x_out);  // 将新的时间存储，只需操作一次即可
                p_joint1_.push_back(y_out);
                v_joint1_.push_back(vel);
                a_joint1_.push_back(acc);
            }

            // joint2
            if (spline.loadData(time_from_start, p_joint2, point_num, 0, 0, cubicSpline::BoundType_First_Derivative))
            {

                p_joint2_.clear();
                v_joint2_.clear();
                a_joint2_.clear();
                x_out = -rate;
                while(x_out < max_time) {
                    x_out += rate;
                    spline.getYbyX(x_out, y_out);
                    p_joint2_.push_back(y_out);
                    v_joint2_.push_back(vel);
                    a_joint2_.push_back(acc);
                }

                // joint3
                if (spline.loadData(time_from_start, p_joint3, point_num, 0, 0, cubicSpline::BoundType_First_Derivative))
                {
                    p_joint3_.clear();
                    v_joint3_.clear();
                    a_joint3_.clear();
                    x_out = -rate;
                    while(x_out < max_time) {
                        x_out += rate;
                        spline.getYbyX(x_out, y_out);
                        p_joint3_.push_back(y_out);
                        v_joint3_.push_back(vel);
                        a_joint3_.push_back(acc);
                    }

                    // joint4
                    if (spline.loadData(time_from_start, p_joint4, point_num, 0, 0, cubicSpline::BoundType_First_Derivative))
                    {
                        p_joint4_.clear();
                        v_joint4_.clear();
                        a_joint4_.clear();
                        x_out = -rate;
                        while(x_out < max_time) {
                            x_out += rate;
                            spline.getYbyX(x_out, y_out);
                            p_joint4_.push_back(y_out);
                            v_joint4_.push_back(vel);
                            a_joint4_.push_back(acc);
                        }

                        // joint5
                        if (spline.loadData(time_from_start, p_joint5, point_num, 0, 0, cubicSpline::BoundType_First_Derivative))
                        {
                            p_joint5_.clear();
                            v_joint5_.clear();
                            a_joint5_.clear();
                            x_out = -rate;
                            while(x_out < max_time) {
                                x_out += rate;
                                spline.getYbyX(x_out, y_out);
                                p_joint5_.push_back(y_out);
                                v_joint5_.push_back(vel);
                                a_joint5_.push_back(acc);
                            }

                            // joint6
                            if (spline.loadData(time_from_start, p_joint6, point_num, 0, 0, cubicSpline::BoundType_First_Derivative))
                            {
                                p_joint6_.clear();
                                v_joint6_.clear();
                                a_joint6_.clear();
                                x_out = -rate;
                                while(x_out < max_time) {
                                    x_out += rate;
                                    spline.getYbyX(x_out, y_out);
                                    p_joint6_.push_back(y_out);
                                    v_joint6_.push_back(vel);
                                    a_joint6_.push_back(acc);
                                }

                                // joint7
                                if(arm_type_ == 75)
                                {
                                    if (spline.loadData(time_from_start, p_joint7, point_num, 0, 0, cubicSpline::BoundType_First_Derivative))
                                    {
                                        p_joint7_.clear();
                                        v_joint7_.clear();
                                        a_joint7_.clear();
                                        x_out = -rate;
                                        while(x_out < max_time) {
                                            x_out += rate;
                                            spline.getYbyX(x_out, y_out);
                                            p_joint7_.push_back(y_out);
                                            v_joint7_.push_back(vel);
                                            a_joint7_.push_back(acc);
                                        }
                                    

                                    p2.vector_len = time_from_start_.size();
                                    p2.vector_cnt = 0;

                                    point_changed = true;
                                    //等待定时器将数据取出并发送完
                                    while(point_changed)
                                    {
                                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                                        if (!rclcpp::ok() || goal_handle->is_canceling())
                                        {
                                            result->error_code = -1;
                                            result->error_string = "has cancel";
                                            goal_handle->canceled(result);
                                            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                                            return;
                                        }
                                    }
                                    }
                                
                                }
                                else
                                {
                                    p2.vector_len = time_from_start_.size();
                                    p2.vector_cnt = 0;

                                    point_changed = true;
                                    //等待定时器将数据取出并发送完
                                    while(point_changed)
                                    {
                                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                                        if (!rclcpp::ok() || goal_handle->is_canceling())
                                        {
                                            result->error_code = -1;
                                            result->error_string = "has cancel";
                                            goal_handle->canceled(result);
                                            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                                            return;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        
        }
    }

    else if (point_num > 0)
    {
        p_joint1_.clear();
        v_joint1_.clear();
        a_joint1_.clear();
        p_joint2_.clear();
        v_joint2_.clear();
        a_joint2_.clear();
        p_joint3_.clear();
        v_joint3_.clear();
        a_joint3_.clear();
        p_joint4_.clear();
        v_joint4_.clear();
        a_joint4_.clear();
        p_joint5_.clear();
        v_joint5_.clear();
        a_joint5_.clear();
        p_joint6_.clear();
        v_joint6_.clear();
        a_joint6_.clear();
        if(arm_type_ == 75)
        {
            p_joint7_.clear();
            v_joint7_.clear();
            a_joint7_.clear();
        }
        for (int i = 0; i < point_num; i++) {
            p_joint1_.push_back(goal->trajectory.points[i].positions[joint_index[0]]);
            v_joint1_.push_back(value_or_zero(goal->trajectory.points[i].velocities, joint_index[0]));
            a_joint1_.push_back(value_or_zero(goal->trajectory.points[i].accelerations, joint_index[0]));
            p_joint2_.push_back(goal->trajectory.points[i].positions[joint_index[1]]);
            v_joint2_.push_back(value_or_zero(goal->trajectory.points[i].velocities, joint_index[1]));
            a_joint2_.push_back(value_or_zero(goal->trajectory.points[i].accelerations, joint_index[1]));
            p_joint3_.push_back(goal->trajectory.points[i].positions[joint_index[2]]);
            v_joint3_.push_back(value_or_zero(goal->trajectory.points[i].velocities, joint_index[2]));
            a_joint3_.push_back(value_or_zero(goal->trajectory.points[i].accelerations, joint_index[2]));
            p_joint4_.push_back(goal->trajectory.points[i].positions[joint_index[3]]);
            v_joint4_.push_back(value_or_zero(goal->trajectory.points[i].velocities, joint_index[3]));
            a_joint4_.push_back(value_or_zero(goal->trajectory.points[i].accelerations, joint_index[3]));
            p_joint5_.push_back(goal->trajectory.points[i].positions[joint_index[4]]);
            v_joint5_.push_back(value_or_zero(goal->trajectory.points[i].velocities, joint_index[4]));
            a_joint5_.push_back(value_or_zero(goal->trajectory.points[i].accelerations, joint_index[4]));
            p_joint6_.push_back(goal->trajectory.points[i].positions[joint_index[5]]);
            v_joint6_.push_back(value_or_zero(goal->trajectory.points[i].velocities, joint_index[5]));
            a_joint6_.push_back(value_or_zero(goal->trajectory.points[i].accelerations, joint_index[5]));
            if(arm_type_ == 75)
            {
                p_joint7_.push_back(goal->trajectory.points[i].positions[joint_index[6]]);
                v_joint7_.push_back(value_or_zero(goal->trajectory.points[i].velocities, joint_index[6]));
                a_joint7_.push_back(value_or_zero(goal->trajectory.points[i].accelerations, joint_index[6]));
            }
        }
        p2.vector_len = point_num;
        p2.vector_cnt = 0;

        point_changed = true;
        //等待定时器将数据取出并发送完
        while(point_changed)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (!rclcpp::ok() || goal_handle->is_canceling())
            {
                result->error_code = -1;
                result->error_string = "has cancel";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                return;
            }
        }
    }

    if (movej_canfd_result_count.load() == 0)
    {
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "rm_driver returned no CANFD command status";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
        return;
    }

    const auto canfd_error = movej_canfd_error.load();
    if (canfd_error != 0)
    {
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "rm_driver rejected a CANFD point; error=" + std::to_string(canfd_error);
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
        return;
    }

    // A zero SDK return value only confirms that the command was accepted by
    // the API.  Verify feedback before reporting action success; otherwise a
    // controller-side protection (for example 4109) is misreported as motion.
    std::vector<double> actual_positions(dof, 0.0);
    bool complete_joint_feedback = true;
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (!joint_state_received_)
        {
            complete_joint_feedback = false;
        }
        else
        {
            for (int joint = 0; joint < dof; ++joint)
            {
                const auto found = std::find(
                    latest_joint_names_.begin(), latest_joint_names_.end(),
                    expected_joint_names[joint]);
                if (found == latest_joint_names_.end())
                {
                    complete_joint_feedback = false;
                    break;
                }
                const size_t feedback_index = static_cast<size_t>(
                    std::distance(latest_joint_names_.begin(), found));
                if (feedback_index >= latest_joint_positions_.size())
                {
                    complete_joint_feedback = false;
                    break;
                }
                actual_positions[joint] = latest_joint_positions_[feedback_index];
            }
        }
    }

    if (!complete_joint_feedback)
    {
        result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "Cannot verify physical motion: incomplete /joint_states feedback";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
        return;
    }

    const auto & final_positions = goal->trajectory.points.back().positions;
    double max_joint_error = 0.0;
    int worst_joint = 0;
    for (int joint = 0; joint < dof; ++joint)
    {
        const double error = std::abs(
            actual_positions[joint] - final_positions[joint_index[joint]]);
        if (error > max_joint_error)
        {
            max_joint_error = error;
            worst_joint = joint;
        }
    }

    constexpr double physical_goal_tolerance = 0.05;  // rad, about 2.9 degrees
    if (max_joint_error > physical_goal_tolerance)
    {
        result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string =
            "Physical robot did not reach the commanded target; " +
            expected_joint_names[worst_joint] + " error=" +
            std::to_string(max_joint_error) + " rad";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "%s", result->error_string.c_str());
        return;
    }

    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    result->error_string = "";

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
}

void Rm_Control::timer_callback()
{
    
    joint_msg.follow = follow_;
    

    if(point_changed)
    {
        if(p2.vector_cnt < p2.vector_len)
        {
            // RCLCPP_INFO(this->get_logger(), "Pos:[%f, %f, %f, %f, %f, %f]",  p_joint1_.at(p2.vector_cnt), p_joint2_.at(p2.vector_cnt), p_joint3_.at(p2.vector_cnt), p_joint4_.at(p2.vector_cnt), p_joint5_.at(p2.vector_cnt), p_joint6_.at(p2.vector_cnt));
            if(arm_type_ == 75)
            {
                joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt);
                joint_msg.joint[1] = p_joint2_.at(p2.vector_cnt);
                joint_msg.joint[2] = p_joint3_.at(p2.vector_cnt);
                joint_msg.joint[3] = p_joint4_.at(p2.vector_cnt);
                joint_msg.joint[4] = p_joint5_.at(p2.vector_cnt);
                joint_msg.joint[5] = p_joint6_.at(p2.vector_cnt);
                joint_msg.joint[6] = p_joint7_.at(p2.vector_cnt);
                this->joint_pos_publisher->publish(joint_msg);
            }
            else
            {
                joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt);
                joint_msg.joint[1] = p_joint2_.at(p2.vector_cnt);
                joint_msg.joint[2] = p_joint3_.at(p2.vector_cnt);
                joint_msg.joint[3] = p_joint4_.at(p2.vector_cnt);
                joint_msg.joint[4] = p_joint5_.at(p2.vector_cnt);
                joint_msg.joint[5] = p_joint6_.at(p2.vector_cnt);
                this->joint_pos_publisher->publish(joint_msg);
            }
            
            p2.vector_cnt++;
        }
        else
        {
            if(count_final_joint <= count_keep_send)
            {
                if(arm_type_ == 75)
                {
                    joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt-1);
                    joint_msg.joint[1] = p_joint2_.at(p2.vector_cnt-1);
                    joint_msg.joint[2] = p_joint3_.at(p2.vector_cnt-1);
                    joint_msg.joint[3] = p_joint4_.at(p2.vector_cnt-1);
                    joint_msg.joint[4] = p_joint5_.at(p2.vector_cnt-1);
                    joint_msg.joint[5] = p_joint6_.at(p2.vector_cnt-1);
                    joint_msg.joint[6] = p_joint7_.at(p2.vector_cnt-1);
                    this->joint_pos_publisher->publish(joint_msg);
                }
                else
                {
                    joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt-1);
                    joint_msg.joint[1] = p_joint2_.at(p2.vector_cnt-1);
                    joint_msg.joint[2] = p_joint3_.at(p2.vector_cnt-1);
                    joint_msg.joint[3] = p_joint4_.at(p2.vector_cnt-1);
                    joint_msg.joint[4] = p_joint5_.at(p2.vector_cnt-1);
                    joint_msg.joint[5] = p_joint6_.at(p2.vector_cnt-1);
                    this->joint_pos_publisher->publish(joint_msg);
                }
                count_final_joint++;
            }
            else
            {
                count_final_joint = 0;
                p2.vector_cnt = 0;
                p2.vector_len = 0;
                point_changed = false;
            }
        }
    }
}

void Rm_Control::get_move_stop_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    // bool result;
    // result = msg->data;
    point_changed=false;
    //RCLCPP_INFO(this->get_logger(), "move stop is true!!! ");
}

void Rm_Control::movej_canfd_result_callback(const std_msgs::msg::UInt32::SharedPtr msg)
{
    movej_canfd_result_count.fetch_add(1);
    if (msg->data != 0)
    {
        uint32_t expected = 0;
        movej_canfd_error.compare_exchange_strong(expected, msg->data);
    }
}

void Rm_Control::joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_names_ = msg->name;
    latest_joint_positions_ = msg->position;
    joint_state_received_ = true;
}
/* 主函数主要用于动作订阅和套接字通信 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Keep transmitting the final point for the configured settling time.
  count_keep_send = static_cast<int>(wait_move_finish_time / rate);
  rclcpp::spin(std::make_shared<Rm_Control>("rm_control"));
  rclcpp::shutdown();
  return 0;
}
