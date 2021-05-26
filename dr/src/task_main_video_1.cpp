#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <string>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <thread>     
#include <chrono>


constexpr double pi = 3.141592653589793238462643383279502884L;

namespace dr{


enum class ControllerCommands : uint16_t
{
// change mode
    manual, 
    shutdown, // shutdown
    recover_current,
    recover_velocity, 
    recover_position,
    arm_home,
    clear_flag,
// collect the arrow

    stand_grab_arrow,
    stand_release_arrow,
    stand_lift_arrow,
    stand_lower_arrow,

    arm_rotate,
    arm_rotate_to_grab_arrow,
    arm_rotate_to_grab_table,
    arm_rotate_to_load,
    adjust_arm_to_launch,
    arm_grab,
    arm_release,
// launch_and_home the arrow
    launch_start,
    launch_and_home,
    launch_short_start,
    launch_medium_start,
    launch_long_start,
// related to delay
    set_delay_250ms,
    set_delay_500ms,
    set_delay_1s,
    delay,
    wait_next_pressed,
};

enum class SolenoidValveCommands : uint8_t
{
    shutdown_cmd      = 0b000000,
    recover_cmd       = 0b000001,
    
    stand_release_arrow_cmd  = 0b000010,//default close
    lift_arrow_cmd           = 0b010000,//default down
    spread_palm_cmd          = 0b100000,//default grab
};

enum class MotorCommands : uint8_t
{
    shutdown_cmd      = 0x00,
    recover_cmd       = 0x01,
    homing_cmd        = 0x02,
    get_status        = 0x03,
    recover_current   = 0x04,
	recover_velocity  = 0x05,
	recover_position  = 0x06,
    
};

enum class OpMode : uint8_t
{
    def,         
    full_op,   
};



class dr_nodelet_main : public nodelet::Nodelet
{
public:
    virtual void onInit();
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void control_timer_callback(const ros::TimerEvent &event);
    void reset_launcher();
    
    
    void reset_launcher_status();
    void arm_grab_arrow();
    void arm_release_arrow();

    void launch_ready();
    void adjust_arm_vel(float rotate_veloccity);
    void adjust_arm_pos(float rotate_position);
    void rotate_arm_to_grab_arrow();
    void rotate_arm_to_grab_table();
    void rotate_arm_to_load_arrow();

    void stand_grab_arrow();
    void stand_release_arrow();
    void stand_lift_arrow();
    void stand_lower_arrow();


    void next_OpMode();
    void back_OpMode();

    void shutdown();
    void recover();
    void homing();
    void clear_flags();
    void set_delay(double delay_s);

    void change_OpMode();

    ros::NodeHandle nh;
  	ros::NodeHandle _nh;
    ros::NodeHandle nh_MT;

    //int linear_, angular_;
    ros::Subscriber joy_sub;

	/***********************/
	ros::Subscriber ThrowPos_sub;
	/***********************/

    ros::Publisher ArmCmd_pub;
    ros::Publisher ArmVal_pub;
    std_msgs::Float64 arm_vel_msg;
    std_msgs::Float64 arm_position_msg;

	/***********************/

    ros::Publisher act_enable_pub0;
    ros::Publisher act_enable_pub1;
    ros::Publisher act_enable_pub2;
	ros::Publisher act_enable_pub3;
	/**********************/
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;
	/**********************/
    ros::Publisher SolenoidCmd_pub;
    ros::Publisher SolenoidOrder_pub;
	std_msgs::UInt8 solenoid_order_msg;
    uint8_t lastSolenoidOrder = 0b0000000;
	/**********************/
    std_msgs::UInt8 act_conf_cmd_msg;

	/**********************/
	std_msgs::UInt8 shirasu_cmd_msg;
	/**********************/
    ros::Timer control_timer;

    int _delay_s = 0;

    double launch_long_vel;
    double launch_medium_vel;
    double launch_short_vel;

    double launch_long_pos;
    double launch_medium_pos;
    double launch_short_pos;

	double throw_position_observed = 0;

	int dr_mode = 0;
	// 0 : taiki_mode
	// 1 : genten_awase_mode
	// 2 : haji_mode
	// 3 : tohteki_mode
	// 4 : defence_mode

	/************************/
	/************************/

    //		{0, -40 * steps_per_mm;
    //static constexpr int lift_position_first = -40 * steps_per_mm;
    //static constexpr int lift_position_second = lift_position_first - (248 * steps_per_mm);
    //static constexpr int lift_position_third = lift_position_second - (248 * steps_per_mm);
    bool _command_ongoing = false;
    bool _has_loaded = false;
    bool _initial_pose_finished = false;
    bool _launch_angle_reached = false;
    bool _rotating_to_launch = false;
    bool _is_manual_enabled = true;

    bool _a = false;
    bool _b = false;
    bool _x = false;
    bool _y = false;
    bool _start = false;
    bool _back  = false;
    bool _rightthumb = false;
    bool _leftthumb = false;
    bool _righttrigger = false;
    bool _lefttrigger = false;

    static int _padx;
    static int _pady;
    static int _lb;
    static int _rb;

    static int ButtonA;
    static int ButtonB;
    static int ButtonX;
    static int ButtonY;
    static int ButtonLB;
    static int ButtonRB;
    static int ButtonStart;
    static int ButtonBack;
	/***************/
	static int ButtonLeftThumb;
    static int ButtonRightThumb;
	/***************/

    static int AxisDPadX;
    static int AxisDPadY;
    static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
    static int AxisRightThumbY;
    static int ButtonLeftTrigger;
    static int ButtonRightTrigger;

    int currentCommandIndex = 0;

    static const std::vector<OpMode> opmode;
    static const std::vector<ControllerCommands> launch_short_test_commands;
    static const std::vector<ControllerCommands> launch_medium_test_commands;
    static const std::vector<ControllerCommands> launch_long_test_commands;
    static const std::vector<ControllerCommands> load_test_commands;
    static const std::vector<ControllerCommands> initial_pose;
    static const std::vector<ControllerCommands> manual_all;
    const std::vector<ControllerCommands> *command_list;
};

int dr_nodelet_main::_padx = 0;
int dr_nodelet_main::_pady = 0;
int dr_nodelet_main::_lb = 0;
int dr_nodelet_main::_rb = 0;

int dr_nodelet_main::ButtonA = 1;
int dr_nodelet_main::ButtonB = 2;
int dr_nodelet_main::ButtonX = 0;
int dr_nodelet_main::ButtonY = 3;
int dr_nodelet_main::ButtonLB = 4;
int dr_nodelet_main::ButtonRB = 5;
int dr_nodelet_main::ButtonBack = 8;
int dr_nodelet_main::ButtonStart = 9;
int dr_nodelet_main::ButtonLeftThumb = 6;
int dr_nodelet_main::ButtonRightThumb = 7;

int dr_nodelet_main::AxisDPadX = 4;
int dr_nodelet_main::AxisDPadY = 5;
int dr_nodelet_main::AxisLeftThumbX = 0;
int dr_nodelet_main::AxisLeftThumbY = 1;
int dr_nodelet_main::AxisRightThumbX = 2;
int dr_nodelet_main::AxisRightThumbY = 3;
int dr_nodelet_main::ButtonLeftTrigger = 10;
int dr_nodelet_main::ButtonRightTrigger = 11;

const std::vector<ControllerCommands> dr_nodelet_main::launch_short_test_commands(
    {
        ControllerCommands::recover_velocity,
        ControllerCommands::launch_short_start,
        ControllerCommands::launch_and_home,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::launch_medium_test_commands(
    {
        ControllerCommands::recover_velocity,
        ControllerCommands::launch_medium_start,
        ControllerCommands::launch_and_home,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        
    }
);
const std::vector<ControllerCommands> dr_nodelet_main::launch_long_test_commands(
    {
        ControllerCommands::recover_velocity,
        ControllerCommands::launch_long_start,
        ControllerCommands::launch_and_home,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::load_test_commands(
    {
        ControllerCommands::arm_home,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::stand_grab_arrow,
        ControllerCommands::arm_release,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::stand_lift_arrow,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::arm_grab,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::stand_release_arrow,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::recover_position,
        ControllerCommands::arm_rotate_to_load,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::stand_lower_arrow,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::initial_pose(

    {
        ControllerCommands::recover_position,
        ControllerCommands::stand_release_arrow,
        ControllerCommands::arm_release,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::stand_lower_arrow,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::arm_home,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
    } 

);

const std::vector<ControllerCommands> dr_nodelet_main::manual_all(
    {
        ControllerCommands::manual
    }
);

void dr_nodelet_main::onInit(void)
{
    nh = getNodeHandle();
    nh_MT = getMTNodeHandle();
    _nh = getPrivateNodeHandle();

    this->joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &dr_nodelet_main::joyCallback, this);

	/***************************************/
	this->ThrowPos_sub = nh_MT.subscribe<std_msgs::Float32>("motor4_current_val", 10, &dr_nodelet_main::PosCallback, this);
	/***************************************/

    this->ArmVal_pub = nh.advertise<std_msgs::Float64>("arm_val", 1);
    this->ArmCmd_pub = nh.advertise<std_msgs::UInt8>("arm_cmd", 1);

	/**************************************************/
	this->SolenoidCmd_pub = nh.advertise<std_msgs::UInt8>("solenoid_cmd", 1);
    this->SolenoidOrder_pub = nh.advertise<std_msgs::UInt8>("solenoid_order", 1);
	/**************************************************/
    this->cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    this->act_enable_pub0 = nh.advertise<std_msgs::UInt8>("foot0_cmd", 1);
    this->act_enable_pub1 = nh.advertise<std_msgs::UInt8>("foot1_cmd", 1);
    this->act_enable_pub2 = nh.advertise<std_msgs::UInt8>("foot2_cmd", 1);
	/**************************************************/
	this->act_enable_pub3 = nh.advertise<std_msgs::UInt8>("foot3_cmd", 1);
	/**************************************************/
    //this->hand_unchuck_thres_pub = _nh.advertise<std_msgs::UInt16>("hand/unchuck_thres", 1);


    /*nh_priv.getParam("lift_step_per_mm", this->steps_per_mm);

    std::vector<double> tmp;
    nh_priv.getParam("pick_position", tmp);
    if (tmp.size() == 5)
    {
        this->pick_position = tmp;
    }

    for (double& pos : this->pick_position)
    {
        pos *= (-steps_per_mm);
    }

    ROS_INFO("pick_pos: %f, %f, %f, %f, %f", this->pick_position[0], this->pick_position[1], this->pick_position[2],
            this->pick_position[3], this->pick_position[4]);*/

	/************************************************************
	std::vector<int> tmp2;
    nh_priv.getParam("solenoid_position", tmp2);
    if (tmp2.size() == 5)
    {
        this->solenoid_position = tmp2;
    }

    for (int& pos : this->solenoid_position)
    {
        pos *= (-steps_per_mm);
    }

    ROS_INFO("solenoid_pos: %d, %d, %d, %d, %d", this->solenoid_position[0], this->solenoid_position[1], this->solenoid_position[2],
            this->solenoid_position[3], this->solenoid_position[4]);
	************************************************************/
    _nh.param("launch_long_vel", launch_long_vel, 0.0);
    _nh.param("launch_medium_vel", launch_medium_vel, 0.0);
    _nh.param("launch_short_vel", launch_short_vel, 0.0);

    _nh.param("launch_long_pos", launch_long_pos, 0.0);
    _nh.param("launch_medium_pos", launch_medium_pos, 0.0);
    _nh.param("launch_short_pos", launch_short_pos, 0.0);

    nh.getParam("ButtonA", ButtonA);
    nh.getParam("ButtonB", ButtonB);
    nh.getParam("ButtonX", ButtonX);
    nh.getParam("ButtonY", ButtonY);
    nh.getParam("ButtonLB", ButtonLB);
    nh.getParam("ButtonRB", ButtonRB);
    nh.getParam("ButtonStart", ButtonStart);
    nh.getParam("ButtonLeftThumb", ButtonLeftThumb);
    nh.getParam("ButtonRightThumb", ButtonRightThumb);
	/***************************/
	nh.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh.getParam("AxisRightThumbX", AxisRightThumbX);
    nh.getParam("AxisRightThumbY", AxisRightThumbY);
    nh.getParam("AxisDPadX", AxisDPadX);
    nh.getParam("AxisDPadY", AxisDPadY);

    this->control_timer = nh.createTimer(ros::Duration(0.05), &dr_nodelet_main::control_timer_callback, this);
    NODELET_INFO("dr node has started.");

    this->command_list = &dr_nodelet_main::manual_all;

}

/**************************************************************************************/
void dr_nodelet_main::PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->throw_position_observed = msg->data;
}

void dr_nodelet_main::arm_grab_arrow(void){
    this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::spread_palm_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void dr_nodelet_main::arm_release_arrow(void){
    this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::spread_palm_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void dr_nodelet_main::rotate_arm_to_grab_table(void){
    this->arm_position_msg.data = -pi/4;
    this->ArmVal_pub.publish(arm_position_msg);
}
void dr_nodelet_main::rotate_arm_to_load_arrow(void){
    this->arm_position_msg.data = -pi/2;
    this->ArmVal_pub.publish(arm_position_msg);
}

//minas is load direction

void dr_nodelet_main::stand_grab_arrow(void)
{
    this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::stand_release_arrow_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void dr_nodelet_main::stand_release_arrow(void)
{
    this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::stand_release_arrow_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void dr_nodelet_main::stand_lift_arrow(void)
{
    this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::lift_arrow_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}
void dr_nodelet_main::stand_lower_arrow(void)
{
    this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::lift_arrow_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}
/**************************************************************************************/
void dr_nodelet_main::shutdown(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    ArmCmd_pub.publish(act_conf_cmd_msg);
    act_enable_pub0.publish(act_conf_cmd_msg);
    act_enable_pub1.publish(act_conf_cmd_msg);
    act_enable_pub2.publish(act_conf_cmd_msg);
    act_enable_pub3.publish(act_conf_cmd_msg);
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
}



void dr_nodelet_main::recover(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
    act_enable_pub0.publish(act_conf_cmd_msg);
    act_enable_pub1.publish(act_conf_cmd_msg);
    act_enable_pub2.publish(act_conf_cmd_msg);
    act_enable_pub3.publish(act_conf_cmd_msg);
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
    ArmCmd_pub.publish(act_conf_cmd_msg);
}

void dr_nodelet_main::homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    ArmCmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    ArmCmd_pub.publish(act_conf_cmd_msg);
}

void dr_nodelet_main::set_delay(double delay_s)
{
    this->_delay_s = ros::Time::now().toSec() + delay_s;
}

void dr_nodelet_main::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    static bool last_start;
    static bool last_back;
    static bool last_x;
    static bool last_y;
    static bool last_rb;
    static bool last_lb;
    static bool last_padx;
    static bool last_pady;
	/******/
	static bool last_rightthumb;
    static bool last_leftthumb;
	/******/
    static bool last_righttrigger;
    static bool last_leftrigger;
    //static int last_dpadXCmd = 0;

    this->_a = joy->buttons[ButtonA];
    this->_b = joy->buttons[ButtonB];
    this->_x = joy->buttons[ButtonX];
    this->_y = joy->buttons[ButtonY];
    this->_lb = joy->buttons[ButtonLB];
    this->_rb = joy->buttons[ButtonRB];
    this->_padx = joy->axes[AxisDPadX];
    this->_pady = joy->axes[AxisDPadY];
    this->_rightthumb = joy->buttons[ButtonRightThumb];
    this->_leftthumb = joy->buttons[ButtonLeftThumb];
    this->_righttrigger = joy->buttons[ButtonRightTrigger];
    this->_lefttrigger = joy->buttons[ButtonLeftTrigger];
    

    this->_start = joy->buttons[ButtonStart];
    this->_back  = joy->buttons[ButtonBack];

    //std::vector<double> throw_pos_fixed = { 0+this->throw_position_observed, 2*pi+this->throw_position_observed, -2*pi+this->throw_position_observed };

   if (_start)
    {
        this->recover();
    }
    if (_back)
    {
        this->shutdown();
    }
    if(!this->_command_ongoing)
    {
        if(_initial_pose_finished){

            if(!this->_has_loaded)
            {
                if (_y)
                {
                    this->command_list = &load_test_commands;
                    _command_ongoing = true;
                    _has_loaded = true;
                }
            }
            else if (_a&&(_padx == 1))
            {      
                this->command_list = &launch_short_test_commands;
                _command_ongoing = true;
                _has_loaded = false;
            }
            else if (_a&&(_pady == 1))
            {      
                this->command_list = &launch_medium_test_commands;
                _command_ongoing = true;
                _has_loaded = false;
            }
            else if (_a&&(_pady == -1))
            {      
                this->command_list = &launch_long_test_commands;
                _command_ongoing = true;
                _has_loaded = false;
            }
        }
        if (_b)
        {            
                this->homing();
                NODELET_INFO("homing");
        }
        if (_righttrigger)
        {
            this->command_list = &initial_pose;
            _command_ongoing = true;
            _initial_pose_finished = true;
        }
    }
    if (this->_is_manual_enabled)
    {
        double vel_x = joy->axes[AxisRightThumbX];   
        double vel_y = joy->axes[AxisRightThumbY];
        double vel_yaw_l = (joy->buttons[ButtonLeftThumb] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw_r = (joy->buttons[ButtonRightThumb] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw = vel_yaw_l + vel_yaw_r;
        double vel_norm = hypot(vel_x, vel_y);


        if (vel_norm > 1.0)
        {
            vel_x /= vel_norm;
            vel_y /= vel_norm;
        }
        this->cmd_vel_msg.linear.x = -vel_x;
        this->cmd_vel_msg.linear.y = vel_y;
        this->cmd_vel_msg.angular.z = vel_yaw;
        this->cmd_vel_pub.publish(this->cmd_vel_msg);
    }
    last_start = _start;
    last_back = _back;
    last_x = _x;
    last_y = _y;
    last_rb = _rb;
    last_lb = _lb;
    last_padx = _padx;
    last_pady = _pady;
	/******/
	last_rightthumb = _rightthumb;
    last_leftthumb = _leftthumb;
	/******/
    last_righttrigger= _righttrigger;
    last_leftrigger = _lefttrigger;

	
}

void dr_nodelet_main::control_timer_callback(const ros::TimerEvent &event)
{ 
    //this->command_list->size() <= (int)this->currentCommandIndex || this->command_list == &this->manual_all
    if (!this->_command_ongoing)
    {
        NODELET_INFO("control_time_return");
        
    }

    ControllerCommands currentCommand = this->command_list->at(this->currentCommandIndex);

    if(currentCommand == ControllerCommands::arm_home)
    {
        this->homing();
        this->currentCommandIndex++;
        NODELET_INFO("home");
    }
    else if(currentCommand == ControllerCommands::recover_current)
    {
        this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_current;
        this->ArmCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
        NODELET_INFO("home");
    }
    else if(currentCommand == ControllerCommands::recover_velocity)
    {
        this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
        this->ArmCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
        NODELET_INFO("velocity");
    }
    else if(currentCommand == ControllerCommands::recover_position)
    {
        this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
        this->ArmCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
        NODELET_INFO("position");
    }
    else if(currentCommand == ControllerCommands::stand_grab_arrow)
    {   
        this->stand_grab_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("stand_grab_arrow");
    }
    else if(currentCommand == ControllerCommands::stand_release_arrow)
    {   
        this->stand_release_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("stand_release_arrow");
    }
    else if(currentCommand == ControllerCommands::stand_lift_arrow)
    { 
        this->stand_lift_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("stand_lift_arrow");
    }
    else if(currentCommand == ControllerCommands::stand_lower_arrow)
    {  
        this->stand_lower_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("stand_lower_arrow");
    }
    else if(currentCommand == ControllerCommands::arm_rotate_to_grab_arrow)
    { 
        this->rotate_arm_to_grab_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_grab");
    }
    else if(currentCommand == ControllerCommands::arm_rotate_to_grab_table)
    {   
        this->rotate_arm_to_grab_table(); 
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_set");
    }
    else if(currentCommand == ControllerCommands::arm_rotate_to_load)
    {   
        this->rotate_arm_to_load_arrow(); 
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_set");
    }
    else if(currentCommand == ControllerCommands::launch_short_start)
    {
        this->arm_vel_msg.data = this->launch_short_vel;
        this->ArmVal_pub.publish(arm_vel_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_velocity_short");
    }
    else if(currentCommand == ControllerCommands::launch_medium_start)
    {
        this->arm_vel_msg.data = this->launch_medium_vel;
        this->ArmVal_pub.publish(arm_vel_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_velocity_medium");
    }
    else if(currentCommand == ControllerCommands::launch_long_start)
    {
        this->arm_vel_msg.data = this->launch_long_vel;
        this->ArmVal_pub.publish(arm_vel_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_velocity_long");
    }
    else if(currentCommand == ControllerCommands::launch_and_home)
    {
        while (!this->_launch_angle_reached)
        {
            if(this->arm_vel_msg.data == this->launch_long_vel && this->throw_position_observed <= this->launch_long_pos)
            {
                this->_launch_angle_reached = true;
            }
            else if(this->arm_vel_msg.data == this->launch_medium_vel && this->throw_position_observed <= this->launch_medium_pos)
            {
                this->_launch_angle_reached = true;
            }
            else if(this->arm_vel_msg.data == this->launch_short_vel && this->throw_position_observed <= this->launch_short_pos)
            {
                this->_launch_angle_reached = true;
            }
            NODELET_INFO("%f",this->throw_position_observed);
        }
        this->arm_release_arrow();
        std::this_thread::sleep_for(std::chrono::milliseconds(900));
        this->homing();
        this->currentCommandIndex++;
        NODELET_INFO("launch_and_home");
        this->_launch_angle_reached = false;
    }
    else if(currentCommand == ControllerCommands::arm_grab)
    {   
        this->arm_grab_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("grab_arrow");
    }
    else if(currentCommand == ControllerCommands::arm_release)
    {   
        this->arm_release_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("release_arrow");
    }
    else if (currentCommand == ControllerCommands::set_delay_250ms)
    {
        set_delay(0.250);
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_250ms");
    }
    else if (currentCommand == ControllerCommands::set_delay_500ms)
    {
        set_delay(0.500);
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_500ms");
    }
    else if (currentCommand == ControllerCommands::set_delay_1s)
    {
        set_delay(1.000);
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_1s");
    }
    else if (currentCommand == ControllerCommands::delay)
    {
        if (this->_delay_s == 0)
        {
            return;
        }

        if (this->_delay_s < ros::Time::now().toSec())
        {
            this->_delay_s = 0;
            this->currentCommandIndex++;
        }
    }
    
    if(this->command_list->size() <= (int)this->currentCommandIndex)
    {
        this->_command_ongoing = false;
        NODELET_INFO("command_list_finish");
        this->command_list = &this->manual_all;
        currentCommandIndex = 0;
    }


}



}
PLUGINLIB_EXPORT_CLASS(dr::dr_nodelet_main, nodelet::Nodelet);
