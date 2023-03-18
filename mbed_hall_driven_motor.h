#include "mbed_PWMServoDriver.h"
#include "mbed.h"
#include "named_type.hpp"
#include "PID_v1.h"
#include "PinNames.h"
// #include "MotorConfig/MotorConfig.h"

/*!
 *  @brief classe pour gérer un moteur asservis par un compteur à effet Hall  

 */
class mbed_hall_driven_motor
{
public:
  mbed_hall_driven_motor(int & count,
                    Stop_pin stop_pin,
                    mbed_PWMServoDriver &pwm,
                    Forward_or_dir_pin forward_or_dir_pin,
                    Backward_or_speed_pin backward_or_speed_pin,
                    Motor_name motor_name,
                    Motor_shield_type motor_shield_type,
                    Flag_start flag_start,
                    Flag_stop flag_stop,
                    Init_speed init_speed,
                    Min_speed min_speed,
                    Max_speed max_speed,
                    Coef_Kp coef_Kp,
                    Coef_Ki coef_Ki,
                    Coef_Kd coef_Kd,
                    Nb_tic_per_deg nb_tic_per_deg ,
                    End_stop_type end_stop_type,
                    bool reverse_rotation);

 
  // methodes
  void run();
  void init();
  void set_speed_sync(mbed_hall_driven_motor *pSynchronised_motor);
  double get_angle();  
  //variables
  int32_t _flag_start;
  int32_t _flag_stop;
  string _motor_name;
  double _deplacement;
  double angle;

  double _start_angle;
  double _target;
  bool _debug_flag;
  bool _reverse_rotation;
  double _debug_count_when_stoped; 
  int  *_count;

private:

  DigitalIn _DigitalIn_stop;
  mbed_PWMServoDriver *_pwm;

  int _backward_pin;
  int _forward_pin;
  int _dir_pin;
  int _pwm_pin;
  bool _sens;
  int tic_forward=0;
  int tic_backward=0;
  
  
  bool _flag_is_running;
  Timer   timer;
  int  previous_time; 
  double _flag_sens;
  bool _flag_speed_sync;
  int Nb_Motor_sync = 0;
 

  double previous_speed;

  double _nb_tic_per_deg;
  int _init_speed;
  double Input;
  double Output;
  double Setpoint;

  int _max_speed;
  int _min_speed;
  int _end_stop_type;

  // int _cmde_flag_start;
  // int _cmde_flag_stop;
  int _motor_shield_type;
  void motor_run_forward(double speed);
  void motor_run_backward(double speed);
  void motor_stop();
  int get_speed(double target);
  double get_speed_coef(double pTarget); 
  PID _PID;

  mbed_hall_driven_motor *synchronised_motor_list[10];
  
 
  
};