#include "mbed_hall_driven_motor.h"

/*!
 *  @brief constructeur
 *  @param count_pin
 *  @param stop_pin
 *  @param pwm
 *  @param forward_pin
 *  @param backward_pin
 *  @param &target
 *  @param &angle
 *  @param motor_name
 *  @param Motor_shield_type
 *  @param flag_start
 *  @param flag_stop
 *  @param Init_speed
 *  @param
   PullNone          = 0,
    PullUp            = 1,
    PullDown          = 2,
 */
mbed_hall_driven_motor::mbed_hall_driven_motor(Count_pin count_pin,
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
                                               Nb_tic_per_deg nb_tic_per_deg)
    : _interrupt_count(count_pin.get(), PullDown),
      _interrupt_stop(stop_pin.get(), PullDown),
      _pwm(&pwm),
      _PID(&Input, &Output, &Setpoint, coef_Kp.get(), coef_Ki.get(), coef_Kd.get(), P_ON_E, DIRECT)

{

  // create the InterruptIn on the pin specified to Counter
  _interrupt_count.fall(callback(
      this, &mbed_hall_driven_motor::increment)); // attach increment function of
                                                  // this counter instance
  _interrupt_count.rise(callback(
      this, &mbed_hall_driven_motor::increment)); // attach increment function of
                                                  // this counter instance
  _motor_shield_type = motor_shield_type.get();
  if (_motor_shield_type == 1)
  {
    _dir_pin = forward_or_dir_pin.get();
    _pwm_pin = backward_or_speed_pin.get();
  }
  if (_motor_shield_type == 2)
  {
    _forward_pin = forward_or_dir_pin.get();
    _backward_pin = backward_or_speed_pin.get();
  }
  //   default value
  _motor_name = motor_name.get();
  _init_speed = init_speed.get();
  _min_speed = min_speed.get();
  _max_speed = max_speed.get();
  _nb_tic_per_deg = nb_tic_per_deg.get();

  _flag_start = flag_start.get();
  _flag_stop = flag_stop.get();

  // définit la valeur par défaut
  _debug_flag = false;
}

//****************** interruptions
void mbed_hall_driven_motor::increment()
{
  if (_sens)
  {
    _count++;
  }
  else
  {
    _count--;
  };
  // met à jour l'angle du moteur
  _angle = _count / _nb_tic_per_deg;
}

//********************** methodes publiques

void mbed_hall_driven_motor::init()
{
  printf("init %s\n", _motor_name.c_str());
  // _interrupt_stop.enable_irq(); // --> on allume la lecture de la butée
  // _interrupt_stop.read() == 1 --> en butée
  // on fait tourner le moteur jusqu'a la butée
  if (_debug_flag)
  {
    printf("run_forward %s interrupt: %i\n", _motor_name.c_str() ,_interrupt_stop.read());
  };
  while (_interrupt_stop.read() == 1)
  {
    motor_run_forward(_init_speed);
  }
  motor_stop();
  if (_debug_flag)
  {
    printf("stop %s : on attends une seconde pour stabiliser le moteur, interrupt: %i \n ", _motor_name.c_str(),_interrupt_stop.read());
  };                                                 // on arrete le moteur
  ThisThread::sleep_for(chrono::milliseconds(1000)); // on attend une seconde pour stabiliser le moteur

  // on repart tout doucement pour que l'init soit juste après la butée
  if (_debug_flag)
  {
    printf("run_backward %s : on repart tout doucement pour que l'init soit juste après la butée \n ", _motor_name.c_str());
  };
  while (_interrupt_stop.read() == 0)
  {
    motor_run_backward(_init_speed / 2);
  }
  motor_stop(); // on arrete le moteur
  // _interrupt_stop.disable_irq(); // --> on eteint la lecture de la butée

  // on initialise les variables
  previous_speed = 0;

  _flag_speed_sync = false;
  _count = 0;
  _angle = 0;
  _sens = true;
  _PID.SetOutputLimits(_min_speed, _max_speed);
  _PID.SetMode(1);

  // log
  printf("fin init %s\n", _motor_name.c_str());
}

void mbed_hall_driven_motor::set_speed_sync(mbed_hall_driven_motor *pSynchronised_motor)
{
  _flag_speed_sync = true;

  //  synchronised_motor_list = *pSynchronised_motor_list ;
  Nb_Motor_sync++;
  if (_debug_flag)
  {
    printf("Nb_Motor_sync = %i \n", Nb_Motor_sync);
  }
  synchronised_motor_list[Nb_Motor_sync - 1] = pSynchronised_motor;
};

void mbed_hall_driven_motor::run()
{

  _deplacement = _target - _angle; // au demarrage on calcul le deplacement pour la synchro
  _start_angle = _angle;           // on enregistre la position des moteurs liés au demarrage
  previous_speed = 0;

  if (_debug_flag)
  {
    printf("start run \n");
    printf("deplacement: %f\n", _deplacement);
    printf("start_angle: %f\n", _start_angle);
  };

  double target_count = _target * _nb_tic_per_deg; // calcul la target en nombre de tic

  if ((target_count - _count) > 0)
  {
    // on recule
    while (target_count > _count)
    {
      // calcul de la vitesse à chaque tour
      int speed = get_speed(target_count);
      if (_debug_flag)
      {
        printf("   backward\n");
      }
      _sens = true; // true = forward / false = backward
      motor_run_backward(speed);
    }
  }
  else
  {
    // on avance
    while (target_count < _count)
    {
      // calcul de la vitesse à chaque tour
      int speed = get_speed(target_count);
      if (_debug_flag)
      {
        printf("   forward\n");
      }
      _sens = false; // true = forward / false = backward
      motor_run_forward(speed);
    }
  };
  // stop quand le compteur est arrivé
  motor_stop();
  if (_debug_flag)
  {
    printf("fin target moteur : angle %f\n", _angle);
  }
}
//********************** methodes privées

int mbed_hall_driven_motor::get_speed(double target)
{

  // calcul de la vitesse avec le PID
  Setpoint = 0;                  // SetPoint pour le PID
  Input = -abs(target - _count); // Input pour le PID
  double speed;
  // le PID demarre à 1500 de la target.
  if (abs(target - _count) < 1500)
  {
    _PID.Compute(); // le PID calcule la vitesse et commence a prendre en compte les valeurs pour l'intégrale
    speed = Output;
  }
  else
  {
    _PID.Compute(true); // on fait calculer le PID en reinitialisant la période de temp
    speed = Output;     // le PID calcule la vitesse et sans a prendre en compte le temp pour l'intégrale
  }

  double speed_coef = 1;

  if (_flag_speed_sync)
  {
    speed_coef = get_speed_coef(target);
  }

  // on applique le coeficient le plus petit pour la synchro avec la vitesse

  speed = speed * speed_coef;

  // debug
  if (_debug_flag)
  {
    printf("get_speed target %i\t", (int)(target));
    printf("speed %i\t", (int)(speed));
    printf("count %i\t", (int)(_count));
    printf("angle %f\t", (_angle));
    printf("speed_coef %f\t", (speed_coef));
  }

  // pour ne pas demarrer trop vite, on n'augmente pas la vitesse de plus de 100 par cycle

  if (speed > previous_speed + 100)
  {
    speed = previous_speed + 100;
  }

  previous_speed = speed;

  return (int)speed;
}

double mbed_hall_driven_motor::get_speed_coef(double pTarget)
{

  // calcul du coeficient de vitesse pour la synchro
  // on ajuste la vitesse pour que l'erreur ne depasse pas 2deg
  // coef = pourcentage de l'erreur d'angle
  //          2 deg -> 0%
  //          1 deg -> 50%
  //          0 deg -> 100%
  // coef = (Nb_deg_autorisé - erreur d'angle) / Nb_deg_autorisé
  // à 2 degrès on stop le moteur
  double speed_coef_final = 1;
  double speed_coef = 1;
  double Nb_deg_autorise = 1;

  double move_angle_linked_motor;
  double move_angle_current_motor;
  double coef_angle;
  double err_angle;

  for (int i = 0; i < Nb_Motor_sync; i++)
  {
    if (_debug_flag)
    {
      printf("%s : ", synchronised_motor_list[i]->_motor_name.c_str());
    };
    if (synchronised_motor_list[i]->_motor_name != _motor_name && _deplacement != 0 && synchronised_motor_list[i]->_deplacement != 0)
    {
      // pour la synchro, on doit toujours avoir
      // on doit avoir (move_angle_current_motor)  = (move_angle_linked_motor )*deplacement/linked_deplacement
      // err_angle = (move_angle_current_motor)  - (move_angle_linked_motor )*deplacement/linked_deplacement
      coef_angle = _deplacement / (synchronised_motor_list[i]->_deplacement);
      move_angle_linked_motor = synchronised_motor_list[i]->_angle - synchronised_motor_list[i]->_start_angle;
      move_angle_current_motor = _angle - _start_angle;
      err_angle = move_angle_current_motor - ((move_angle_linked_motor * coef_angle));

      if (_debug_flag)
      {
        printf(" move_angle_linked_motor = %f ", move_angle_linked_motor);
        printf(" move_angle_current_motor = %f ", move_angle_current_motor);
        printf(" coef_angle = %f  ", coef_angle);
        printf(" err_angle = %f  ", err_angle);
      };

      // backward (pTarget > _count)  => quand l'angle augmente et que l'angle est plus petit que l'angle lié, on est en retard -> donc pas de speed_coef
      //  dans ce sens si l'erreur est positive, on ralenti, si elle est negative on ne fait rien

      // forward =>  c'est l'inverse, on dit que err_angle=-err_angle pour applique la même mecanique
      if (pTarget < (_count))
      {
        err_angle = -err_angle;
      };

      //  si l'erreur est supérieure au nb de degrés autorisés -> on stop
      if (err_angle > Nb_deg_autorise)
      {
        speed_coef = 0;
      }
      //  si l'erreur est inférieure au nb de degrés autorisés et supérieur à 0 -> on on ralenti
      if (err_angle < Nb_deg_autorise && err_angle > 0)
      {
        speed_coef = (Nb_deg_autorise - err_angle) / Nb_deg_autorise;
      }
      //  si l'erreur est inférieure à 0 -> pas de coef, on est trop lent
      if (err_angle <= 0)
      {
        speed_coef = 1;
      }

      // on prend le speed coef minimal
      if (_debug_flag)
      {
        printf("  speed_coef = %f  ", speed_coef);
      }
      if (speed_coef_final > speed_coef)
      {
        speed_coef_final = speed_coef;
      }
    }
  }
  return speed_coef_final;
}

void mbed_hall_driven_motor::motor_run_forward(double speed)
{
  // la vitesse max est 4095
  if (speed > 4095)
  {
    speed = 4095;
  }

  if (_motor_shield_type == 1)
  {
    _pwm->setPWM(_pwm_pin, 0, int(speed));
    _pwm->setPWM(_dir_pin, 0, 4095);
  }

  if (_motor_shield_type == 2)
  {
    _pwm->setPWM(_forward_pin, 0, int(speed));
    _pwm->setPWM(_backward_pin, 0, 0);
  }
}
void mbed_hall_driven_motor::motor_run_backward(double speed)
{
  // la vitesse max est 4095
  if (speed > 4095)
  {
    speed = 4095;
  }
  if (_motor_shield_type == 1)
  {
    _pwm->setPWM(_pwm_pin, 0, int(speed));
    _pwm->setPWM(_dir_pin, 0, 0);
  }

  if (_motor_shield_type == 2)
  {
    _pwm->setPWM(_forward_pin, 0, 0);
    _pwm->setPWM(_backward_pin, 0, int(speed));
  }
}
void mbed_hall_driven_motor::motor_stop()
{
  if (_motor_shield_type == 1)
  {
    _pwm->setPWM(_pwm_pin, 0, 0);
    _pwm->setPWM(_dir_pin, 0, 0);
  }

  if (_motor_shield_type == 2)
  {
    _pwm->setPWM(_forward_pin, 0, 0);
    _pwm->setPWM(_backward_pin, 0, 0);
  }
}