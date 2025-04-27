#include "mbed_hall_driven_motor.h"

/*!
 *  @brief constructeur
 *  @param count
 *  @param stop_pin
 *  @param pwm
 *  @param forward_pin
 *  @param backward_pin
 *  @param &target
 *  @param &angle
 *  @param motor_name 
 *  @param flag_start
 *  @param flag_stop
 *  @param Init_speed
 *  @param
   PullNone          = 0,
    PullUp            = 1,
    PullDown          = 2,
 */ /**/

mbed_hall_driven_motor::mbed_hall_driven_motor( int & count,
                                               DigitalIn &stop_pin,
                                               mbed_PWMServoDriver &pwm,
                                               Forward_or_dir_pin forward_or_dir_pin,
                                               Backward_or_speed_pin backward_or_speed_pin,
                                               Motor_name motor_name, 
                                               Flag_start flag_start,
                                               Flag_stop flag_stop, 
                                               Min_speed min_speed,
                                               Max_speed max_speed,
                                               Coef_Kp coef_Kp,
                                               Coef_Ki coef_Ki,
                                               Coef_Kd coef_Kd,
                                               Nb_tic_per_deg nb_tic_per_deg,
                                              //  End_stop_type end_stop_type,
                                               bool reverse_rotation
                                               ,int coef_accel)
    :_DigitalIn_stop(&stop_pin),
      _pwm(&pwm),
      _PID(&Input, &Output, &Setpoint, coef_Kp.get(), coef_Ki.get(), coef_Kd.get(), P_ON_E, DIRECT)

{ 
      _dir_pin = forward_or_dir_pin.get();
      _pwm_pin = backward_or_speed_pin.get();

      //   default value
      _motor_name = motor_name.get(); 
      _min_speed = min_speed.get();
      _max_speed = max_speed.get();
      _nb_tic_per_deg = nb_tic_per_deg.get(); 
      _coef_accel = coef_accel;
      //flag_pid_enable = false;
      //flag_start_smooth_enable = false;
      speed_manual_run = 0; 

      _flag_start = flag_start.get();
      _flag_stop = flag_stop.get();
      // _end_stop_type = end_stop_type.get();
      // définit la valeur par défaut
      _debug_flag = false  ;
      _reverse_rotation= reverse_rotation;
      _count = &count; 


}
 

//********************** methodes publiques

void mbed_hall_driven_motor::init()
{
  printf("init carte %s\n", _motor_name.c_str());
  
  // on initialise les variables
  //previous_speed = 0; 
  _flag_speed_sync = false;
  *_count = 0;
  _debug_count_when_stoped=0; 
  _PID.SetOutputLimits(_min_speed, _max_speed);
  _PID.SetMode(1);
  // log
  printf("fin init %s count %f  ", _motor_name.c_str(),*_count);
  printf(" count_min: %i \n",    _count_min);
}

void mbed_hall_driven_motor::init_position()
{
  printf("init %s\n", _motor_name.c_str());
  // _DigitalIn_stop->enable_irq(); // --> on allume la lecture de la butée
  // _DigitalIn_stop->read() == 1 --> en butée
  // on fait tourner le moteur jusqu'a la butée
  // if(_end_stop_type==0){_DigitalIn_stop->mode(PullDown);}
    // on fait tourner le moteur jusqu'a la butée
  // if(_end_stop_type==1){
   // _DigitalIn_stop->mode(PullUp);
    // }
  if (_debug_flag)
  {
    printf("run_forward %s interrupt: %i\n", _motor_name.c_str() ,_DigitalIn_stop->read());
  };
  while (_DigitalIn_stop->read() == 1   )
  {
    motor_run_forward(_min_speed );
  }
  motor_stop(); 
  if (_debug_flag)
  {
    printf("stop %s : on attends 100 milisecondes pour stabiliser le moteur, interrupt: %i \n ", _motor_name.c_str(),_DigitalIn_stop->read());
  };                                                 // on arrete le moteur
  ThisThread::sleep_for(chrono::milliseconds(100)); // on attend 100 milisecondes pour stabiliser le moteur
 *_count = 0;
  // on repart tout doucement pour que l'init soit juste après la butée
  if (_debug_flag)
  {
    printf("run_backward %s : on repart tout doucement pour que l'init soit juste apres la butee \n ", _motor_name.c_str());
  };
  while (_DigitalIn_stop->read() != 1)
  {
    motor_run_backward(_min_speed);
  }
  motor_stop(); // on arrete le moteur 
  //_DigitalIn_stop->disable_irq(); // --> on eteint la lecture de la butée

  ThisThread::sleep_for(chrono::milliseconds(100)); // on attend 100 milisecondes pour stabiliser le moteur
  //on bouge de 2 deg pour être assez loin de la butée
   while (*_count < abs((int) _nb_tic_per_deg)*2)
  {
    motor_run_backward(_min_speed);
  }
  motor_stop(); // on arrete le moteur 
  ThisThread::sleep_for(chrono::milliseconds(100)); // on attend 100 milisecondes pour stabiliser le moteur
  // on initialise les variables
  //previous_speed = 0;

  //_flag_speed_sync = false;
 
  *_count = 0;
  _debug_count_when_stoped=0; 
  // _PID.SetOutputLimits(_min_speed, _max_speed);
  // _PID.SetMode(1);

  // log
  printf("fin init %s count %f  ", _motor_name.c_str(),*_count);
  printf(" count_min: %i \n",    _count_min);
  //on stop le thread au cas ou
  //  flag_manual_stop = 1;
  flag_init_done = true; // on indique que l'init est faite
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

 
/*************** RUN ********
 * fait tourner le moteur en avant ou en arriere
 * la vitesse est donnée par get_speed  
 
*/
void mbed_hall_driven_motor::run()
{

//commandes manuelles
//forward -> superieur à 0, backward -> inférieur à 0  
if (speed_manual_run < 0) { motor_run_backward(- speed_manual_run );
                                    return;}
if( speed_manual_run > 0) { //si on est en butée on stop
                            // si l'init n'est pas faite, on ne fait rien
                            // sinon on arrete le moteur quand la butée est atteinte
                            if(flag_init_done == true) {
                              if (  *_count < (_count_min * _nb_tic_per_deg) ) {motor_stop();  
                                                             return;
                                                                    } 
                                                                  }                                     
                            // if (_DigitalIn_stop->read() == 0   ) {motor_stop(); 
                            //                                       return;
                            //                                     } 
                            motor_run_forward(speed_manual_run);
                            return; }
 //printf("motor run auto\n"); 
// deplacement avec une target
  _deplacement = _target - get_angle(); // au demarrage on calcul le deplacement pour la synchro
  _start_angle = get_angle();           // on enregistre la position des moteurs liés au demarrage
  previous_speed = _min_speed;
  _PID.Compute(true); // on redemarre le pid
  double target_count = _target * _nb_tic_per_deg; // calcul la target en nombre de tic
  
  if (_debug_flag) { printf("start run  deplacement: %f - start_angle: %f\n", _deplacement, _start_angle); }; 

  if ((target_count - *_count) > 0)
  {
          // on recule
          while (target_count > *_count)
          {
            int speed = get_speed(target_count);// calcul de la vitesse à chaque tour
            if (_debug_flag) {  printf("backward speed: %i\n",speed); } 
            motor_run_backward(speed);// on ajuste la vitesse
          }
  }
  else
  {
          // on avance
          while (target_count < *_count)
          {
            int speed = get_speed(target_count);// calcul de la vitesse à chaque tour
            if (_debug_flag) { printf("forward speed: %i\n",speed); } 
            motor_run_forward(speed); // on ajuste la vitesse
          }
  };
  // stop quand le compteur est arrivé
   motor_stop();
   _deplacement = 0;           
  if (_debug_flag)  { printf("fin target moteur : angle %f\n", get_angle()); }
 
}
//********************** methodes privées

/********GET_SPEED**********
 * appelé par RUN, elle renvoie la vitesse a appliquer au moteur
 * d'abord , on calcul la vitesse avec le PID
 * si  mouvement_type = false, on ne met pas en route le PID, cela permet de ne pas avoir d'arret quand il y a plusieurs étapes
*/
int mbed_hall_driven_motor::get_speed(double target)
{

  // calcul de la vitesse avec le PID
  Setpoint = 0;                  // SetPoint pour le PID
  Input = -abs(target - *_count); // Input pour le PID
  double speed;
  // le PID demarre à 1500 de la target.
  if (abs(target - *_count) < 1500)
  {
    _PID.Compute(); // le PID calcule la vitesse et commence a prendre en compte les valeurs pour l'intégrale
    speed = Output;
  }
  else
  {
    _PID.Compute(true); // on fait calculer le PID en reinitialisant la période de temp
    speed = Output;     // le PID calcule la vitesse et sans a prendre en compte le temp pour l'intégrale
  }
  // //pour les 2 premier degrès on commence au min
  // if  ((_start_angle - get_angle()) < 2 && speed == _max_speed){
  //   if (speed > previous_speed + 10 ) { speed = previous_speed + 10; }
  // } ;  

  //si le pid est desactivé flag_pid_enable == false, on ne ralentit pas
  // cela permet de ne pas avoir d'arret quand il y a plusieurs étapes 

  //if (flag_pid_enable == false  ){speed = _max_speed;} //SANS PID 

  double speed_coef = 1; 
  if (_flag_speed_sync) { speed_coef = get_speed_coef(target); }
  // pour ne pas demarrer trop vite, on n'augmente pas la vitesse de plus de 10 par cycle
   
   if (speed_coef == 1)
   {
    if (speed > previous_speed + _coef_accel ) { speed = previous_speed + _coef_accel; }
    }
  // on applique le coeficient le plus petit pour la synchro avec la vitesse 

  speed = speed * speed_coef;
 
  // debug
  if (_debug_flag)
  {
    printf("get_speed target %i\t", (int)(target));
    printf("speed %i\t", (int)(speed));
    printf("count %i\t", (int)(*_count));
    printf("angle %f\t", (get_angle()));
    printf("speed_coef %f\t", (speed_coef));
  }

  // pour ne pas demarrer trop vite, on n'augmente pas la vitesse de plus de 100 par cycle
  //quand il sont synchro, uniquement sur le moteur qui a le plus grand déplacement
   
  // int max_deplacement = 0 ;
  // std::string max_deplacement_motor_name ;
  // for (int i = 0; i < Nb_Motor_sync; i++)
  // { 
  //   if (max_deplacement < abs(synchronised_motor_list[i]->_deplacement) ) {
  //     max_deplacement = abs(synchronised_motor_list[i]->_deplacement) ;
  //     max_deplacement_motor_name = synchronised_motor_list[i]->_motor_name;  
  //     //printf("max  %s  motor_name %s  \n",max_deplacement_motor_name.c_str(),_motor_name.c_str());
  //     } 
  // }
    
    //si flag_start_smooth_enable = true
    // on met un amortisseur au demarrage
    // if (flag_start_smooth_enable == true  ){  
    // //if ( max_deplacement == abs(_deplacement)  )  
    // //  {
    //     if (speed > previous_speed + 10 ) { speed = previous_speed + 10; }
    //    //}
    // }
if (speed!=0) {
  previous_speed = speed;}
  else {previous_speed = _min_speed;}

  return (int)speed;
}
/***********GET SPEED COEF******
*   calcul du coeficient de vitesse pour la synchro
*   on ajuste la vitesse pour que l'erreur ne depasse pas 2deg
*   coef = pourcentage de l'erreur d'angle
*            2 deg -> 0% -> à 2 degrés, on stop le moteur
*            1 deg -> 50%
*            0 deg -> 100%
*   coef = (Nb_deg_autorisé - erreur d'angle) / Nb_deg_autorisé
*   
*/
double mbed_hall_driven_motor::get_speed_coef(double pTarget)
{ 
  double speed_coef_final = 1;
  double speed_coef = 1;
  double Nb_deg_autorise = 1;

  double move_angle_linked_motor;
  double move_angle_current_motor;
  double coef_angle;
  double err_angle;

  for (int i = 0; i < Nb_Motor_sync; i++)
  {
    if (_debug_flag) { printf("\n %s \t: ", synchronised_motor_list[i]->_motor_name.c_str());  };
    if (synchronised_motor_list[i]->_motor_name != _motor_name && _deplacement != 0 && synchronised_motor_list[i]->_deplacement != 0)
    {
      // pour la synchro, on doit toujours avoir
      // on doit avoir (move_angle_current_motor)  = (move_angle_linked_motor )*deplacement/linked_deplacement
      // err_angle = (move_angle_current_motor)  - (move_angle_linked_motor )*deplacement/linked_deplacement
      coef_angle = _deplacement / (synchronised_motor_list[i]->_deplacement);
      move_angle_linked_motor = synchronised_motor_list[i]->get_angle() - synchronised_motor_list[i]->_start_angle;
      move_angle_current_motor = get_angle() - _start_angle;
      err_angle = move_angle_current_motor - ((move_angle_linked_motor * coef_angle));

      if (_debug_flag)
      {
        printf(" move_angle_linked_motor = %f \t", move_angle_linked_motor);
        printf(" move_angle_current_motor = %f \t", move_angle_current_motor);
        printf(" coef_angle = %f  \t", coef_angle);
        printf(" err_angle = %f \t ", err_angle);
      };

      // backward (pTarget > *_count)  => quand l'angle augmente et que l'angle est plus petit que l'angle lié, on est en retard -> donc pas de speed_coef
      //  dans ce sens si l'erreur est positive, on ralenti, si elle est negative on ne fait rien

      // forward =>  c'est l'inverse, on dit que err_angle=-err_angle pour appliquer la même mecanique
      if (pTarget < (*_count)) { err_angle = -err_angle; };

      //  si l'erreur est supérieure au nb de degrés autorisés -> on stop
      if (err_angle >= Nb_deg_autorise) {  speed_coef = 0;  }
      
      //  si l'erreur est inférieure au nb de degrés autorisés et supérieur à 0 -> on on ralenti
      if (err_angle < Nb_deg_autorise && err_angle > 0) { speed_coef = (Nb_deg_autorise - err_angle) / Nb_deg_autorise; }
      
      //  si l'erreur est inférieure à 0 -> pas de coef, on est trop lent
      if (err_angle <= 0) { speed_coef = 1; }

      // comme on calcul un speed_coef pour chaque moteur synchronisé, on prend le speed_coef minimal
      if (_debug_flag) { printf("  speed_coef = %f \t ", speed_coef); }
      if (speed_coef_final > speed_coef) { speed_coef_final = speed_coef; }
    }
  }

  return speed_coef_final;
}

/***Methodes pour piloter la carte PWM *******/
void mbed_hall_driven_motor::motor_run_forward(double speed)
{ 
  // la vitesse max est 4095
  if (speed > 4095) { speed = 4095; }
  // if (_motor_name == "Poignet") {printf("countf %i\t", (int)(*_count));}
if(_reverse_rotation){ 
    _pwm->setPWM(_pwm_pin, 0, int(speed));
    _pwm->setPWM(_dir_pin, 0, 0); 
 }
    else{ 
      _pwm->setPWM(_pwm_pin, 0, int(speed));
      _pwm->setPWM(_dir_pin, 0, 4095); 
  }
}

void mbed_hall_driven_motor::motor_run_backward(double speed)
{ 
  // la vitesse max est 4095
  if (speed > 4095) {  speed = 4095; }
  // if (_motor_name == "Poignet") {printf("countb %i\t", (int)(*_count));}
  if(_reverse_rotation){ 
      _pwm->setPWM(_pwm_pin, 0, int(speed));
      _pwm->setPWM(_dir_pin, 0, 4095);
   }
    else{ 
    _pwm->setPWM(_pwm_pin, 0, int(speed));
    _pwm->setPWM(_dir_pin, 0, 0); 
    }
}

void mbed_hall_driven_motor::motor_stop()
{ //on met a zero les commandes manuelles
    speed_manual_run = 0; 
    _pwm->setPWM(_pwm_pin, 0, 0);
    _pwm->setPWM(_dir_pin, 0, 0);
    
    //printf("motor_stop\n");
}
double mbed_hall_driven_motor::get_angle()
{
  angle = *_count / _nb_tic_per_deg;
  return angle;
}