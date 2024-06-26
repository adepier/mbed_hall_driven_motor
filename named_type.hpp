#include "../NamedType/include/NamedType/named_type.hpp"
#include "PinNames.h"

// using FirstName = fluent::NamedType<std::string, struct FirstNameTag>;
// using LastName = fluent::NamedType<std::string, struct LastNameTag>;

// static const FirstName::argument firstName;
// static const LastName::argument lastName;

// using Count_1_pin = fluent::NamedType<PinName, struct Count_1_pinTag>;                         //pin compteur de tour
//using Stop_pin = fluent::NamedType<PinName, struct Stop_pinTag>;                           //pin de fin de course
using Forward_or_dir_pin = fluent::NamedType<int, struct Forward_or_dir_pinTag>;       //pin de commande de la direction du moteur
using Backward_or_speed_pin = fluent::NamedType<int, struct Backward_or_speed_pinTag>; //pin de commande de la vitesse du moteur
using Motor_name = fluent::NamedType<std::string, struct Motor_nameTag>;                   //nom du moteur 
using Flag_start = fluent::NamedType<std::int32_t, struct Flag_startTag>;                  //numéro du flag start
using Flag_stop = fluent::NamedType<std::int32_t, struct Flag_stopTag>;                    //numéro du flag stop 

using Min_speed = fluent::NamedType<std::int32_t, struct Min_speedTag>;                        //vitesse mini
using Max_speed = fluent::NamedType<std::int32_t, struct Max_speedTag>;                        //vitesse maxi
using Coef_Kp = fluent::NamedType< double, struct Coef_KpTag>;                         //PID coef proportionnel
using Coef_Ki = fluent::NamedType< double, struct Coef_KiTag>;                         //PID coef integral
using Coef_Kd = fluent::NamedType< double, struct Coef_KdTag>;                         //PID coef dérivée
using Nb_tic_per_deg = fluent::NamedType< double, struct Nb_tic_per_degTag>;           // nombre de tic par tour pour calculer l'angle 
// using End_stop_type = fluent::NamedType< int, struct End_stop_typeTag>;           // 0 : end stop a 0 au repos / 1: end stop a 1 au repos

//     int motor_shield_type,
//     int32_t flag_start,
//     int32_t flag_stop,
//     int init_speed,
//     int min_speed,
//     int max_speed,
//     double coef_Kp,
//     double coef_Ki,            //PID coef integral
//     double coef_Kd,            //PID coef dérivée
//     double nb_tic_per_deg      // nombre de tic par tour pour calculer l'angle

// static const Count_1_pin::argument count_1_pin;
//static const Stop_pin::argument stop_pin;
static const Forward_or_dir_pin::argument forward_or_dir_pin;
static const Backward_or_speed_pin::argument backward_or_speed_pin;
static const Motor_name::argument motor_name; 
static const Flag_start::argument flag_start;
static const Flag_stop::argument flag_stop;
static const Min_speed::argument min_speed;
static const Max_speed::argument max_speed;
static const Coef_Kp::argument coef_Kp;
static const Coef_Ki::argument coef_Ki;
static const Coef_Kd::argument coef_Kd;
static const Nb_tic_per_deg::argument nb_tic_per_deg; 
// static const End_stop_type::argument end_stop_type; 