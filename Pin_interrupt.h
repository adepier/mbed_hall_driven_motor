#include "mbed.h" 
struct  Encoder 
{  
 int  tic_forward =0 ;
 int  tic_backward=0 ;   
 int count = 0;
};

class Pin_interrupt {
public:
    Pin_interrupt(PinName pin , int num_compteur, Encoder &encoder)  ;
  void  fall_1();
  void  fall_2();
  void  rise_2();
  void  increment();



private:
    InterruptIn _interrupt; 
    Encoder * _encoder ;  
};
