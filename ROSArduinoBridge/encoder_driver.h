/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
 
 
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define RIGHT_ENC_PIN_A PD3  //pin 3
  

#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void setEncoderDir(int i, int8_t dir);

