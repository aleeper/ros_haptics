#include "s626comm.h"
#include <iostream>

S626Comm::S626Comm(): initalized(false)
{
}

void S626Comm::init(){
   daq = comedi_open("/dev/comedi0");
   if (daq != 0)
       std::cout << "Comedi device open" << std::endl;
   else {
       std::cout << "Comedi device did not open" << std::endl;
       return;
   }


   // What do we have
   numDAs = 3;
   numEncoders = 3;

   // Init encoders
   for(int i=0;i<numEncoders;i++){
       encoder[i] = new S626Encoder(daq,i);
       encoder[i]->init();
   }

   // Init DAs
   for(int i=0;i<numDAs;i++){
       da[i] = new S626DA(daq,i);
   }

   // Start external amplifier by setting some digital output pins
   if(numEncoders >= 1){
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_5, COMEDI_OUTPUT);
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_3, COMEDI_OUTPUT);
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_1, COMEDI_OUTPUT);
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_5, digitalLow); // 5 = yellow = pos enable
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_3, digitalLow); // 3 = green = neg enable
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_1, digitalLow); // 1 = ENABLE (default is pulled high, so 1 = power off, 0 = power on)
    }
   if(numEncoders >= 2){
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_11, COMEDI_OUTPUT);
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_9, COMEDI_OUTPUT);
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_7, COMEDI_OUTPUT);
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_11, digitalLow); // 5 = yellow = pos enable
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_9, digitalLow); // 3 = green = neg enable
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_7, digitalLow); // 1 = ENABLE (default is pulled high, so 1 = power off, 0 = power on)
   }
   if(numEncoders >= 3){
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_17, COMEDI_OUTPUT);
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_15, COMEDI_OUTPUT);
       comedi_dio_config(daq, digitalOutSubdev, s626_J3_pin_13, COMEDI_OUTPUT);
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_17, digitalLow); // 5 = yellow = pos enable
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_15, digitalLow); // 3 = green = neg enable
       comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_13, digitalLow); // 1 = ENABLE (default is pulled high, so 1 = power off, 0 = power on)
   }
   initalized = true;


}

void S626Comm::shutdown(){

    if(initalized){
        for(int i=0;i<numDAs;i++){
            da[i]->panic();
        }



        // Stop amplifier
        comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_5, digitalHigh); // pos enable
        comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_3, digitalHigh); // neg enable
        comedi_dio_write( daq, digitalOutSubdev, s626_J3_pin_1, digitalHigh); //Disable Amplifier

        comedi_close(daq);

    }

}
