#include "s626encoder.h"

S626Encoder::S626Encoder(comedi_t *parentDaq, int encoderChannel, int subdev):
        parentDaq(parentDaq), encoderChannel(encoderChannel), subdev(subdev)
{
}

void S626Encoder::init(lsampl_t initValue){

    // Set up instruction for configureation
    insnInit.insn=INSN_CONFIG;   //configuration instruction
    insnInit.n=1;                //number of operation (must be 1)
    insnInit.data=&initValue;    //initial value loaded into encoder
                                 //during configuration
    insnInit.subdev=subdev;      //encoder subdevice
    insnInit.chanspec=CR_PACK(encoderChannel,0,AREF_OTHER); //encoder_channel
                                                            //to configure

    comedi_do_insn(parentDaq,&insnInit); //executing configuration

    // Set our inital value
    encoderValue = initValue;

    // Set up instruction for read
    insnRead.insn=INSN_READ;
    insnRead.n=1;
    insnRead.data=&encoderValue;
    insnRead.subdev=subdev;

    // Figure out max values
    maxdata = comedi_get_maxdata(parentDaq,subdev,encoderChannel);

    // Default CPR
    cpr = 1250;

}

void S626Encoder::update(){
    insnRead.chanspec=CR_PACK(encoderChannel,0,AREF_OTHER);
    comedi_do_insn(parentDaq,&insnRead);
}

double S626Encoder::getAngle()
{

    const double pi = 3.14159265359;

    // No quadrupture? 1*cpr instead of 4*cpr
    if(encoderValue >= maxdata/2)
        return -1.0*2.0*pi*(maxdata-encoderValue)/double(1*cpr);
    return 2.0*pi*encoderValue/double(1*cpr);
}
