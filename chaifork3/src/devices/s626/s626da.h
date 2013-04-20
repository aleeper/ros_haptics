#ifndef S626DA_H
#define S626DA_H

#include <comedilib.h>
#include <iostream>

class S626DA
{
public:
    S626DA(comedi_t *parentDaq, int channel, int subdev=1);
    void write(lsampl_t data);
    lsampl_t getDigitalMax() { return digitalMax; }
    double getAnalogMax() { return analogMax; }
    double getAnalogMin() { return analogMin; }

    void panic(){
        enable = false;
        write(digitalMax/2+1); // Hopefully mapped to 0...
    }

    void writeVolt(double volt){
        if(!enable)
            return;

        //std::cout << "writeVolt " << volt << " on channel" << channel << std::endl;

        double cap = 3.0;
        if(volt > cap)
            volt = cap;
        if(volt < -cap)
            volt = -cap;

        if(volt <= cap && volt >= -cap )
            write(comedi_from_phys(volt,range_info,digitalMax));
        else
            std::cout << "Woaaaha horesy!" << std::endl;
        lastWriteVolt = volt;
    }

    double getLastWriteVolt() { return lastWriteVolt; }

protected:
    comedi_t *parentDaq;
    int channel;
    int subdev;

    comedi_range *range_info;
    double analogMax;
    double analogMin;
    lsampl_t digitalMax;

    double lastWriteVolt;
    bool enable;
};

#endif // S626DA_H
