#ifndef S626ENCODER_H
#define S626ENCODER_H

#include <comedilib.h>



class S626Encoder
{
public:
    S626Encoder(comedi_t *parentDaq, int encoderChannel, int subdev=5);
    void init(lsampl_t initValue = 0);
    void update();
    void setCPR(int cprNonQuadrupture) { this->cpr = cprNonQuadrupture; }

    double getAngle();

    lsampl_t getValue() { return encoderValue; }
    lsampl_t getMax() { return maxdata; }


protected:
    comedi_t *parentDaq;
    int encoderChannel;
    int subdev;

    int cpr;

    comedi_insn insnInit;
    comedi_insn insnRead;

    lsampl_t encoderValue;

    lsampl_t maxdata;
};

#endif // S626ENCODER_H
