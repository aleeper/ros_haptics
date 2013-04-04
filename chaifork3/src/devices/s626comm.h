#ifndef S626COMM_H
#define S626COMM_H

#include <iostream>

/*
 * WoodenPhantom interface code
 * (C) Jonas Forsslund, Stanford University 2012 <jofo@stanford.edu>
 *
 * Comedilib tutorial attribution to David A. Schleef <ds@schleef.org>
 * Initalization from s626.c attribution to Gianluca Palli <gpalli@deis.unibo.it>
 */

#include <comedilib.h>
#include "s626encoder.h"
#include "s626da.h"

class S626Comm
{
public:
    S626Comm();

    ~S626Comm() {
        std::cout << "S626Comm destructor: shutting down." << std::endl;
        shutdown();
    }

    void init();
    void shutdown();

    void stopMotors(){
        for(int i=0;i<numDAs;i++){
            da[i]->panic();
        }
    }

    bool isInitialized() { return initalized; }



    static const int digitalOutSubdev = 4;
    static const int s626_J3_pin_1 = 15;
    static const int s626_J3_pin_3 = 14;
    static const int s626_J3_pin_5 = 13;

    static const int s626_J3_pin_7 = 12;
    static const int s626_J3_pin_9 = 11;
    static const int s626_J3_pin_11 = 10;

    static const int s626_J3_pin_13 = 9;
    static const int s626_J3_pin_15 = 8;
    static const int s626_J3_pin_17 = 7;


    static const int digitalHigh = 0;
    static const int digitalLow = 1;


    S626Encoder *encoder[3];
    S626DA *da[3];

protected:
    bool initalized;
    comedi_t* daq;


    int numEncoders;
    int numDAs;
};

#endif // S626COMM_H
