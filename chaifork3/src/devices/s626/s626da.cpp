#include "s626da.h"

S626DA::S626DA(comedi_t *parentDaq, int channel, int subdev):
    parentDaq(parentDaq), channel(channel), subdev(subdev), enable(true)
{

    // for ther 0 parameter, see http://stm.lbl.gov/comedi/doc/x94.html
    range_info = comedi_get_range(parentDaq, subdev, channel, 0);
    digitalMax = comedi_get_maxdata(parentDaq, subdev, channel);
    analogMin = range_info->min;
    analogMax = range_info->max;


}

void S626DA::write(lsampl_t data){
    int range = 0;
    comedi_data_write(parentDaq,subdev,channel,range,AREF_DIFF,data);
}

