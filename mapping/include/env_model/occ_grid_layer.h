#ifndef ENV_MODEL_OCC_GRID_LAYER_H
#define ENV_MODEL_OCC_GRID_LAYER_H

#include <string.h>

struct layer_t
{   //May later decide to add meta_data into layer  OR even specific layer functionality

    /* 

        double resolution;
        int size_x;
        int size_y;
        double origin_x;
        double origin_y

    */

    std::string name;
    union {
        signed char * data;  
        double * ddata;  
    };
};
    
#endif