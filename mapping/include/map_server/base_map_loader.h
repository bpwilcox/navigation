#ifndef MAP_SERVER_BASE_MAP_LOADER_H
#define MAP_SERVER_BASE_MAP_LOADER_H

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <boost/any.hpp>

enum MapMode {TRINARY, SCALE, RAW};

class BaseMapLoader
{


    public:
        
        boost::any map;

        // MetaData from YAML file, should this be defined here? 
        std::string fname = "";
        std::string mapfname = "";
        double origin[3];
        int negate;
        double occ_th, free_th;
        double res;
        MapMode mode = TRINARY;
        std::string frame_id = "map";
        
        BaseMapLoader(){}
        
        virtual void loadMapInfoFromFile(std::string fname)= 0;
        
        virtual void loadMapFromFile(std::string mapfname) = 0;

        
        ~BaseMapLoader(){}

};

#endif