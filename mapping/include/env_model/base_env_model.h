#ifndef ENV_MODEL_BASE_ENV_MODEL_H
#define ENV_MODEL_BASE_ENV_MODEL_H

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <boost/any.hpp>


class BaseEnvModel
{


    public:

        
        BaseEnvModel(){}
        // get map coordinates from cell index
        virtual double getPosX(int i) = 0;
        virtual double getPosY(int j) = 0;

        // get map cell index from map coordinates
        virtual int getCellX(double x) = 0;
        virtual int getCellY(double y) = 0;

        // perform raytracing
        virtual double getRayTrace(std::string layer, double x, double y, double a, double max_range) = 0;

        //Check if map is valid 
        virtual bool isValid(int i, int j) = 0;

        //get map value by index
        virtual float getValueAt(std::string layer, int i, int j) = 0;

        // get map value by position 
        virtual float getValueAtPos(std::string layer, double x, double y) = 0;
        
        //Update the map layer specified
        virtual void updateMap(std::string layer) = 0;
        
        // Get map given a map name (assuming service message from map server)
        virtual bool getMap(std::string mapname) = 0;

        //Initialize map object from map server 
        virtual bool initializeMapFromServer(std::string layer, std::string mapname) = 0;

        //add empty layer
        virtual void addMapLayer(std::string layer) = 0;

        //add specificed map to map layer
        virtual void addMapLayer(std::string layer,std::string mapname) = 0;
        
        ~BaseEnvModel(){}

};

#endif