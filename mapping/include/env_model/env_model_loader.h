#ifndef ENV_MODEL_ENV_MODEL_LOADER_H
#define ENV_MODEL_ENV_MODEL_LOADER_H


#include "env_model/env_models.h"

class EnvModelLoader 
{
    public:
        static BaseEnvModel* createEnv(std::string EnvType);
};

#endif