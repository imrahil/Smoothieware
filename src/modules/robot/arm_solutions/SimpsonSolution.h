#ifndef SIMPSONSOLUTION_H
#define SIMPSONSOLUTION_H
#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class SimpsonSolution : public BaseSolution {
    public:
        SimpsonSolution(Config*);
        void cartesian_to_actuator( float[], float[] );
        void actuator_to_cartesian( float[], float[] );

    private:
        void init();

        float simpson_size;
        
        float max_arm_length_x;
        float max_arm_length_y;
        float max_arm_length_z;
        float shoulder_height_x;
        float shoulder_height_y;
        float shoulder_height_z;

        float SIMPSON_PRINTABLE_RADIUS;

        float SIMPSON_ARM1_X;
        float SIMPSON_ARM1_Y;
        float SIMPSON_ARM2_X;
        float SIMPSON_ARM2_Y;
        float SIMPSON_ARM3_X;
        float SIMPSON_ARM3_Y;
};
#endif // SIMPSONSOLUTION_H
