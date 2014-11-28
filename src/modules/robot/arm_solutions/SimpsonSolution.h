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

        bool set_optional(const arm_options_t& options);
        bool get_optional(arm_options_t& options);

    private:
        void init();

        float simpson_effector_height;
        float simpson_arm_length;
        float simpson_printable_radius;
        
        float max_arm_length_x;
        float max_arm_length_y;
        float max_arm_length_z;
        float shoulder_height_x;
        float shoulder_height_y;
        float shoulder_height_z;

        float SIMPSON_ARM1_X;
        float SIMPSON_ARM1_Y;
        float SIMPSON_ARM2_X;
        float SIMPSON_ARM2_Y;
        float SIMPSON_ARM3_X;
        float SIMPSON_ARM3_Y;
};
#endif // SIMPSONSOLUTION_H
