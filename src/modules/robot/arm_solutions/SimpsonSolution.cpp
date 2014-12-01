#include "SimpsonSolution.h"
#include <fastmath.h>
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"
#include "Vector3.h"

#define simpson_size_checksum                   CHECKSUM("simpson_size")

#define max_arm_length_x_checksum               CHECKSUM("max_arm_length_x")
#define max_arm_length_y_checksum               CHECKSUM("max_arm_length_y")
#define max_arm_length_z_checksum               CHECKSUM("max_arm_length_z")
#define shoulder_height_x_checksum              CHECKSUM("shoulder_height_x")
#define shoulder_height_y_checksum              CHECKSUM("shoulder_height_y")
#define shoulder_height_z_checksum              CHECKSUM("shoulder_height_z")


#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * 1e ## y) / 1e ## y)

SimpsonSolution::SimpsonSolution(Config* config)
{
    // Distance between shoulder pivot bolts (i.e. the vertical bolts through the bed)
    simpson_size    = config->value(simpson_size_checksum)->by_default(250.0f)->as_number();

    // For GUS, these MUST be updated with calibrated values
    max_arm_length_x    = config->value(max_arm_length_x_checksum)->by_default(295.0f)->as_number();
    max_arm_length_y    = config->value(max_arm_length_y_checksum)->by_default(295.0f)->as_number();
    max_arm_length_z    = config->value(max_arm_length_z_checksum)->by_default(295.0f)->as_number();
    shoulder_height_x    = config->value(shoulder_height_x_checksum)->by_default(70.f)->as_number();
    shoulder_height_y    = config->value(shoulder_height_y_checksum)->by_default(70.f)->as_number();
    shoulder_height_z    = config->value(shoulder_height_z_checksum)->by_default(70.f)->as_number();

    init();
}

void SimpsonSolution::init() {
    float SIN_60   = 0.8660254037844386F;
    float COS_60   = 0.5F;

    SIMPSON_PRINTABLE_RADIUS = (simpson_size / 2.0) / SIN_60;
    
    SIMPSON_ARM1_X = -SIN_60 * SIMPSON_PRINTABLE_RADIUS; // front left arm
    SIMPSON_ARM1_Y = -COS_60 * SIMPSON_PRINTABLE_RADIUS;

    SIMPSON_ARM2_X =  SIN_60 * SIMPSON_PRINTABLE_RADIUS; // front right arm
    SIMPSON_ARM2_Y = -COS_60 * SIMPSON_PRINTABLE_RADIUS;

    SIMPSON_ARM3_X = 0.0F; // back middle arm
    SIMPSON_ARM3_Y = SIMPSON_PRINTABLE_RADIUS;
}

void SimpsonSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] )
{
    actuator_mm[ALPHA_STEPPER] = max_arm_length_x - sqrtf(SQ(cartesian_mm[X_AXIS] - SIMPSON_ARM1_X) 
                                + SQ(cartesian_mm[Y_AXIS] - SIMPSON_ARM1_Y) 
                                + SQ(cartesian_mm[Z_AXIS] + shoulder_height_x));
    actuator_mm[BETA_STEPPER]  = max_arm_length_y - sqrtf(SQ(cartesian_mm[X_AXIS] - SIMPSON_ARM2_X) 
                                + SQ(cartesian_mm[Y_AXIS] - SIMPSON_ARM2_Y) 
                                + SQ(cartesian_mm[Z_AXIS] + shoulder_height_y));
    actuator_mm[GAMMA_STEPPER] = max_arm_length_z - sqrtf(SQ(cartesian_mm[X_AXIS] - SIMPSON_ARM3_X) 
                                + SQ(cartesian_mm[Y_AXIS] - SIMPSON_ARM3_Y) 
                                + SQ(cartesian_mm[Z_AXIS] + shoulder_height_z));
}

void SimpsonSolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] )
{
    float d = SIMPSON_ARM2_X * 2.0;
    float i = SIMPSON_ARM2_X;
    float j = -SIMPSON_ARM2_Y * 3.0;
    
    float x = (SQ(max_arm_length_x - actuator_mm[0]) - SQ(max_arm_length_y - actuator_mm[1]) + SQ(d)) / (2.0 * d);
    float y = (SQ(max_arm_length_x - actuator_mm[0]) - SQ(max_arm_length_z - actuator_mm[2]) - SQ(x) + SQ(x-i) + SQ(j)) / (2.0 * j);
    float zsq = SQ(max_arm_length_x - actuator_mm[0]) - SQ(x) - SQ(y);
    float z;
    
    if(zsq < 0.0){
        z = 0.0;
    }else{
        z = sqrtf(SQ(max_arm_length_x - actuator_mm[0]) - SQ(x) - SQ(y));
    }
    
    cartesian_mm[0] = ROUND(x, 4);
    cartesian_mm[1] = ROUND(y, 4);
    cartesian_mm[2] = ROUND(z, 4);
}
