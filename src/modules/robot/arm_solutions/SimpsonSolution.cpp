#include "SimpsonSolution.h"
#include <fastmath.h>
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"
#include "Vector3.h"

#define simpson_effector_height_checksum        CHECKSUM("simpson_effector_height")
#define simpson_arm_length_checksum             CHECKSUM("simpson_arm_length")
#define simpson_printable_radius_checksum       CHECKSUM("simpson_printable_radius")

#define max_arm_length_x_checksum               CHECKSUM("max_arm_length_x")
#define max_arm_length_y_checksum               CHECKSUM("max_arm_length_y")
#define max_arm_length_z_checksum               CHECKSUM("max_arm_length_z")
#define shoulder_height_x_checksum              CHECKSUM("shoulder_height_x")
#define shoulder_height_y_checksum              CHECKSUM("shoulder_height_y")
#define shoulder_height_z_checksum              CHECKSUM("shoulder_height_z")


#define SQ(x) powf(x, 2)

SimpsonSolution::SimpsonSolution(Config* config)
{
    // Approximate Z height from shoulder swivel to wrist hinge (with hot end on bed)
    simpson_effector_height     = config->value(simpson_effector_height_checksum)->by_default(70.0f)->as_number();
    // Approximate distance between the centers of the hinge bolts when the end stop is triggered
    simpson_arm_length          = config->value(simpson_arm_length_checksum)->by_default(300.0f)->as_number();
    // Actually this is distance from the arm swivel point to bed center. It corresponds to 250mm between any pair of swivel points.
    simpson_printable_radius    = config->value(simpson_printable_radius_checksum)->by_default(144.37f)->as_number();

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

    SIMPSON_ARM1_X = -SIN_60 * simpson_printable_radius; // front left arm
    SIMPSON_ARM1_Y = -COS_60 * simpson_printable_radius;

    SIMPSON_ARM2_X =  SIN_60 * simpson_printable_radius; // front right arm
    SIMPSON_ARM2_Y = -COS_60 * simpson_printable_radius;

    SIMPSON_ARM3_X = 0.0F; // back middle arm
    SIMPSON_ARM3_Y = simpson_printable_radius;
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
    // unimplemented 
}

bool SimpsonSolution::set_optional(const arm_options_t& options) {

    arm_options_t::const_iterator i;

    i= options.find('H');
    if(i != options.end()) {
        simpson_effector_height= i->second;
    }
    i= options.find('L');
    if(i != options.end()) {
        simpson_arm_length= i->second;
    }
    i= options.find('R');
    if(i != options.end()) {
        simpson_printable_radius= i->second;
    }
    init();
    return true;
}

bool SimpsonSolution::get_optional(arm_options_t& options) {
    options['H']= this->simpson_effector_height;
    options['L']= this->simpson_arm_length;
    options['R']= this->simpson_printable_radius;
    return true;
};
