#ifndef KERNEL_H_
#define KERNEL_H_

#include "structures.h"

void Passport_Applyment(Cal_Passport *pass, Input_data *in, Nav_platform *platform);
void Input2Platform(Input_data *in, Nav_platform *platform);
void Acc_Calc(Nav_platform *platform);
void Velocity_Calc(Nav_platform *platform);
void Puasson(Nav_platform *platform);
void Normalization(Nav_platform *platform);
void Orientation_Angles_Calc(Nav_platform *platform);
void Platform_Step(Nav_platform *platform);
void Nav_Param_Calc(Nav_platform *platform);

#endif /* KERNEL_H_ */
