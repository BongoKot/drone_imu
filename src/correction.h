#ifndef CORRECTION_H_
#define CORRECTION_H_

#include "structures.h"

void Radial_Correction(Nav_platform *platform);
void Satellite_Correction(Nav_platform *platform, Input_data *inp);

#endif /* CORRECTION_H_ */
