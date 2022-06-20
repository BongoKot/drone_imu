#ifndef GLOBAL_INIT_H_
#define GLOBAL_INIT_H_

#include "structures.h"

extern void Platform_Initialization(Nav_platform *platform, Input_data *inp);
void Global_Initialization(Nav_platform *pl1, Nav_platform *pl2, Input_data *inp);

#endif /* GLOBAL_INIT_H_ */
