#ifndef CONST_H_
#define CONST_H_

#include "types.h"

#define G_M_S_S      		((_FLOAT32) 9.806)
#define FREQ_HZ      		((_UINT32) 100)
#define DT_S         		((_FLOAT32) 1.0/FREQ_HZ)
#define R_EARTH_M   		((_UINT32) 6371302)
#define IN_BUFF_SIZE 		((_UINT32) 500)
#define ALIGNMENT_TIME_SEC  ((_UINT32) 10)
#define PI           		((_FLOAT32) 3.14159265)
#define U_EARTH_HZ   		((_FLOAT32) 7.292115e-5)
#define ACC_OFFSET_D_S      ((_FLOAT32) 0.5)
#define GYR_OFFSET_M_S_S    ((_FLOAT32) 0.196)
#define KSI                 ((_FLOAT32) 0.70711)
#define TAU_S               ((_FLOAT32) 0.05)
#define U_EARTH_RAD_S		((_FLOAT32) 2*3.14/86400)


#endif
