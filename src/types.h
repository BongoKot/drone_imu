#ifndef TYPES_H_
#define TYPES_H_

#define _SINT8     char
#define _UINT8     unsigned char
#define _SINT16    signed short
#define _UINT16    unsigned short
#define _SINT32    signed long
#define _UINT32    unsigned long
#define _FLOAT32   float
#define _DOUBLE64  double

typedef _FLOAT32 _Vector[3];
typedef _FLOAT32 _Matrix[3][3];
typedef char     _NAME[7];
typedef _FLOAT32 _Temp[24][7];
typedef _FLOAT32 _DoubleVector[6];

#endif
