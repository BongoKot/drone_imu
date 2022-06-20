#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "types.h"

#define sqr(x)             ((x) * (x))
#define Deg2Rad_rad(x)     ((_FLOAT32)(x) * PI / 180.0)
#define Rad2Deg_deg(x)     ((_FLOAT32)(x) * 180.0  / PI)

void mult_m_v(_Matrix A, _Vector B, _Vector C);
void mult_m_m(_Matrix A, _Matrix B, _Matrix C);
void mult_v_num(_Vector A, _FLOAT32 num, _Vector C);
void mult_m_num(_Matrix A, _FLOAT32 num, _Matrix C);
void sum_m_m(_Matrix A, _Matrix B, _Matrix C);
void sub_m_m(_Matrix A, _Matrix B, _Matrix C);
void sum_v_v(_Vector A, _Vector B, _Vector C);
void det_m(_Matrix A, _FLOAT32 *D);
void inv_m(_Matrix A);
void union_m(_Matrix A, _Matrix B);
void transp_m(_Matrix A);
void inv_m(_Matrix A);
void zeros_m(_Matrix A);
void print_v(_NAME Name, _Vector Vector);
void print_m(_NAME Name, _Matrix Matrix);
_FLOAT32 v_len(_Vector A);

#endif /* FUNCTIONS_H_ */
