#include <stdio.h>
#include "types.h"
#include "functions.h"

void mult_m_v(_Matrix A, _Vector B, _Vector C)     // умножение матрицы 3х3 вектор 3х1
{
	for (_SINT16 i = 0; i<3; i++)
	{
		C[i] = 0;
		for (_SINT16 j = 0; j<3; j++)
		{
			C[i] += A[i][j]*B[j];
		}
	}
}

void mult_v_num(_Vector A, _FLOAT32 num, _Vector C)     // умножение вектора на число
{
	for (_SINT16 i = 0; i<3; i++)
	{
		C[i] = 0;
		C[i] += A[i]*num;
	}
}

void mult_m_m(_Matrix A, _Matrix B, _Matrix C)     // умножение матрицы 3х3 на матрицу 3х3
{
	for (_SINT16 i = 0; i<3; i++)
			for (_SINT16 j = 0; j<3; j++)
			{
				C[i][j] = 0;
				for (_SINT16 k = 0; k<3; k++)
					C[i][j] += A[i][k]*B[k][j];
			}
}

void mult_m_num(_Matrix A, _FLOAT32 num, _Matrix C)   // умножение матрицы 3х3 на число
{
	for (_SINT16 i = 0; i<3; i++)
		for (_SINT16 j = 0; j<3; j++)
		{
			C[i][j] = 0;
			C[i][j] += A[i][j]*num;
		}
}

void sum_m_m(_Matrix A, _Matrix B, _Matrix C)      // суммирование матриц 3х3
{
	for (_SINT16 i = 0; i<3; i++)
		for (_SINT16 j = 0; j<3; j++)
		{
			C[i][j] = 0;
			C[i][j] += A[i][j] + B[i][j];
		}
}

void sum_v_v(_Vector A, _Vector B, _Vector C)      // суммирование векторов
{
	for (_SINT16 i = 0; i<3; i++)
	{
		C[i] = 0;
		C[i] += A[i] + B[i];
	}
}

void sub_m_m(_Matrix A, _Matrix B, _Matrix C)      // вычитание матриц 3х3
{
	for (_SINT16 i = 0; i<3; i++)
		for (_SINT16 j = 0; j<3; j++)
		{
			C[i][j] = 0;
			C[i][j] += A[i][j] + (-B[i][j]);
		}
}

void det_m(_Matrix A, _FLOAT32 *D)
{
	*D = A[0][0] * A[1][1] * A[2][2] + A[0][1] * A[1][2] * A[2][0] + A[1][0] * A[2][1] * A[0][2]
	  - A[0][2] * A[1][1] * A[2][0] - A[1][0] * A[0][1] * A[2][2] - A[0][0] * A[2][1] * A[1][2];
}

void union_m(_Matrix A, _Matrix B)
{
	B[0][0] = A[1][1] * A[2][2] - A[1][2] * A[2][1];
	B[1][0] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]);
	B[2][0] = A[0][1] * A[1][2] - A[0][2] * A[1][1];

	B[0][1] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]);
	B[1][1] = A[0][0] * A[2][2] - A[0][2] * A[2][0];
	B[2][1] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]);

	B[0][2] = A[1][0] * A[2][1] - A[1][1] * A[2][0];
	B[1][2] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]);
	B[2][2] = A[0][0] * A[1][1] - A[0][1] * A[1][0];

}

void transp_m(_Matrix A)
{
	_FLOAT32 buf;
	buf = A[1][0];
	A[1][0] = A[0][1];
	A[0][1] = buf;

	buf = A[2][0];
	A[2][0] = A[0][2];
	A[0][2] = buf;

	buf = A[2][1];
	A[2][1] = A[1][2];
	A[1][2] = buf;
}

void inv_m(_Matrix A)
{
	_FLOAT32 D = 0;
	_Matrix B;

	det_m(A, &D);

	if (D != 0)
	{
		union_m(A, B);
		transp_m(B);
		mult_m_num(B, 1/D, A);
	}
}
void zeros_m(_Matrix A)
{
	for(_SINT16 i = 0; i<3; i++)
	{
		for(_SINT16 j = 0; j<3; j++){
			A[i][j] = 0;
		}
	}
}

void print_v(_NAME Name, _Vector Vector)           // вывод вектора в консоль
{
	printf("\n%s:\n", Name);
		for (_SINT16 i=0;i<3;i++)
			printf("%.3f\n", Vector[i]);
}

void print_m(_NAME Name, _Matrix Matrix)           // вывод вектора в консоль
{
	printf("\n%s:\n", Name);
		for (_SINT16 i=0;i<3;i++)
			printf("%.3f %.3f %.3f\n", Matrix[i][0], Matrix[i][1], Matrix[i][2]);
}

_FLOAT32 v_len(_Vector A)
{
	_FLOAT32 result;
	result = sqr(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
	return result;
}
