#include <stdio.h>
#include "types.h"
#include "structures.h"
#include "const.h"
#include "functions.h"

void bin_file_writer(Cal_Passport *pas)
{
	FILE *passport = NULL;
	if((passport = fopen("D:/drone_imu/CPas.dat", "wb")) == NULL)
	{
		printf("file not found");
	}

	fwrite(pas, sizeof(Cal_Passport), 1, passport);

	fclose(passport);
}

void bin_file_reader(Cal_Passport *pas)
{
	FILE *passport = fopen("D:/drone_imu/CPas.dat", "rb");
	fread (pas, sizeof(Cal_Passport), 1, passport);
}

void PasDataRead(Cal_Passport *pas)
{
	FILE *file = fopen("D:/drone_imu/passport.txt", "r");
	const _SINT8 fmt[] = "%f %f %f %f %f %f %f\n";
	_SINT8 buffer[IN_BUFF_SIZE];
	for(_SINT16 i = 0; i < 24 && !feof(file); i++)
	{
		fgets(buffer, IN_BUFF_SIZE, file);
		sscanf(buffer, fmt, &pas->T[i][0],
							&pas->T[i][1],
							&pas->T[i][2],
							&pas->T[i][3],
							&pas->T[i][4],
							&pas->T[i][5],
							&pas->T[i][6]);
	}
	fclose(file);
}

void CalData_Averaging(Cal_File *f, FILE *file, Cal_Passport *pas)
{
	const _SINT8 fmt[] = "%ld %f %f %f %f %f %f %f %f %f %f\n";
	_SINT8 buffer[IN_BUFF_SIZE];
	_SINT16 i = 1;


	while (!feof(file))
	{
		if (file == NULL)
		{
			printf("file not found");
		}
		else
		{
			fgets(buffer, IN_BUFF_SIZE, file);
			sscanf(buffer, fmt, &f->index,
								&f->omega_deg_s[0],
								&f->omega_deg_s[1],
								&f->omega_deg_s[2],
								&f->acceleration_m_s_s[0],
								&f->acceleration_m_s_s[1],
								&f->acceleration_m_s_s[2],
								&f->temperature_deg_c,
								&f->roll_deg,
								&f->pitch_deg,
								&f->heading_deg);

			//применение температурной калибровки

			for (_SINT16 j = 0; j < 23; j++)
			{
				if (f->temperature_deg_c != 0.00 && pas->T[j][0] > f->temperature_deg_c)
				{
					for (_SINT16 k = 1; k < 7; k++)
					{
						f->P1[k-1] = (pas->T[j][k]-pas->T[j-1][k])/(pas->T[j][0]-pas->T[j-1][0]);
						f->P2[k-1]= (pas->T[j-1][k] - (pas->T[j][k]-pas->T[j-1][k])/(pas->T[j][0]-pas->T[j-1][0])*pas->T[j-1][0]);
					}
				}
			}

			f->omega_deg_s[0] -= (f->P1[0] * f->temperature_deg_c + f->P2[0]);
			f->omega_deg_s[1] -= (f->P1[1] * f->temperature_deg_c + f->P2[1]);
			f->omega_deg_s[2] -= (f->P1[2] * f->temperature_deg_c + f->P2[2]);

			f->acceleration_m_s_s[0] -= (f->P1[3] * f->temperature_deg_c + f->P2[3]);
			f->acceleration_m_s_s[1] -= (f->P1[4] * f->temperature_deg_c + f->P2[4]);
			f->acceleration_m_s_s[2] -= (f->P1[5] * f->temperature_deg_c + f->P2[5] - G_M_S_S);

			f->omega_av_deg_s[0] = (f->omega_av_deg_s[0] * (i-1) + f->omega_deg_s[0])/i;
			f->omega_av_deg_s[1] = (f->omega_av_deg_s[1] * (i-1) + f->omega_deg_s[1])/i;
			f->omega_av_deg_s[2] = (f->omega_av_deg_s[2] * (i-1) + f->omega_deg_s[2])/i;

			f->acceleration_av_m_s_s[0] = (f->acceleration_av_m_s_s[0] * (i-1) + f->acceleration_m_s_s[0])/i;
			f->acceleration_av_m_s_s[1] = (f->acceleration_av_m_s_s[1] * (i-1) + f->acceleration_m_s_s[1])/i;
			f->acceleration_av_m_s_s[2] = (f->acceleration_av_m_s_s[2] * (i-1) + f->acceleration_m_s_s[2])/i;

		i++;
		}
	}
	fclose(file);
}

void coef_finder(Cal_File *p1, Cal_File *p2, Cal_File *p3, Cal_File *p4, Cal_File *p5, Cal_File *p6, Cal_File *rot, Cal_Passport *pas)
{
	rot->buf0[0][0] = 2 * G_M_S_S;
	rot->buf0[1][0] = 0;
	rot->buf0[2][0] = 0;
	rot->buf0[0][1] = 0;
	rot->buf0[1][1] = 2 * G_M_S_S;
	rot->buf0[2][1] = 0;
	rot->buf0[0][2] = 0;
	rot->buf0[1][2] = 0;
	rot->buf0[2][2] = 2 * G_M_S_S;

	// alpha matrix

	rot->buf1[0][0] = p4->acceleration_av_m_s_s[0] - p2->acceleration_av_m_s_s[0];
	rot->buf1[1][0] = p4->acceleration_av_m_s_s[1] - p2->acceleration_av_m_s_s[1];
	rot->buf1[2][0] = p4->acceleration_av_m_s_s[2] - p2->acceleration_av_m_s_s[2];

	rot->buf1[0][1] = p5->acceleration_av_m_s_s[0] - p6->acceleration_av_m_s_s[0];
	rot->buf1[1][1] = p5->acceleration_av_m_s_s[1] - p6->acceleration_av_m_s_s[1];
	rot->buf1[2][1] = p5->acceleration_av_m_s_s[2] - p6->acceleration_av_m_s_s[2];

	rot->buf1[0][2] = p1->acceleration_av_m_s_s[0] - p3->acceleration_av_m_s_s[0];
	rot->buf1[1][2] = p1->acceleration_av_m_s_s[1] - p3->acceleration_av_m_s_s[1];
	rot->buf1[2][2] = p1->acceleration_av_m_s_s[2] - p3->acceleration_av_m_s_s[2];

	inv_m(rot->buf1); //ошибка здесь

	mult_m_m(rot->buf1, rot->buf0, pas->A);

	// alpha vector

	mult_m_num(pas->A, 0.16666, rot->buf1);

	rot->buf2[0] = p1->acceleration_av_m_s_s[0] + p2->acceleration_av_m_s_s[0]+ p3->acceleration_av_m_s_s[0]
			+ p4->acceleration_av_m_s_s[0]+ p5->acceleration_av_m_s_s[0]+ p6->acceleration_av_m_s_s[0];
	rot->buf2[1] = p1->acceleration_av_m_s_s[1] + p2->acceleration_av_m_s_s[1]+ p3->acceleration_av_m_s_s[1]
			+ p4->acceleration_av_m_s_s[1]+ p5->acceleration_av_m_s_s[1]+ p6->acceleration_av_m_s_s[1];
	rot->buf2[2] = p1->acceleration_av_m_s_s[2] + p2->acceleration_av_m_s_s[2]+ p3->acceleration_av_m_s_s[2]
			+ p4->acceleration_av_m_s_s[2]+ p5->acceleration_av_m_s_s[2]+ p6->acceleration_av_m_s_s[2];

	mult_m_v(rot->buf1, rot->buf2, pas->alpha);

	// beta matrix

	inv_m(rot->int_omega);

	rot->buf0[0][0] = 180;
	rot->buf0[1][0] = 0;
	rot->buf0[2][0] = 0;
	rot->buf0[0][1] = 0;
	rot->buf0[1][1] = 180;
	rot->buf0[2][1] = 0;
	rot->buf0[0][2] = 0;
	rot->buf0[1][2] = 0;
	rot->buf0[2][2] = 180;

	mult_m_m(rot->int_omega, rot->buf0, pas->B);

	// beta vector

	rot->buf2[0] = (p5->omega_av_deg_s[0] + p6->omega_av_deg_s[0])/2;
	rot->buf2[1] = (p5->omega_av_deg_s[1] + p6->omega_av_deg_s[1])/2;
	rot->buf2[2] = (p5->omega_av_deg_s[2] + p6->omega_av_deg_s[2])/2;

	mult_m_v(pas->B, rot->buf2, pas->beta);

	// AB

	rot->buf2[0] = (p4->omega_av_deg_s[0] - p2->omega_av_deg_s[0])/2;
	rot->buf2[1] = (p4->omega_av_deg_s[1] - p2->omega_av_deg_s[1])/2;
	rot->buf2[2] = (p4->omega_av_deg_s[2] - p2->omega_av_deg_s[2])/2;

	mult_m_v(pas->B, rot->buf2, rot->buf3);

	mult_v_num(rot->buf3, 1/G_M_S_S, rot->buf2);

	pas->AB[0][0] = rot->buf2[0];
	pas->AB[0][1] = rot->buf2[1];
	pas->AB[0][2] = rot->buf2[2];

	rot->buf2[0] = (p5->omega_av_deg_s[0] - p6->omega_av_deg_s[0])/2;
	rot->buf2[1] = (p5->omega_av_deg_s[1] - p6->omega_av_deg_s[1])/2;
	rot->buf2[2] = (p5->omega_av_deg_s[2] - p6->omega_av_deg_s[2])/2;

	mult_m_v(pas->B, rot->buf2, rot->buf3);

	mult_v_num(rot->buf3, 1/G_M_S_S, rot->buf2);

	pas->AB[1][0] = rot->buf2[0];
	pas->AB[1][1] = rot->buf2[1];
	pas->AB[1][2] = rot->buf2[2];

	rot->buf2[0] = (p1->omega_av_deg_s[0] - p3->omega_av_deg_s[0])/2;
	rot->buf2[1] = (p1->omega_av_deg_s[1] - p3->omega_av_deg_s[1])/2;
	rot->buf2[2] = (p1->omega_av_deg_s[2] - p3->omega_av_deg_s[2])/2;

	mult_m_v(pas->B, rot->buf2, rot->buf3);

	mult_v_num(rot->buf3, 1/G_M_S_S, rot->buf2);

	pas->AB[2][0] = rot->buf2[0];
	pas->AB[2][1] = rot->buf2[1];
	pas->AB[2][2] = rot->buf2[2];
}


/*
void temp_coef_finder(Cal_File *f, Cal_Passport *pas)
{
	for (_SINT16 i = 0; i < 23; i++)
	{
		if (f->temperature_deg_c != 0.00 && pas->T[i][0] > f->temperature_deg_c)
		{
			for (_SINT16 k = 1; k < 7; k++)
			{
				f->P1[k] = (pas->T[i][k]-pas->T[i-1][k])/(pas->T[i][0]-pas->T[i-1][0]);
				f->P2[k]= (pas->T[i-1][k] - (pas->T[i][k]-pas->T[i-1][k])/(pas->T[i][0]-pas->T[i-1][0])*pas->T[i-1][0]);
			}
		}
	}
}
*/
void integrate(Cal_File *f, FILE *file, _SINT16 n)
{
	const _SINT8 fmt[] = "%ld %f %f %f %f %f %f %f %f %f %f\n";
	_SINT8 buffer[IN_BUFF_SIZE];

	f->int_omega[0][n] = 0;
	f->int_omega[1][n] = 0;
	f->int_omega[2][n] = 0;

	while (!feof(file))
	{
		if (file == NULL)
		{
			printf("file not found");
		}
		else
		{
			fgets(buffer, IN_BUFF_SIZE, file);
					sscanf(buffer, fmt, &f->index,
										&f->omega_deg_s[0],
										&f->omega_deg_s[1],
										&f->omega_deg_s[2],
										&f->acceleration_m_s_s[0],
										&f->acceleration_m_s_s[1],
										&f->acceleration_m_s_s[2],
										&f->temperature_deg_c,
										&f->roll_deg,
										&f->pitch_deg,
										&f->heading_deg);
		}

		f->int_omega[0][n] = f->int_omega[0][n] + f->omega_deg_s[0] * DT_S;
		f->int_omega[1][n] = f->int_omega[1][n] + f->omega_deg_s[1] * DT_S;
		f->int_omega[2][n] = f->int_omega[2][n] + f->omega_deg_s[2] * DT_S;
	}
}

void calibration(Cal_File *p1, Cal_File *p2, Cal_File *p3, Cal_File *p4, Cal_File *p5, Cal_File *p6, Cal_File *rot, Cal_Passport *pas)
{
	FILE *P1 = fopen("H:/fpv_imu/pos1.txt", "r");
	FILE *P2 = fopen("H:/fpv_imu/pos2.txt", "r");
	FILE *P3 = fopen("H:/fpv_imu/pos3.txt", "r");
	FILE *P4 = fopen("H:/fpv_imu/pos4.txt", "r");
	FILE *P5 = fopen("H:/fpv_imu/pos5.txt", "r");
	FILE *P6 = fopen("H:/fpv_imu/pos6.txt", "r");

	FILE *Fx = fopen("H:/fpv_imu/HrXrep1.txt", "r");
	FILE *Fy = fopen("H:/fpv_imu/HrYrep1.txt", "r");
	FILE *Fz = fopen("H:/fpv_imu/HrZrep1.txt", "r");

	PasDataRead(pas);

	CalData_Averaging(p1, P1, pas);
	CalData_Averaging(p2, P2, pas);
	CalData_Averaging(p3, P3, pas);
	CalData_Averaging(p4, P4, pas);
	CalData_Averaging(p5, P5, pas);
	CalData_Averaging(p6, P6, pas);

	integrate(rot, Fx, 0);
	integrate(rot, Fy, 1);
	integrate(rot, Fz, 2);

	coef_finder(p1, p2, p3, p4, p5, p6, rot, pas);
	bin_file_writer(pas);
}


