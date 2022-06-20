#include <stdio.h>
#include <math.h>
#include "functions.h"
#include "kernel.h"
#include "types.h"
#include "const.h"

void Passport_Applyment(Cal_Passport *pass, Input_data *in, Nav_platform *platform)
{
	if (in->temperature_deg_c < platform->Current_Temp_A_deg_c || in->temperature_deg_c > platform->Current_Temp_B_deg_c)
	{
		for (int i = 0; i < 24; i++)
		{
			if (pass->T[i][0] > in->temperature_deg_c)
			{
				platform->Current_Temp_B_deg_c = pass->T[i][0];
				platform->Current_Temp_A_deg_c = pass->T[i-1][0];

				for (int j = 0; j < 6; j++)
				{
					platform->P1[j] = (pass->T[i][j+1] - pass->T[i-1][j+1])/(pass->T[i][0] - pass->T[i-1][0]);
					platform->P2[j] = pass->T[i-1][j+1] - (pass->T[i][j+1] - pass->T[i-1][j+1])/(pass->T[i][0] - pass->T[i-1][0]) * pass->T[i-1][0];
				}
				break;
			}
		}
	}

	for (int k = 0; k < 6; k++)
	{
		platform->Corr_Value[k] = platform->P1[k] * in->temperature_deg_c + platform->P2[k];
		if (k == 5)
		{
			platform->Corr_Value[k] = platform->P1[k] * in->temperature_deg_c + platform->P2[k] - G_M_S_S;
		}
	}

	for (int k = 0; k < 3; k++)
	{
		in->omega_raw_deg_s[k] = in->omega_raw_deg_s[k] - platform->Corr_Value[k];
		in->acceleration_raw_m_s_s[k] = in->acceleration_raw_m_s_s[k] - platform->Corr_Value[k+3];
	}

	//применение коэффициентов к ускорениям

	mult_m_v(pass->A, in->acceleration_raw_m_s_s, in->A);
	sum_v_v(in->A, pass->alpha, in->acceleration_m_s_s);

	//применение коэффициентов к угловым скоростям

	mult_m_v(pass->B, in->omega_raw_deg_s, in->A);
	sum_v_v(in->A, pass->beta, in->B);
	mult_m_v(pass->AB, in->acceleration_m_s_s, in->A);
	sum_v_v(in->B, in->A, in->omega_deg_s);
}


void Input2Platform(Input_data *in, Nav_platform *platform)
{

	platform->Temperature_deg_c = in->temperature_deg_c;

	if (platform->Iteration == 0)
	{
		platform->Om_B_Av_rad_s[0] = platform->Om_B_Av_rad_s[0]/(ALIGNMENT_TIME_SEC * FREQ_HZ);
		platform->Om_B_Av_rad_s[1] = platform->Om_B_Av_rad_s[1]/(ALIGNMENT_TIME_SEC * FREQ_HZ);
		platform->Om_B_Av_rad_s[2] = platform->Om_B_Av_rad_s[2]/(ALIGNMENT_TIME_SEC * FREQ_HZ);

		platform->F_B_av_m_s_s[0] = platform->F_B_av_m_s_s[0]/(ALIGNMENT_TIME_SEC * FREQ_HZ);
		platform->F_B_av_m_s_s[1] = platform->F_B_av_m_s_s[1]/(ALIGNMENT_TIME_SEC * FREQ_HZ);
		platform->F_B_av_m_s_s[2] = platform->F_B_av_m_s_s[2]/(ALIGNMENT_TIME_SEC * FREQ_HZ) - G_M_S_S;

	}
	if (platform->Iteration > 0)
	{
		platform->Om_B_rad_s[0] = Deg2Rad_rad(in->omega_deg_s[0]);// - (platform->Om_B_Av_rad_s[0]);
		platform->Om_B_rad_s[1] = Deg2Rad_rad(in->omega_deg_s[1]);// - (platform->Om_B_Av_rad_s[1]);
		platform->Om_B_rad_s[2] = Deg2Rad_rad(in->omega_deg_s[2]);// - (platform->Om_B_Av_rad_s[2]);

		platform->F_B_m_s_s[0] = in->acceleration_m_s_s[0];// - platform->F_B_av_m_s_s[0];
		platform->F_B_m_s_s[1] = in->acceleration_m_s_s[1];// - platform->F_B_av_m_s_s[1];
		platform->F_B_m_s_s[2] = in->acceleration_m_s_s[2];// - platform->F_B_av_m_s_s[2];
	}
	if (platform->Iteration < 0)
	{
		platform->Om_B_rad_s[0] = Deg2Rad_rad(in->omega_deg_s[0]);
		platform->Om_B_rad_s[1] = Deg2Rad_rad(in->omega_deg_s[1]);
		platform->Om_B_rad_s[2] = Deg2Rad_rad(in->omega_deg_s[2]);

		platform->F_B_m_s_s[0] = in->acceleration_m_s_s[0];
		platform->F_B_m_s_s[1] = in->acceleration_m_s_s[1];
		platform->F_B_m_s_s[2] = in->acceleration_m_s_s[2];

	}

	//Deg2Rad_rad(in->latitude_sns_deg);
	//Deg2Rad_rad(in->longitude_sns_deg);
}
	// Пересчет ускорений в с.к. LocalLevel [4]

void Acc_Calc(Nav_platform *platform)
{
	//platform->F_m_s_s[0] += platform->F_c_m_s_s[0];
	//platform->F_m_s_s[1] += platform->F_c_m_s_s[1];
	//platform->F_m_s_s[2] += platform->F_c_m_s_s[2];

	mult_m_v(platform->C_B_LL, platform->F_B_m_s_s, platform->F_m_s_s); // k1
}

	// Расчет скоростей коррекции положения с.к. LL [5]

void Velocity_Calc(Nav_platform *platform)
{
	//расчет линейных скоростей коррекции положения с.к. LocalLevel [5.1]
	platform->F_m_s_s[0] += platform->F_c_m_s_s[0];
	platform->F_m_s_s[1] += platform->F_c_m_s_s[1];

	_FLOAT32 dVe_M_S = (platform->F_m_s_s[0] - platform->Om_rad_s[1] * platform->V_m_s[2] + platform->Om_rad_s[2] * platform->V_m_s[1] - U_EARTH_HZ * cos(platform->fi_rad) * platform->V_m_s[2] + U_EARTH_HZ * sin(platform->fi_rad) * platform->V_m_s[1]) * DT_S;
	_FLOAT32 dVn_M_S = (platform->F_m_s_s[1] - platform->Om_rad_s[0] * platform->V_m_s[2] - platform->Om_rad_s[2] * platform->V_m_s[0] - U_EARTH_HZ * sin(platform->fi_rad) * platform->V_m_s[0]) * DT_S;

	platform->V_m_s[0] += dVe_M_S;
	platform->V_m_s[1] += dVn_M_S;
	platform->V_m_s[2] = 0;

	//расчет угловых скоростей коррекции положения с.к. LocalLevel [5.2]

	platform->Om_rad_s[0] = (-platform->V_m_s[1])/(R_EARTH_M + platform->h_m);
	platform->Om_rad_s[1] = platform->V_m_s[0]/(R_EARTH_M + platform->h_m) + U_EARTH_HZ * cos(platform->fi_rad);
	platform->Om_rad_s[2] = platform->V_m_s[0]/(R_EARTH_M + platform->h_m) * tan(platform->fi_rad) + U_EARTH_HZ * sin(platform->fi_rad);

	platform->F_c_m_s_s[0] = 0;
	platform->F_c_m_s_s[1] = 0;
	platform->F_c_m_s_s[2] = 0;
}

	// Уравнения Пуассона [6]

void Puasson(Nav_platform *platform)
{
	platform->C1[0][0] = 0;
	platform->C1[0][1] = -platform->Om_B_rad_s[2];
	platform->C1[0][2] = platform->Om_B_rad_s[1];
	platform->C1[1][0] = platform->Om_B_rad_s[2];
	platform->C1[1][1] = 0;
	platform->C1[1][2] = -platform->Om_B_rad_s[0];
	platform->C1[2][0] = -platform->Om_B_rad_s[1];
	platform->C1[2][1] = platform->Om_B_rad_s[0];
	platform->C1[2][2] = 0;

	platform->C2[0][0] = 0;
	platform->C2[0][1] = -(platform->Om_rad_s[2] + platform->Om_c_rad_s[2]); //k2
	platform->C2[0][2] = (platform->Om_rad_s[1] + platform->Om_c_rad_s[1]);
	platform->C2[1][0] = (platform->Om_rad_s[2] + platform->Om_c_rad_s[2]);
	platform->C2[1][1] = 0;
	platform->C2[1][2] = -(platform->Om_rad_s[0] + platform->Om_c_rad_s[0]);
	platform->C2[2][0] = -(platform->Om_rad_s[1] + platform->Om_c_rad_s[1]);
	platform->C2[2][1] = (platform->Om_rad_s[0] + platform->Om_c_rad_s[0]);
	platform->C2[2][2] = 0;

	mult_m_m(platform->C_B_LL, platform->C1, platform->C3);
	mult_m_num(platform->C3, DT_S, platform->C1);

	mult_m_m(platform->C2, platform->C_B_LL, platform->C3);
	mult_m_num(platform->C3, DT_S, platform->C2);

	sum_m_m(platform->C_B_LL, platform->C1, platform->C3);
	sub_m_m(platform->C3, platform->C2, platform->C_B_LL);

	platform->Om_c_rad_s[0] = 0;
	platform->Om_c_rad_s[1] = 0;
	platform->Om_c_rad_s[2] = 0;
}

	// Нормализация матрицы направляющих косинусов [7]

void Normalization(Nav_platform *platform)
{
	_FLOAT32 S = 0;
	_FLOAT32 Sum = 0;

	if (platform->Iteration % 2 == 0) // Для четных тактов
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Sum += sqr(platform->C_B_LL[i][j]);
			}
		}

		S = sqrt(Sum);

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				platform->C_B_LL[i][j] = platform->C_B_LL[i][j]/S;
			}
		}
	}
	else // Для нечетных тактов
	{
		for (int j = 0; j < 3; j++)
		{
			for (int i = 0; i < 3; i++)
			{
				Sum += sqr(platform->C_B_LL[i][j]);
			}
		}

		S = sqrt(Sum);

		for (int j = 0; j < 3; j++)
		{
			for (int i = 0; i < 3; i++)
			{
				platform->C_B_LL[i][j] = platform->C_B_LL[i][j]/S;
			}
		}
	}
}

	// Расчет углов ориентации [8]

void Orientation_Angles_Calc(Nav_platform *platform)
{
	platform->C0 = sqrt(sqr(platform->C_B_LL[2][0]) + sqr(platform->C_B_LL[2][2]));

	// Pitch

	platform->x = platform->C0;
	platform->y = platform->C_B_LL[2][1];

	if (platform->x == 0)
		if (platform->y > 0)
			platform->Pitch_rad = PI/2;
		else
			platform->Pitch_rad = -PI/2;
	else
		platform->Pitch_rad = atan(platform->y/platform->x);

	// Roll

	platform->x = platform->C_B_LL[2][2];
	platform->y = - platform->C_B_LL[2][0];

	if (platform->x == 0)
		if (platform->y > 0)
			platform->Roll_rad = PI/2;
		else
			platform->Roll_rad = -PI/2;
	else if (platform->x > 0)
		platform->Roll_rad = atan(platform->y/platform->x);
	else if (platform->y > 0)
		platform->Roll_rad = PI + atan(platform->y/platform->x);
	else
		platform->Roll_rad = - PI + atan(platform->y/platform->x);

	// Heading

	platform->x = platform->C_B_LL[1][1];
	platform->y = platform->C_B_LL[0][1];

	if (platform->x == 0)
		if (platform->y > 0)
			platform->Heading_rad = PI/2;
		else
			platform->Heading_rad = -PI/2;
	else if (platform->x > 0)
		platform->Heading_rad = atan(platform->y/platform->x);
	else if (platform->y > 0)
		platform->Heading_rad = PI + atan(platform->y/platform->x);
	else
		platform->Heading_rad = - PI + atan(platform->y/platform->x);

	if (platform->Heading_rad < 0)
		platform->Heading_rad += 2 * PI;
}

	// Расчет навигационных параметров [9]

void Nav_Param_Calc(Nav_platform *platform)
{
	platform->fi_rad = platform->fi_rad + (platform->V_m_s[1]/(R_EARTH_M +platform->h_m)) * DT_S;
	platform->lambda_rad = platform->lambda_rad + (platform->V_m_s[0]/((R_EARTH_M +platform->h_m) * cos(platform->fi_rad))) * DT_S;
}

    // Выставка

void Allignment(Nav_platform *platform)
{
	_FLOAT32 k = 1.0;

	platform->Om_c_rad_s[0] = -k * platform->F_m_s_s[1];
	platform->Om_c_rad_s[1] = k * platform->F_m_s_s[0];

	Acc_Calc(platform);
	Puasson(platform);
	Normalization(platform);
	Orientation_Angles_Calc(platform);

	if(platform->Temperature_deg_c != 0.00)
	{
		platform->Om_B_Av_rad_s[0] += (platform->Om_B_rad_s[0]);
		platform->Om_B_Av_rad_s[1] += (platform->Om_B_rad_s[1]);
		platform->Om_B_Av_rad_s[2] += (platform->Om_B_rad_s[2]);

		platform->F_B_av_m_s_s[0] += platform->F_B_m_s_s[0];
		platform->F_B_av_m_s_s[1] += platform->F_B_m_s_s[1];
		platform->F_B_av_m_s_s[2] += platform->F_B_m_s_s[2];
	}

}

	// Шаг навигационного алгоритма [4-9]

void Platform_Step(Nav_platform *platform)
{
	if (platform->Iteration < 0)
	{
		Allignment(platform);
	}
	else
	{
		Acc_Calc(platform);
		Velocity_Calc(platform);
		Puasson(platform);
		Normalization(platform);
		Orientation_Angles_Calc(platform);
		Nav_Param_Calc(platform);
	}
}






