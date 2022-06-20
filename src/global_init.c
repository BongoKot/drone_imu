#include <stdio.h>
#include "const.h"
#include "structures.h"
#include "functions.h"
#include "kernel.h"

	// Инициализация, происходит присвоение начальных значений переменных

void Platform_Initialization(Nav_platform *platform, Input_data *inp)
{
	platform->Om_B_Av_rad_s[0] = 0;
	platform->Om_B_Av_rad_s[1] = 0;
	platform->Om_B_Av_rad_s[2] = 0;

	platform->Pitch_rad = 0;
	platform->Roll_rad = 0;
	platform->Heading_rad = 0;

	platform->F_m_s_s[0] = 0;
	platform->F_m_s_s[1] = 0;
	platform->F_m_s_s[2] = 0;

	platform->Om_rad_s[0] = 0;
	platform->Om_rad_s[1] = 0;
	platform->Om_rad_s[2] = 0;

	platform->F_c_m_s_s[0] = 0;
	platform->F_c_m_s_s[1] = 0;
	platform->F_c_m_s_s[2] = 0;

	platform->Om_c_rad_s[0] = 0;
	platform->Om_c_rad_s[1] = 0;
	platform->Om_c_rad_s[2] = 0;

	platform->V_m_s[0] = 0;
	platform->V_m_s[1] = 0;
	platform->V_m_s[2] = 0;

	platform->C_B_LL[0][0] = 1;
	platform->C_B_LL[0][1] = 0;
	platform->C_B_LL[0][2] = 0;
	platform->C_B_LL[1][0] = 0;
	platform->C_B_LL[1][1] = 1;
	platform->C_B_LL[1][2] = 0;
	platform->C_B_LL[2][0] = 0;
	platform->C_B_LL[2][1] = 0;
	platform->C_B_LL[2][2] = 1;

	platform->fi0_rad =  Deg2Rad_rad(55);
	platform->lambda0_rad = Deg2Rad_rad(37);

	platform->fi_rad = platform->fi0_rad;
	platform->lambda_rad = platform->lambda0_rad;

	platform->Current_Temp_A_deg_c = 0;
	platform->Current_Temp_B_deg_c = 0;

	platform->Iteration = 	-ALIGNMENT_TIME_SEC * FREQ_HZ;

}

void Global_Initialization(Nav_platform *pl1, Nav_platform *pl2, Input_data *inp)
{
	Platform_Initialization(pl1, inp);
	Platform_Initialization(pl2, inp);

	printf("Initialization done!\n");
}
