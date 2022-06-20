#include <stdio.h>
#include "kernel.h"
#include "types.h"
#include "const.h"
#include "functions.h"
#include "structures.h"
#include <math.h>
#include "global_var.h"

// Считывание данных из файла построчно, запись в массив, из массива форматным считыванием запись в переменные структуры

void DataRead(Nav_platform *platform, Input_data *inp, FILE *RawData)
{

	_SINT8 buffer[IN_BUFF_SIZE];
	/*
	const _SINT8 fmt[] = "%ld %f %f %f %f %f %f %f %f %f %f\n";
	fgets(buffer, IN_BUFF_SIZE, RawData);
	sscanf(buffer, fmt, &inp->index,
						&inp->omega_raw_deg_s[0],
						&inp->omega_raw_deg_s[1],
						&inp->omega_raw_deg_s[2],
						&inp->acceleration_raw_m_s_s[0],
						&inp->acceleration_raw_m_s_s[1],
						&inp->acceleration_raw_m_s_s[2],
						&inp->temperature_deg_c,
						&inp->roll_deg,
						&inp->pitch_deg,
						&inp->heading_deg);
	*/
	const _SINT8 fmt[] = "%ld %f %f %f %f %f %f %f %f %f %f %lf %lf %f %f %f %f %f %f %f %f %f %lf %lf %f %f %f %f %f %f %f %f\n";

	fgets(buffer, IN_BUFF_SIZE, RawData);
	sscanf(buffer, fmt, &inp->index,
						&inp->time_s,
						&inp->nav_time_s,
						&inp->roll_deg,
						&inp->pitch_deg,
						&inp->heading_deg,
						&inp->magnetic_heading_deg,
						&inp->ve_m_s,
						&inp->vn_m_s,
						&inp->vup_m_s,
						&inp->v_m_s,
						&inp->latitude_deg,
						&inp->longitude_deg,
						&inp->height_m,
						&inp->bar_height_m,
						&inp->static_pressure_pa,
						&inp->omega_deg_s[0],
						&inp->omega_deg_s[1],
						&inp->omega_deg_s[2],
						&inp->acceleration_m_s_s[0],
						&inp->acceleration_m_s_s[1],
						&inp->acceleration_m_s_s[2],
						&inp->latitude_sns_deg,
						&inp->longitude_sns_deg,
						&inp->v_sns_m_s,
						&inp->track_angle_sns_deg,
						&inp->height_sns_m,
						&inp->PDOP,
						&inp->HDOP,
						&inp->VDOP,
						&inp->visible_sat,
						&inp->usable_sat);
	inp->track_angle_sns_deg += 360;

}

void DataWrite(Nav_platform *platform, Output_data *outp, FILE *ProData, Input_data *inp)
{/*
	const _SINT8 output_fmt[] = "\n%ld\t %lf\t %lf\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f %f\t %f\t %f\t %f\t %f";

	if (ALIGNMENT_TIME_SEC * FREQ_HZ + platform->Iteration == 0)
	{
		//fprintf(ProData, "Iteration\t temp\t omega X\t omega Y\t omega Z\t Fx\t Fy\t Fz\t roll\t pitch\t heading");
	}
	fprintf(ProData, output_fmt, ALIGNMENT_TIME_SEC * FREQ_HZ + platform->Iteration,
												 inp->temperature_deg_c,                    //1
												 Rad2Deg_deg(platform->Om_B_rad_s[0]),      //2
												 Rad2Deg_deg(platform->Om_B_rad_s[1]),      //3
												 Rad2Deg_deg(platform->Om_B_rad_s[2]),      //4
												 platform->F_B_m_s_s[0],                    //5
												 platform->F_B_m_s_s[1],                    //6
												 platform->F_B_m_s_s[2],                    //7
												 Rad2Deg_deg(platform->Roll_rad),           //8
												 Rad2Deg_deg(platform->Pitch_rad),          //9
												 Rad2Deg_deg(platform->Heading_rad),   	    //10
												 Rad2Deg_deg(platform->fi_rad),             //11
												 Rad2Deg_deg(platform->lambda_rad),         //12
												 platform->V_m_s[0],                        //13
												 platform->V_m_s[1],                        //14
												 platform->V_m_s[2]);                       //15

												 */

	const _SINT8 output_fmt[] = "\n%ld\t %lf\t %lf\t %f\t %f\t %f\t %f\t %f\t %f";

	//if (ALIGNMENT_TIME_SEC * FREQ_HZ + platform->Iteration == 0)
	//	fprintf(ProData, "Iteration\t latitude\t longtitude\t Roll\t Pitch\t Heading\t VE\t VN\t VUp");

	fprintf(ProData, output_fmt, ALIGNMENT_TIME_SEC * FREQ_HZ + platform->Iteration,
											 Rad2Deg_deg(platform->fi_rad),
											 Rad2Deg_deg(platform->lambda_rad),
											 Rad2Deg_deg(platform->Roll_rad),
											 Rad2Deg_deg(platform->Pitch_rad),
											 Rad2Deg_deg(platform->Heading_rad),
											 //inp->track_angle_sns_deg,
											 platform->V_m_s[0],
											 platform->V_m_s[1],
											 platform->V_m_s[2]);

}
