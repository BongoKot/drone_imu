#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "structures.h"
#include "correction.h"
#include "fileio.h"
#include "kernel.h"
#include "const.h"
#include "global_init.h"
#include "types.h"
#include "CoefficientFinder.h"

Nav_platform Platform_Corr;
Nav_platform Platform_Auto;
Cal_Passport Calp;
Input_data Inp;
Output_data Outp;

Cal_File Pos1;
Cal_File Pos2;
Cal_File Pos3;
Cal_File Pos4;
Cal_File Pos5;
Cal_File Pos6;

Cal_File Rot;

int main()
{

	bin_file_reader(&Calp);

	Global_Initialization(&Platform_Auto, &Platform_Corr, &Inp);

	FILE *RawData = fopen("D:/drone_imu/NavData.txt", "r");
	FILE *ProData = fopen("D:/drone_imu/DataProcessed.txt", "w");

	while (!feof(RawData))
	{
		if (RawData == NULL)
		{
			printf("file not found");
		}
		else
		{
			DataRead(&Platform_Auto, &Inp, RawData);
			//Passport_Applyment(&Calp, &Inp, &Platform_Auto);
			Input2Platform(&Inp, &Platform_Auto);
			Platform_Step(&Platform_Auto);
			if (Platform_Auto.Iteration <= 150000)
				Satellite_Correction(&Platform_Auto, &Inp);
			else
				Radial_Correction(&Platform_Auto);
			DataWrite(&Platform_Auto, &Outp, ProData, &Inp);

			Platform_Auto.Iteration++;
		}
	}

	printf("EOF reached!\n");

	fclose(RawData);
	fclose(ProData);

//	calibration(&Pos1, &Pos2, &Pos3, &Pos4, &Pos5, &Pos6, &Rot, &Calp);

	return 0;
}
