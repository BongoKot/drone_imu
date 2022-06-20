#ifndef COEFFICIENTFINDER_H_
#define COEFFICIENTFINDER_H_

#include "structures.h"

void bin_file_writer(Cal_Passport *pas);
void bin_file_reader(Cal_Passport *pas);
void PasDataRead(Cal_Passport *pas);
void CalData_Averaging(Cal_File *f, FILE *file, Cal_Passport *pas);
void integrate(Cal_File *f, FILE *file, _SINT16 n);
void coef_finder(Cal_File *p1, Cal_File *p2, Cal_File *p3, Cal_File *p4, Cal_File *p5, Cal_File *p6, Cal_File *rot, Cal_Passport *pas);
void calibration(Cal_File *p1, Cal_File *p2, Cal_File *p3, Cal_File *p4, Cal_File *p5, Cal_File *p6, Cal_File *rot, Cal_Passport *pas);

#endif
