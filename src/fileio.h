#ifndef FILEIO_H_
#define FILEIO_H_

#include "structures.h"

void DataRead(Nav_platform *platform, Input_data *inp, FILE *RawData);
void DataWrite(Nav_platform *platform, Output_data *outp, FILE *ProData, Input_data *inp);

#endif /* FILEIO_H_ */
