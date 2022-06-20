#include "types.h"

#ifndef STRUCTURES_H_
#define STRUCTURES_H_

#pragma pack(1)
typedef struct {

	_UINT32 Index;
	_Vector   F_m_s_s;               // Ускорения с.к. LL
	_Vector   F_B_m_s_s;             // Ускорения с.к. Body
	_Vector   F_B_av_m_s_s;
	_Vector   F_c_m_s_s;
	_Vector   Om_B_rad_s;            // Угловые скорости с.к. Body
	_Vector   Om_B_Av_rad_s;         // Среднее значение шума в углвых скоростях по результатам выставки с.к. Body
	_Vector   Om_c_rad_s;            // Угловые скорости коррекции положения с.к. LL
	_Vector   V_m_s;                 // Линейные скорости
	_Vector   Om_rad_s;              // Угловые скорости с.к. LL
	_DOUBLE64 fi_rad;                // Широта
	_DOUBLE64 lambda_rad;            // Долгота
	_DOUBLE64 fi0_rad;               // Начальное положение (широта)
	_DOUBLE64 lambda0_rad;           // Начальное положение (долгота)
	_FLOAT32  h_m;                   // Высота
	_FLOAT32  C0;                    //
	_FLOAT32  Pitch_rad;             // Угол крена
	_FLOAT32  Roll_rad;              // Угол тангажа
	_FLOAT32  Heading_rad;           // Угол курса
	_FLOAT32  x, y;                  //
	_Matrix   C_B_LL;                // Матрица направляющих косинусов
	_Vector   F_deg;                 // Углы рассогласования
	_SINT32   Iteration;    	     // Номер такта
	_FLOAT32  K_Radial;              // Коэффициент радиальной коррекции
	_FLOAT32  K1_SNS;
	_FLOAT32  K2_SNS;
	_FLOAT32  K3_SNS;
	_FLOAT32  K4_SNS;
	_FLOAT32  K5_SNS;

	_FLOAT32  Temperature_deg_c;

	_Matrix   C1;                    //
	_Matrix   C2;                    //
	_Matrix   C3;                    //

	_FLOAT32 Current_Temp_A_deg_c;
	_FLOAT32 Current_Temp_B_deg_c;
	_DoubleVector P1;
	_DoubleVector P2;
	_DoubleVector Corr_Value;

} Nav_platform;
#pragma pack()

typedef struct {

	// датчики

	_SINT32   index;                   // индекс данных
	_FLOAT32  time_s;                  // время с подачи питания (сек)
	_FLOAT32  nav_time_s;              // время в режиме навигации (сек)
	_FLOAT32  roll_deg; 	 	       // крен (град)
	_FLOAT32  pitch_deg;               // тангаж (град)
	_FLOAT32  heading_deg;             // курс (град)
	_FLOAT32  magnetic_heading_deg;    // магнитный курс (град)
	_FLOAT32  ve_m_s;                  // восточная скорость (м/с)
	_FLOAT32  vn_m_s;                  // северная скорость (м/с)
	_FLOAT32  vup_m_s;                 // вертикальная скорость (м/с)
	_FLOAT32  v_m_s;                   // модуль скорости (м/с)
	_DOUBLE64 latitude_deg;            // широта (град)
	_DOUBLE64 longitude_deg;           // долгота (град)
	_FLOAT32  height_m;                // высота (метр)
	_FLOAT32  bar_height_m;            // барометрическая высота (метр)
	_FLOAT32  static_pressure_pa;      // статическое давление (паскаль)
	_Vector   omega_deg_s;             // угловая скорость X Y Z (град/с)
	_Vector   acceleration_m_s_s;      // ускорение X Y Z(м/с^2)
	_Vector   omega_raw_deg_s;         // угловая скорость X Y Z (град/с)
	_Vector   acceleration_raw_m_s_s;  // ускорение X Y Z(м/с^2)
	_FLOAT32  temperature_deg_c;       // температура с датчика (градусы Цельсия)

	_Vector  A;
	_Vector  B;
	_Matrix  M;

	//снс

	_DOUBLE64 latitude_sns_deg;      // широта СНС (град)
	_DOUBLE64 longitude_sns_deg;     // долгота СНС (град)
	_FLOAT32  v_sns_m_s;             // модуль скорости СНС (м/с)
	_FLOAT32  track_angle_sns_deg;   // путевой угол СНС (град)
	_FLOAT32  height_sns_m;          // высота СНС (метр)
	_FLOAT32  PDOP;
	_FLOAT32  HDOP;
	_FLOAT32  VDOP;
	_FLOAT32  visible_sat;           // количество видимых спутников СНС
	_FLOAT32  usable_sat;            // количество спутников СНС в решении

} Input_data;

typedef struct {

	_SINT32   index;
	_DOUBLE64 latitude_deg;
	_DOUBLE64 longtitude_deg;
	_FLOAT32  roll_deg; 	 	     // крен (град)
	_FLOAT32  pitch_deg;             // тангаж (град)
	_FLOAT32  heading_deg;           // курс (град)
	_Vector   V_m_s;                 // Линейные скорости

} Output_data;

typedef struct {

	_Temp   T;
	_Matrix A;
	_Vector alpha;
	_Matrix B;
	_Vector beta;
	_Matrix AB;

} Cal_Passport;

typedef struct {

	_SINT32   index;                   // индекс данных
	_Vector   omega_deg_s;			   // угловая скорость X Y Z (град/с)
	_Vector   acceleration_m_s_s;      // ускорение X Y Z(м/с^2)
	_Vector   omega_av_deg_s;		   // угловая скорость X Y Z (град/с)
	_Vector   acceleration_av_m_s_s;   // ускорение X Y Z(м/с^2)
	_FLOAT32  temperature_deg_c;       // температура с датчика (градусы Цельсия)
	_FLOAT32  roll_deg; 	 	       // крен (град)
	_FLOAT32  pitch_deg;               // тангаж (град)
	_FLOAT32  heading_deg;             // курс (град)

	_Matrix int_omega;
	_FLOAT32 Current_Temp_A_deg_c;
	_FLOAT32 Current_Temp_B_deg_c;
	_DoubleVector P1;
	_DoubleVector P2;
	_DoubleVector Corr_Value;

	_Matrix buf0;
	_Matrix buf1;
	_Vector buf2;
	_Vector buf3;

}Cal_File;

#endif
