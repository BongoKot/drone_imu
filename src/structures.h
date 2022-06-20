#include "types.h"

#ifndef STRUCTURES_H_
#define STRUCTURES_H_

#pragma pack(1)
typedef struct {

	_UINT32 Index;
	_Vector   F_m_s_s;               // ��������� �.�. LL
	_Vector   F_B_m_s_s;             // ��������� �.�. Body
	_Vector   F_B_av_m_s_s;
	_Vector   F_c_m_s_s;
	_Vector   Om_B_rad_s;            // ������� �������� �.�. Body
	_Vector   Om_B_Av_rad_s;         // ������� �������� ���� � ������ ��������� �� ����������� �������� �.�. Body
	_Vector   Om_c_rad_s;            // ������� �������� ��������� ��������� �.�. LL
	_Vector   V_m_s;                 // �������� ��������
	_Vector   Om_rad_s;              // ������� �������� �.�. LL
	_DOUBLE64 fi_rad;                // ������
	_DOUBLE64 lambda_rad;            // �������
	_DOUBLE64 fi0_rad;               // ��������� ��������� (������)
	_DOUBLE64 lambda0_rad;           // ��������� ��������� (�������)
	_FLOAT32  h_m;                   // ������
	_FLOAT32  C0;                    //
	_FLOAT32  Pitch_rad;             // ���� �����
	_FLOAT32  Roll_rad;              // ���� �������
	_FLOAT32  Heading_rad;           // ���� �����
	_FLOAT32  x, y;                  //
	_Matrix   C_B_LL;                // ������� ������������ ���������
	_Vector   F_deg;                 // ���� ���������������
	_SINT32   Iteration;    	     // ����� �����
	_FLOAT32  K_Radial;              // ����������� ���������� ���������
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

	// �������

	_SINT32   index;                   // ������ ������
	_FLOAT32  time_s;                  // ����� � ������ ������� (���)
	_FLOAT32  nav_time_s;              // ����� � ������ ��������� (���)
	_FLOAT32  roll_deg; 	 	       // ���� (����)
	_FLOAT32  pitch_deg;               // ������ (����)
	_FLOAT32  heading_deg;             // ���� (����)
	_FLOAT32  magnetic_heading_deg;    // ��������� ���� (����)
	_FLOAT32  ve_m_s;                  // ��������� �������� (�/�)
	_FLOAT32  vn_m_s;                  // �������� �������� (�/�)
	_FLOAT32  vup_m_s;                 // ������������ �������� (�/�)
	_FLOAT32  v_m_s;                   // ������ �������� (�/�)
	_DOUBLE64 latitude_deg;            // ������ (����)
	_DOUBLE64 longitude_deg;           // ������� (����)
	_FLOAT32  height_m;                // ������ (����)
	_FLOAT32  bar_height_m;            // ��������������� ������ (����)
	_FLOAT32  static_pressure_pa;      // ����������� �������� (�������)
	_Vector   omega_deg_s;             // ������� �������� X Y Z (����/�)
	_Vector   acceleration_m_s_s;      // ��������� X Y Z(�/�^2)
	_Vector   omega_raw_deg_s;         // ������� �������� X Y Z (����/�)
	_Vector   acceleration_raw_m_s_s;  // ��������� X Y Z(�/�^2)
	_FLOAT32  temperature_deg_c;       // ����������� � ������� (������� �������)

	_Vector  A;
	_Vector  B;
	_Matrix  M;

	//���

	_DOUBLE64 latitude_sns_deg;      // ������ ��� (����)
	_DOUBLE64 longitude_sns_deg;     // ������� ��� (����)
	_FLOAT32  v_sns_m_s;             // ������ �������� ��� (�/�)
	_FLOAT32  track_angle_sns_deg;   // ������� ���� ��� (����)
	_FLOAT32  height_sns_m;          // ������ ��� (����)
	_FLOAT32  PDOP;
	_FLOAT32  HDOP;
	_FLOAT32  VDOP;
	_FLOAT32  visible_sat;           // ���������� ������� ��������� ���
	_FLOAT32  usable_sat;            // ���������� ��������� ��� � �������

} Input_data;

typedef struct {

	_SINT32   index;
	_DOUBLE64 latitude_deg;
	_DOUBLE64 longtitude_deg;
	_FLOAT32  roll_deg; 	 	     // ���� (����)
	_FLOAT32  pitch_deg;             // ������ (����)
	_FLOAT32  heading_deg;           // ���� (����)
	_Vector   V_m_s;                 // �������� ��������

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

	_SINT32   index;                   // ������ ������
	_Vector   omega_deg_s;			   // ������� �������� X Y Z (����/�)
	_Vector   acceleration_m_s_s;      // ��������� X Y Z(�/�^2)
	_Vector   omega_av_deg_s;		   // ������� �������� X Y Z (����/�)
	_Vector   acceleration_av_m_s_s;   // ��������� X Y Z(�/�^2)
	_FLOAT32  temperature_deg_c;       // ����������� � ������� (������� �������)
	_FLOAT32  roll_deg; 	 	       // ���� (����)
	_FLOAT32  pitch_deg;               // ������ (����)
	_FLOAT32  heading_deg;             // ���� (����)

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
