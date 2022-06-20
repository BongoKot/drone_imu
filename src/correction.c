#include "types.h"
#include "const.h"
#include "structures.h"
#include "functions.h"
#include <math.h>
#include "global_var.h"

_FLOAT32 V_E_SNS_M_S;
_FLOAT32 V_N_SNS_M_S;

_DOUBLE64 TL_Common_WrapHeading_minus180_180_deg(_DOUBLE64 H_deg)
{
  _UINT32 SafeCounter = 100;
  while (H_deg > 180 && SafeCounter > 0)
  {
    H_deg -= 360;
    SafeCounter--;
  }

  SafeCounter = 100;
  while (H_deg < -180 && SafeCounter > 0)
  {
    H_deg += 360;
    SafeCounter--;
  }

  return H_deg;
}

// ---------------------------------------------------------------------------------

// returns the shortest arc between H1 and H2 directions([-180; 180])
_DOUBLE64 TL_Common_DeltaHeading_deg(_DOUBLE64 H2_deg, _DOUBLE64 H1_deg)
{
  _DOUBLE64 DeltaH_deg;

  _DOUBLE64 _H1_deg = TL_Common_WrapHeading_minus180_180_deg(H1_deg);
  _DOUBLE64 _H2_deg = TL_Common_WrapHeading_minus180_180_deg(H2_deg);

  DeltaH_deg = (_H2_deg - _H1_deg);

  if (DeltaH_deg > 180.0)
    return DeltaH_deg - 360.0;
  else if (DeltaH_deg < -180.0)
    return DeltaH_deg + 360.0;
  else
    return DeltaH_deg;
}

void Radial_Correction(Nav_platform *platform)
{
	if (fabs(platform->F_m_s_s[0]) > 1)
		platform->K_Radial = 0.01;
	if (fabs(platform->F_m_s_s[0]) < 1)
		platform->K_Radial = 0.05;

	platform->Om_c_rad_s[0] = -platform->K_Radial * platform->F_m_s_s[1];
	platform->Om_c_rad_s[1] = platform->K_Radial * platform->F_m_s_s[0];
}

void Satellite_Correction(Nav_platform *platform, Input_data *inp)
{

	V_E_SNS_M_S = inp->v_sns_m_s * sin(Deg2Rad_rad(inp->track_angle_sns_deg));
	V_N_SNS_M_S = inp->v_sns_m_s * cos(Deg2Rad_rad(inp->track_angle_sns_deg));

	platform->K1_SNS = 2*KSI*TAU_S/(2*PI);
	platform->K2_SNS = TAU_S/(2*PI) * TAU_S/(2*PI) / G_M_S_S ;

	if (/*v_len(*/inp->v_sns_m_s/*)*/ > 10 && fabs(platform->Om_B_rad_s[2]) < 0.017)
	{
		platform->K3_SNS = 0.1;
	}
	else
		platform->K3_SNS = 0;

	platform->K4_SNS = 0.75;
	platform->K5_SNS = 0.2;

	platform->F_c_m_s_s[0] = - platform->K1_SNS * (platform->V_m_s[0] - V_E_SNS_M_S);
	platform->F_c_m_s_s[1] = - platform->K1_SNS * (platform->V_m_s[1] - V_N_SNS_M_S);

	platform->Om_c_rad_s[0] = - platform->K2_SNS * (platform->V_m_s[1] - V_N_SNS_M_S);
	platform->Om_c_rad_s[1] = + platform->K2_SNS * (platform->V_m_s[0] - V_E_SNS_M_S);
	platform->Om_c_rad_s[2] = - platform->K3_SNS *
	Deg2Rad_rad(TL_Common_DeltaHeading_deg(Rad2Deg_deg(platform->Heading_rad), inp->track_angle_sns_deg));

	platform->fi_rad -= platform->K4_SNS * (platform->fi_rad - Deg2Rad_rad(inp->latitude_sns_deg));
	platform->lambda_rad -= platform->K4_SNS * (platform->lambda_rad - Deg2Rad_rad(inp->longitude_deg));
}
