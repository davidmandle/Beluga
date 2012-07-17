#ifndef BELUGA_CONTROL_H
#define BELUGA_CONTROL_H

#include "BelugaConstants.h"
#include "Controller.h"

class BelugaWaypointControlLaw : public mt_ControlLaw
{
public:
    BelugaWaypointControlLaw();

    mt_dVector_t doControl(const mt_dVector_t& state,
                           const mt_dVector_t& u_in);
	
    bool doActivate(bool value = true){m_bActive = value;  return m_bActive;};
    
	double m_dTiming;
	double m_dMaxSpeed;
	double m_dDistThreshold;
    double m_dTurningGain;
    
protected:
    bool m_bActive;
    
    static std::string s_sName;
};

class BelugaLowLevelControlLaw : public mt_ControlLaw
{
public:
    BelugaLowLevelControlLaw();

    mt_dVector_t doControl(const mt_dVector_t& state,
                           const mt_dVector_t& u_in);

    bool doActivate(bool value = true){m_bActive = value;  return m_bActive;};

protected:
    bool m_bActive;
    
    static std::string s_sName;
};

class BelugaBoundaryControlLaw : public mt_ControlLaw
{
public:
    BelugaBoundaryControlLaw(const char* force_file_name_x, const char* force_file_name_y);
	
    mt_dVector_t doControl(const mt_dVector_t& state,
                           const mt_dVector_t& u_in);
	
    bool doActivate(bool value = true){m_bActive = value;  return m_bActive;};
    bool loadForceFiles(const char* force_file_name_x, const char* force_file_name_y);
	
	double m_dGain;

	void getLastForce(double *fx, double* fy);

protected:
    bool m_bActive;
	
	static const unsigned int N_FORCE_GRID = 20;
	double FX[2*N_FORCE_GRID+1][2*N_FORCE_GRID+1];
	double FY[2*N_FORCE_GRID+1][2*N_FORCE_GRID+1];

	double m_dLastFx;
	double m_dLastFy;
	
    static std::string s_sName;
};

BelugaWaypointControlLaw* belugaWaypointControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num);
BelugaLowLevelControlLaw* belugaLowLevelControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num);
BelugaBoundaryControlLaw* belugaBoundaryControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num,
                                                          const char* force_file_name_x,
                                                          const char* force_file_name_y);

#endif // BELUGA_CONTROL_H
