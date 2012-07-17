#include <math.h>
#include <iostream>
#include <fstream>
#ifndef MT_CLAMP
#define MT_CLAMP(x, a, b) (((x) > (b)) ? (b) : (((x) < (a)) ? (a) : (x)))
#endif

#include "BelugaControl.h"

std::string BelugaWaypointControlLaw::s_sName("Beluga Waypoint Controller\n");
std::string BelugaLowLevelControlLaw::s_sName("Beluga Low Level Controller\n");
std::string BelugaBoundaryControlLaw::s_sName("Beluga Boundary Controller\n");

BelugaWaypointControlLaw* belugaWaypointControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num)
{
    return new BelugaWaypointControlLaw();
}

BelugaLowLevelControlLaw* belugaLowLevelControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num)
{
    return new BelugaLowLevelControlLaw();
}

BelugaBoundaryControlLaw* belugaBoundaryControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num,
                                                          const char* force_file_name_x,
                                                          const char* force_file_name_y)
{
    return new BelugaBoundaryControlLaw(force_file_name_x, force_file_name_y);
}


BelugaWaypointControlLaw::BelugaWaypointControlLaw()  // includes HITL timing control (instead of having a separate ControlLaw)
    : mt_ControlLaw(3 /* # control inputs */,
                    4 /* # parameters */),
      m_bActive(false),
      m_dTiming(12500),	      // time for robot to travel between waypoints (msec), set in js and updated during IPC exchange
      m_dMaxSpeed(0.75),      // speed robot travels when more than 'm_dDistThreshold' away from waypoint (m/s), updated during IPC exchange
	  m_dDistThreshold(0.5),  // distance from waypoint at which robot starts to decrease speed from max (m)
      m_dTurningGain(10.0)
{
}

mt_dVector_t BelugaWaypointControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
    if (!m_bActive || state.size() < BELUGA_NUM_STATES || u_in.size() < BELUGA_WAYPOINT_SIZE)
    {
        return u_in;
    }

    mt_dVector_t u(BELUGA_CONTROL_SIZE, 0.0);

    double x = state[BELUGA_STATE_X];
    double y = state[BELUGA_STATE_Y];
    double z = state[BELUGA_STATE_Z];
    double th = state[BELUGA_STATE_THETA];
    double omega = state[BELUGA_STATE_OMEGA];

    double to_x = u_in[BELUGA_WAYPOINT_X];
    double to_y = u_in[BELUGA_WAYPOINT_Y];
    double to_z = u_in[BELUGA_WAYPOINT_Z];

	if (to_x == BELUGA_WAYPOINT_NONE || to_y == BELUGA_WAYPOINT_NONE)
	{
		return u;
	}

    if (to_z BELUGA_MAINTAIN_Z)
    {
        to_z = z;
    }

    double dx = to_x - x;
    double dy = to_y - y;
    double dz = to_z - z;

    double dth = atan2(dy, dx) - th;
	dth = atan2(sin(dth), cos(dth));

    double d = sqrt(dx*dx + dy*dy);
    double u_speed = 0;
    double u_vert = 0;
    double u_turn = 0;
	
	if (0 /*d < m_dDist*/)
    {
        m_bActive = false;
    }
    else
    {
		if (d > m_dDistThreshold)
        {
            u_speed = m_dMaxSpeed;
        }
        else
        {
            u_speed = m_dMaxSpeed*(d/m_dDistThreshold)*fabs(cos(dth));
        }
        u_turn = -m_dTurningGain*sin(dth);
		if (dth > 1.57)
		{
			u_turn = -m_dTurningGain;
		}
		if (dth < -1.57)
		{
			u_turn = m_dTurningGain;
		}
    }

    u[BELUGA_CONTROL_FWD_SPEED] = u_speed;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vert;
    u[BELUGA_CONTROL_STEERING] = u_turn;
    
	//printf("dx = %f, dy = %f, dth = %f, dz = %f\n", dx, dy, dth, dz);
	//printf("Control out: speed %f, vert %f, steer %f\n", u[BELUGA_CONTROL_FWD_SPEED], u[BELUGA_CONTROL_VERT_SPEED], u[BELUGA_CONTROL_STEERING]);

    return u;
}

BelugaLowLevelControlLaw::BelugaLowLevelControlLaw()
    : mt_ControlLaw(3 /* # control inputs */,
                    0 /* # parameters */),
      m_bActive(true)
{
}

mt_dVector_t BelugaLowLevelControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
    if (!m_bActive || state.size() < 4 || u_in.size() < BELUGA_CONTROL_SIZE)
    {
        return u_in;
    }

    mt_dVector_t u(BELUGA_CONTROL_SIZE, 0.0);
    
    double u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    double u_vert = u_in[BELUGA_CONTROL_VERT_SPEED];
    double u_turn = u_in[BELUGA_CONTROL_STEERING];

    double u_thrust = (K_d1/K_t)*u_speed;
	double speed_normalization_factor = 0.2;
	if (fabs(u_speed) > BELUGA_MIN_TURNING_SPEED)
	{
		speed_normalization_factor = u_speed;
	}
	else
	{
		if (u_speed < 0)
		{
			speed_normalization_factor *= -1.0;
		}
	}
    double u_steer = (K_omega/(K_d1*speed_normalization_factor*r_1))*u_turn;
    double u_vthrust = 0;

    if (u_vert != 0.0)
    {
        /* need z in the vertical thrust controller */
        double z = state[BELUGA_STATE_Z];
    
        /* calculating the vertical thrust requires finding the roots of
         * the polynomial u^2 + k_vp*u - (S/eta_x)*g, g defined here,
         * eta_x either eta_up or eta_down, and S either +1 (eta_up) or
         * -1 (eta_down).  One of the two combinations should give a positive
         * result - the corresponding thrust has this magnitude and is
         * negative if the eta_down (S = -1) solution is taken.  */
        double g = (k_d*u_vert*fabs(u_vert) - K_t*(z_off - z))*(fabs(u_vert) + v_off);

        /* determinants for polynomial */
        double D_up = k_vp*k_vp + 4*g/eta_up;
        double D_down = k_vp*k_vp - 4*g/eta_down;

        /* roots - note we'll throw out the -'ve root (i.e. (-b - sqrt(D))/2a */
        double r_up = -0.5*k_vp;  // just -b part
        double r_down = r_up;     // same

        /* can only calculate the determinant part if D > 0 */
        if (D_up > 0)
        {
            r_up += 0.25*sqrt(D_up);
        }
        if (D_down > 0)
        {
            r_down += 0.25*sqrt(D_down);
        }

        /* note we favor up control if both have solutions */
        if (D_down > 0)
        {
            u_vthrust = -r_down;
        }
        if (D_up > 0)
        {
            u_vthrust = r_up;
        }
    }

    u[BELUGA_CONTROL_FWD_SPEED] = u_thrust;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vthrust;
    u[BELUGA_CONTROL_STEERING] = u_steer;

	//printf("Control out: thrust %f, v_thrust %f, turn %f\n", u[BELUGA_CONTROL_FWD_SPEED], u[BELUGA_CONTROL_VERT_SPEED], u[BELUGA_CONTROL_STEERING]);

	return u;
}

BelugaBoundaryControlLaw::BelugaBoundaryControlLaw(const char* force_file_name_x,
                                                   const char* force_file_name_y)
	: mt_ControlLaw(3 /* # control inputs */,
					1 /* # parameters */),
	  m_bActive(true),
	  m_dGain(1e-6),    // must be calibrated for robots in tank
	  m_dLastFx(0),
	  m_dLastFy(0)
{
    if (!loadForceFiles(force_file_name_x, force_file_name_y))
    {
        std::cerr << "BelugaBoundaryControlLaw Error: Unable to load force files.  Controller will be inactive." << std::endl;
        m_bActive = false;
    }
}

bool BelugaBoundaryControlLaw::loadForceFiles(const char* force_file_name_x,
                                              const char* force_file_name_y)
{
    /* check that files are there */
	if (!force_file_name_x)
    {
        std::cerr << "BelugaBoundaryControlLaw::loadForceFiles Error: No X input file name" << std::endl;
        return false;
    }
    if (!force_file_name_y)
    {
        std::cerr << "BelugaBoundaryControlLaw::loadForceFiles Error: No Y input file name" << std::endl;
        return false;
    }
    
	/* read boundary forces from files and store in arrays */
	std::ifstream forcesX(force_file_name_x);
	std::ifstream forcesY(force_file_name_y);

    bool load_ok = true;
    if (!forcesX || !forcesX.good())
    {
        std::cerr << "BelugaBoundaryControlLaw::loadForceFiles Error: Could not load X file '" <<
            force_file_name_x << "'" << std::endl;
        load_ok = false;
    }
    if (!forcesY || !forcesY.good())
    {
        std::cerr << "BelugaBoundaryControlLaw::loadForceFiles Error: Could not load Y file '" <<
            force_file_name_y << "'" << std::endl;
        load_ok = false;
    }
    if (!load_ok)
    {
        return false;
    }

    double f = 0;
    unsigned int n_loaded = 0;
    for (unsigned int j = 0; forcesX.good() && j < 2*N_FORCE_GRID+1; j++)
    {
        for (unsigned int i = 0; forcesY.good() && i < 2*N_FORCE_GRID+1; i++)
        {
            forcesX >> f;
            FX[i][j] = f;
            forcesY >> f;
            FY[i][j] = f;
            n_loaded++;
        }
    }
	
	forcesX.close();
	forcesY.close();
	
	/* check that all forces were stored */
    unsigned int n_expected = (2*N_FORCE_GRID + 1)*(2*N_FORCE_GRID + 1);
    if (n_loaded < n_expected)
    {
        std::cerr << "BelugaBoundaryControlLaw::loadForceFiles Error: Only loaded " <<
            n_loaded << " data points.  Expected " << n_expected << std::endl;
        return false;
    }

    return true;
}

void BelugaBoundaryControlLaw::getLastForce(double* fx, double* fy) 
{
	*fx = m_dLastFx;
	*fy = m_dLastFy;
}

mt_dVector_t BelugaBoundaryControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
	if (!m_bActive || state.size() < BELUGA_NUM_STATES || u_in.size() < BELUGA_CONTROL_SIZE)
    {
        return u_in;
    }
	
    mt_dVector_t u(BELUGA_CONTROL_SIZE, 0.0);
	
	double x = state[BELUGA_STATE_X];
    double y = state[BELUGA_STATE_Y];
	double th = state[BELUGA_STATE_THETA];
	
	double u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    double u_vert = u_in[BELUGA_CONTROL_VERT_SPEED];
    double u_turn = u_in[BELUGA_CONTROL_STEERING];
	
	/* get boundary force corresponding to grid point closest to robot */
	double rmax = 1.3*DEFAULT_TANK_RADIUS;
	double step = rmax/N_FORCE_GRID;
	unsigned int i = MT_CLAMP((floor(x/step + 0.5) + N_FORCE_GRID), 0, 2*N_FORCE_GRID);
	unsigned int j = MT_CLAMP((floor(y/step + 0.5) + N_FORCE_GRID), 0, 2*N_FORCE_GRID);
	double fx = FX[i][j];
	double fy = FY[i][j];
	
	m_dLastFx = fx;  // for BelugaTrackerGUI visualization
	m_dLastFy = fy;
	
	/* calculate control parameters */
	double ax = m_dGain*fx/m_eff;
	double ay = m_dGain*fy/m_eff;
	double dt = BELUGA_MIN_COMMAND_PERIOD_MSEC;
	double dvx = ax*dt;
	double dvy = ay*dt;
	
	/* convert control parameters (dvx & dvy) to the robot-body coordinates (dvr & dvth) */
	double dvr = dvx*cos(th) + dvy*sin(th);
	double dvth = -dvx*sin(th) + dvy*cos(th);

	/* add in control parameters */
	u_speed += dvr;
	if (u_speed < 0)
	{
		u_speed = 0;  // don't let robot drive in reverse
	}
	
	double boundary = DEFAULT_TANK_RADIUS/sqrt((double) 2.0);
	double buffer = 0.25;
	double minSpeed = 0.1;
	double maxTurn = 0.05*BELUGA_MAX_TURN;  // adjust 'buffer', 'minSpeed', and 'maxTurn' for optimum performance
	
	u_turn += dvth;
	if ((boundary - fabs(x) < buffer || boundary - fabs(y) < buffer) && u_speed < minSpeed)
	{
		u_turn = maxTurn;  // to help robot escape from regions near boundaries
	}
	
	u[BELUGA_CONTROL_FWD_SPEED] = u_speed;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vert;
    u[BELUGA_CONTROL_STEERING] = u_turn;
    
	//printf("Control out: speed %f, steer %f\n", u[BELUGA_CONTROL_FWD_SPEED], u[BELUGA_CONTROL_STEERING]);
	
    return u;
}
