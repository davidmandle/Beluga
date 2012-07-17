#include <math.h>

#include "BelugaTest.h"
#include "BelugaControl.h"

int testBelugaWaypointControlLaw()
{
    
    BelugaWaypointControlLaw control_law;
    control_law.doActivate(true);
	
    std::string err_msg;
	
    mt_dVector_t u_in, u_out, state;
    u_in.resize(BELUGA_WAYPOINT_SIZE, 0.0);
    state.resize(BELUGA_NUM_STATES, 0.0);
	
	/* the output should have BELUGA_CONTROL_SIZE elements */
    START_TEST("Checking output vector size");
    u_out = control_law.doControl(state, u_in);
    if (u_out.size() != BELUGA_CONTROL_SIZE)
        RETURN_ERROR_ELSE_OK("Incorrect size " << u_out.size());
	
    /* forward speed should max out when robot is far from waypoint */
	START_TEST("Checking that speed saturates for large distances");
    u_in[BELUGA_WAYPOINT_X] = state[BELUGA_STATE_X];
    u_in[BELUGA_WAYPOINT_Y] = state[BELUGA_STATE_Y] + 1.1*control_law.m_dDistThreshold;
    u_out = control_law.doControl(state, u_in);
    if (!eq_wf(u_out[BELUGA_CONTROL_FWD_SPEED], control_law.m_dMaxSpeed))
        RETURN_ERROR_ELSE_OK("Control did not saturate, was " << u_out[BELUGA_CONTROL_FWD_SPEED]);
	
	/* forward speed should decrease as robot approaches waypoint */
	START_TEST("Checking that speed decreases as robot nears waypoint");
    u_in[BELUGA_WAYPOINT_X] = state[BELUGA_STATE_X];
	mt_dVector_t prevOut;
	prevOut.resize(BELUGA_CONTROL_SIZE, 500.0);
	double N = 1.1;
	double step = 0.1;
	while (N > 0.0)
	{
		u_in[BELUGA_WAYPOINT_Y] = state[BELUGA_STATE_Y] + N*control_law.m_dDistThreshold;
		u_out = control_law.doControl(state, u_in);
		if (u_out[BELUGA_CONTROL_FWD_SPEED] > prevOut[BELUGA_CONTROL_FWD_SPEED])
			RETURN_ERROR_ELSE_OK("Speed did not decrease, was " << u_out[BELUGA_CONTROL_FWD_SPEED]);
		prevOut = u_out;
		N -= step;
	}
	
    return OK;
}

int testBelugaLowLevelControlLaw()
{
    BelugaLowLevelControlLaw control_law;
	
	std::string err_msg;
	
    mt_dVector_t u_in, u_out, state;
    u_in.resize(BELUGA_CONTROL_SIZE, 0.0);
    state.resize(4, 0.0);
	
	/* the output should have BELUGA_CONTROL_SIZE elements */
    START_TEST("Checking output vector size");
    u_out = control_law.doControl(state, u_in);
    if (u_out.size() != BELUGA_CONTROL_SIZE)
        RETURN_ERROR_ELSE_OK("Incorrect size " << u_out.size());

    /* the vertical thrust should be zero when u_vert is zero */
    START_TEST("Checking that the vertical thrust is zero if no vertical command is given");
	u_in = randomVector(3);
    state = randomVector(4);
    u_in[BELUGA_CONTROL_VERT_SPEED] = 0.0;
    state[BELUGA_STATE_Z] = z_off;
    u_out = control_law.doControl(state, u_in);
    if (u_out[BELUGA_CONTROL_VERT_SPEED] != 0.0)        
        RETURN_ERROR_ELSE_OK("Control was " << u_out[BELUGA_CONTROL_VERT_SPEED]);
    
    return OK;
}

int testBelugaBoundaryControlLaw()
{
    const char* bad_file_x = "bad_file.txt";
    const char* bad_file_y = "bad_file.txt";
    const char* good_file_x = "../src/Boundary_ForcesX.txt";
    const char* good_file_y = "../src/Boundary_ForcesY.txt";

    BelugaBoundaryControlLaw control_law(good_file_x, good_file_y);

    /* should fail to load bad files */
    DO_TEST("Checking for failure on loading bad files (x)",
            control_law.loadForceFiles(bad_file_x, good_file_y),
            "Didn't return false for " << bad_file_x);
    DO_TEST("Checking for failure on loading bad files (y)",
            control_law.loadForceFiles(good_file_x, bad_file_y),
            "Didn't return false for " << bad_file_y);
    DO_TEST("Checking for success on loading good files",
            !control_law.loadForceFiles(good_file_x, good_file_y),
            "Unable to load files " << good_file_x << " and " << good_file_y);
	
    std::string err_msg;
	
    mt_dVector_t u_in, u_out, state;
    u_in.resize(BELUGA_CONTROL_SIZE, 0.0);
    state.resize(BELUGA_NUM_STATES, 0.0);
	
	/* the output should have BELUGA_CONTROL_SIZE elements */
    START_TEST("Checking output vector size");
    u_out = control_law.doControl(state, u_in);
    if (u_out.size() != BELUGA_CONTROL_SIZE)
        RETURN_ERROR_ELSE_OK("Incorrect size " << u_out.size());
	
	/* robot speed should not change at origin */
	START_TEST("Checking origin");
	state[BELUGA_STATE_X] = 0.0;
	state[BELUGA_STATE_Y] = 0.0;
	double u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    double u_vert = u_in[BELUGA_CONTROL_VERT_SPEED];
    double u_turn = u_in[BELUGA_CONTROL_STEERING];
	u_out = control_law.doControl(state, u_in);
	double d_speed = fabs(u_speed - u_out[BELUGA_CONTROL_FWD_SPEED]);
	double d_vert = fabs(u_vert - u_out[BELUGA_CONTROL_VERT_SPEED]);
	double d_turn = fabs(u_turn - u_out[BELUGA_CONTROL_STEERING]);
	if (!eq_wf(d_speed, 0.0))
		RETURN_ERROR_ELSE_OK("Forward speed not zero, was " << u_out[BELUGA_CONTROL_FWD_SPEED]);
	if (!eq_wf(d_vert, 0.0))
		RETURN_ERROR_ELSE_OK("Vertical speed not zero, was " << u_out[BELUGA_CONTROL_VERT_SPEED]);
	if (!eq_wf(d_turn, 0.0))
		RETURN_ERROR_ELSE_OK("Steering not zero, was " << u_out[BELUGA_CONTROL_STEERING]);
	
	/* check robot's response to boundaries in x and y (up to buffer zone), along the respective axes */
	double minDist = 0.5;
	double step = 0.1;
	double boundary = DEFAULT_TANK_RADIUS/sqrt((double) 2.0);
	double buffer = 0.25;
	
	/* robot speed should decrease in x and not change in y the closer it gets to the +x boundary */
	START_TEST("Checking +x boundary");
	state[BELUGA_STATE_Y] = 0.0;
	state[BELUGA_STATE_THETA] = 0.0;
	double th = state[BELUGA_STATE_THETA];
	u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    u_turn = u_in[BELUGA_CONTROL_STEERING];
	double prev_vx = u_speed*cos(th) - u_turn*sin(th);
	double prev_vy = u_speed*sin(th) + u_turn*cos(th);
	double dist = minDist;
	while (dist < boundary - buffer)
	{
		state[BELUGA_STATE_X] = dist;
		u_out = control_law.doControl(state, u_in);
		double vr = u_out[BELUGA_CONTROL_FWD_SPEED];
		double vth = u_out[BELUGA_CONTROL_STEERING];
		double vx = vr*cos(th) - vth*sin(th);
		double vy = vr*sin(th) + vth*cos(th);
		if (vx > prev_vx)
			RETURN_ERROR_ELSE_OK("Speed did not decrease in x, was " << vx);
		double dvy = fabs(vy - prev_vy);
		if (!eq_wf(dvy, 0.0))
			RETURN_ERROR_ELSE_OK("Y-speed changed, d_vy was " << dvy);
		prev_vx = vx;
		prev_vy = vy;
		dist += step;
	}
	
	/* robot speed should increase in x and not change in y the closer it gets to the -x boundary */
	START_TEST("Checking -x boundary");
	state[BELUGA_STATE_Y] = 0.0;
	state[BELUGA_STATE_THETA] = PI;
	th = state[BELUGA_STATE_THETA];
	u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    u_turn = u_in[BELUGA_CONTROL_STEERING];
	prev_vx = u_speed*cos(th) - u_turn*sin(th);
	prev_vy = u_speed*sin(th) + u_turn*cos(th);
	dist = -minDist;
	while (dist > -(boundary - buffer))
	{
		state[BELUGA_STATE_X] = dist;
		u_out = control_law.doControl(state, u_in);
		double vr = u_out[BELUGA_CONTROL_FWD_SPEED];
		double vth = u_out[BELUGA_CONTROL_STEERING];
		double vx = vr*cos(th) - vth*sin(th);
		double vy = vr*sin(th) + vth*cos(th);
		if (vx < prev_vx)
			RETURN_ERROR_ELSE_OK("Speed did not increase in x, was " << vx);
		double dvy = fabs(vy - prev_vy);
		if (!eq_wf(dvy, 0.0))
			RETURN_ERROR_ELSE_OK("Y-speed changed, d_vy was " << dvy);
		prev_vx = vx;
		prev_vy = vy;
		dist -= step;
	}
	
	/* robot speed should decrease in y and not change in x the closer it gets to the +y boundary */
	START_TEST("Checking +y boundary");
	state[BELUGA_STATE_X] = 0.0;
	state[BELUGA_STATE_THETA] = PI/2;
	th = state[BELUGA_STATE_THETA];
	u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    u_turn = u_in[BELUGA_CONTROL_STEERING];
	prev_vx = u_speed*cos(th) - u_turn*sin(th);
	prev_vy = u_speed*sin(th) + u_turn*cos(th);
	dist = minDist;
	while (dist < boundary - buffer)
	{
		state[BELUGA_STATE_Y] = dist;
		u_out = control_law.doControl(state, u_in);
		double vr = u_out[BELUGA_CONTROL_FWD_SPEED];
		double vth = u_out[BELUGA_CONTROL_STEERING];
		double vx = vr*cos(th) - vth*sin(th);
		double vy = vr*sin(th) + vth*cos(th);
		if (vy > prev_vy)
			RETURN_ERROR_ELSE_OK("Speed did not decrease in y, was " << vy);
		double dvx = fabs(vx - prev_vx);
		if (!eq_wf(dvx, 0.0))
			RETURN_ERROR_ELSE_OK("X-speed changed, d_vy was " << dvx);
		prev_vx = vx;
		prev_vy = vy;
		dist += step;
	}
	
	/* robot speed should increase in y and not change in x the closer it gets to the -y boundary */
	START_TEST("Checking -y boundary");
	state[BELUGA_STATE_X] = 0.0;
	state[BELUGA_STATE_THETA] = (3/2)*PI;
	th = state[BELUGA_STATE_THETA];
	u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    u_turn = u_in[BELUGA_CONTROL_STEERING];
	prev_vx = u_speed*cos(th) - u_turn*sin(th);
	prev_vy = u_speed*sin(th) + u_turn*cos(th);
	dist = -minDist;
	while (dist > -(boundary - buffer))
	{
		state[BELUGA_STATE_Y] = dist;
		u_out = control_law.doControl(state, u_in);
		double vr = u_out[BELUGA_CONTROL_FWD_SPEED];
		double vth = u_out[BELUGA_CONTROL_STEERING];
		double vx = vr*cos(th) - vth*sin(th);
		double vy = vr*sin(th) + vth*cos(th);
		if (vy < prev_vy)
			RETURN_ERROR_ELSE_OK("Speed did not increase in y, was " << vy);
		double dvx = fabs(vx - prev_vx);
		if (!eq_wf(dvx, 0.0))
			RETURN_ERROR_ELSE_OK("X-speed changed, d_vy was " << dvx);
		prev_vx = vx;
		prev_vy = vy;
		dist -= step;
	}
	
    return OK;
}

int main(int argc, char** argv)
{
    RUN_TEST_SUITE("Waypoint Control", testBelugaWaypointControlLaw());
	RUN_TEST_SUITE("Low Level Control", testBelugaLowLevelControlLaw());
	RUN_TEST_SUITE("Boundary Control", testBelugaBoundaryControlLaw());
    
    std::cout << std::endl << "\tAll tests pass!" << std::endl;
    return OK;
}
