#ifndef BELUGADYNAMICS_H
#define BELUGADYNAMICS_H

#include <cv.h>

class BelugaDynamicsParameters
{
public:
    static double m_dDt;
    static double m_dWaterDepth;
	static double m_dK_t;
	static double m_dK_d1;
	static double m_dm_0;
	static double m_dm_1;
	static double m_dr_1;
	static double m_dK_omega;
	static double m_deta_up;
	static double m_deta_down;
	static double m_dv_off;
	static double m_dk_d;
	static double m_dz_off;
	static double m_dk_teth;
	static double m_dk_vp;
	static double m_dJ;
};

void beluga_dynamics(const CvMat* x_k,
                          const CvMat* u_k,
                          const CvMat* v_k,
                          CvMat* x_kplus1);

void beluga_measurement(const CvMat* x_k,
                             const CvMat* n_k,
                             CvMat* z_k);

void constrain_state(CvMat* x_k,
                            CvMat* X_p,
							double tank_radius,
							double water_depth);

#endif // BELUGADYNAMICS_H
