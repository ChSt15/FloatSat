/*
 * ypr.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef YPR_H_
#define YPR_H_

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

template <typename TYPE>
class Vector3D_;
template <typename TYPE>
class Matrix3D_;
template <typename TYPE>
class Quaternion_;
template <typename TYPE>
class AngleAxis_;

typedef enum{
	UNIT_RAD,
	UNIT_DEGREE,
	UNIT_PI
} print_format_t;

// https://de.wikipedia.org/wiki/Eulersche_Winkel#Drehfolgen_in_der_Fahrzeugtechnik
template <typename TYPE = double>
class YPR_ {
public:
    TYPE yaw;
    TYPE pitch;
    TYPE roll;

    YPR_() {
        this->yaw   = 0;
        this->pitch = 0;
        this->roll  = 0;
    }

    YPR_(const YPR_& other) {
        this->yaw   = other.yaw;
        this->pitch = other.pitch;
        this->roll  = other.roll;
    }

    YPR_(const TYPE &yaw,const TYPE &pitch,const TYPE &roll) {
        this->yaw   = yaw;
        this->pitch = pitch;
        this->roll  = roll;
    }

    YPR_(const Quaternion_<TYPE>& q) {
        TYPE m21 = 2*q.q.x*q.q.y + 2*q.q0*q.q.z;
        TYPE m11 = 2*q.q0*q.q0-1 + 2*q.q.x*q.q.x;
        TYPE m31 = 2*q.q.x*q.q.z - 2*q.q0*q.q.y;
        TYPE m32 = 2*q.q.y*q.q.z + 2*q.q0*q.q.x;
        TYPE m33 = 2*q.q0*q.q0-1 + 2*q.q.z*q.q.z;

        this->pitch = atan2(-m31, sqrt(m11*m11 + m21*m21));
        this->yaw   = atan2(m21, m11);
        this->roll  = atan2(m32, m33);
    }

    YPR_(const Matrix3D_<TYPE>& M) {
        TYPE m21 = M.r[1][0];
        TYPE m11 = M.r[0][0];
        TYPE m31 = M.r[2][0];
        TYPE m32 = M.r[2][1];
        TYPE m33 = M.r[2][2];

        this->pitch = atan2(-m31, sqrt(m11*m11 + m21*m21));
        this->yaw   = atan2(m21, m11);
        this->roll  = atan2(m32, m33);
    }

    YPR_(const AngleAxis_<TYPE>& other) {

        Vector3D_<TYPE> u = other.u;
        TYPE phi = other.phi;
        TYPE cp  = cos(phi);
        TYPE sp  = sin(phi);


        TYPE m21 = u.x * u.y *(1-cp) + u.z*sp;
        TYPE m11 = u.x * u.x *(1-cp) + cp;
        TYPE m31 = u.x * u.z *(1-cp) - u.y*sp;
        TYPE m32 = u.y * u.z *(1-cp) + u.x*sp;
        TYPE m33 = u.z * u.z *(1-cp) + cp;

        this->pitch = atan2(-m31, sqrt(m11*m11 + m21*m21));
        this->yaw   = atan2(m21, m11);
        this->roll  = atan2(m32, m33);
    }

    YPR_<TYPE>  scale(const TYPE &factor) const {
        YPR_<TYPE> scale;
        scale.pitch = pitch * factor;
        scale.yaw   = yaw   * factor;
        scale.roll  = roll  * factor;
        return scale;
    }

    YPR_<TYPE> accumulateRotation(YPR_<TYPE>& increment) {
        Quaternion_<TYPE> orig(*this);
        Quaternion_<TYPE> inc(increment);
        orig = orig * inc;
        return orig.toYPR();
    }

    Matrix3D_<TYPE> toMatrix3D() const {
        Matrix3D_<TYPE> M;
        TYPE cy = cos(yaw);
        TYPE cp = cos(pitch);
        TYPE cr = cos(roll);
        TYPE sy = sin(yaw);
        TYPE sp = sin(pitch);
        TYPE sr = sin(roll);

        M.r[0][0]= cy*cp;
        M.r[0][1]= cy*sp*sr - sy*cr;
        M.r[0][2]= cy*sp*cr + sy*sr;

        M.r[1][0]= sy*cp;
        M.r[1][1]= sy*sp*sr + cy*cr;
        M.r[1][2]= sy*sp*cr - cy*sr;

        M.r[2][0]= -sp;
        M.r[2][1]= cp*sr;
        M.r[2][2]= cp*cr;

        return M;
    }

    Quaternion_<TYPE> toQuaternion() const {
        Quaternion_<TYPE> q;
        TYPE cy2 = cos(yaw/2);
        TYPE cp2 = cos(pitch/2);
        TYPE cr2 = cos(roll/2);
        TYPE sy2 = sin(yaw/2);
        TYPE sp2 = sin(pitch/2);
        TYPE sr2 = sin(roll/2);

        q.q0  = cr2*cp2*cy2 + sr2*sp2*sy2;
        q.q.x = sr2*cp2*cy2 - cr2*sp2*sy2;
        q.q.y = cr2*sp2*cy2 + sr2*cp2*sy2;
        q.q.z = cr2*cp2*sy2 - sr2*sp2*cy2;

        q.normalize();

        return q;

    }

    AngleAxis_<TYPE> toAngleAxis() const {
        TYPE y = this->yaw;
        TYPE p = this->pitch;
        TYPE r = this->roll;

        TYPE phi = acos(static_cast<TYPE>(0.5)*(-1 + cos(p)*cos(y) + cos(r)*cos(y) + sin(r)*sin(p)*sin(y) + cos(r)*cos(p) ));

        TYPE u_x = 1/(2*sin(phi))* (sin(r)*cos(p) -cos(r)*sin(p)*sin(y) +sin(r)*cos(y));
        TYPE u_y = 1/(2*sin(phi))* (sin(r)*sin(y) +cos(r)*sin(p)*cos(y) +sin(p));
        TYPE u_z = 1/(2*sin(phi))* (cos(p)*sin(y) -sin(r)*sin(p)*cos(y) + cos(r)*sin(y));

        Vector3D_<TYPE> u(u_x, u_y, u_z);
        AngleAxis_<TYPE> u_phi(phi, u);

        return u_phi;
    }

    void print(const print_format_t format = UNIT_DEGREE) const {
    	if(format == UNIT_RAD){
    		PRINTF("(%3.3f \t %3.3f \t %3.3f)\n", static_cast<double>(yaw), static_cast<double>(pitch), static_cast<double>(roll));
    	} else if(format == UNIT_DEGREE) {
    		PRINTF("(%3.3f deg\t %3.3f deg \t %3.3f deg)\n", static_cast<double>(yaw*180/M_PI), static_cast<double>(pitch*180/M_PI), static_cast<double>(roll*180/M_PI));
    	} else {
    		PRINTF("(%3.3f*PI \t %3.3f*PI \t %3.3f*PI)\n", static_cast<double>(yaw/M_PI), static_cast<double>(pitch/M_PI), static_cast<double>(roll/M_PI));
    	}
    }


	template <typename TYPE2>
	YPR_<TYPE> operator=(const YPR_<TYPE2> &other) {
		this->yaw   = other.yaw;
		this->pitch = other.pitch;
		this->roll  = other.roll;
		return *this;
	}

    YPR_<TYPE>& operator=(const YPR_<TYPE> &other) = default;

};

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* YPR_H_ */
