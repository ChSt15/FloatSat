/*
 * angleaxis.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef ANGLEAXIS_H_
#define ANGLEAXIS_H_

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template<typename TYPE>
class Vector3D_;
template<typename TYPE>
class Matrix3D_;
template<typename TYPE>
class YPR_;

///stellt eine Rotation durch einen Drehwinkel und eine Rotationsachse dar
template<typename TYPE = double>
class AngleAxis_ {
public:
    Vector3D_<TYPE> u; ///< Stellt den Einheitsvektor u der Rotation dar.
    TYPE phi; ///< Stellt den Rotationswinkel phi in Radians dar.

    AngleAxis_() {
        Vector3D_<TYPE> u(1,0,0);
        this->u   = u;
        this->phi = 0;
    }

    AngleAxis_(const TYPE &phi, const TYPE &x, const TYPE &y, const TYPE &z) {
        this->phi = phi;
        this->u.x = x;
        this->u.y = y;
        this->u.z = z;
        this->u = this->u.normalize();
    }

    AngleAxis_(const AngleAxis_& other) {
        this->u   = other.u;
        this->phi = other.phi;
    }

    AngleAxis_(const TYPE &phi, const Vector3D_<TYPE>& u) {
        this->u   = u;
        this->u   = this->u.normalize();
        this->phi = phi;
    }

    AngleAxis_(const Quaternion_<TYPE>& q) {
        this->u   = q.getVec().normalize();
        this->phi = q.getAngle();
    }

    AngleAxis_(const Matrix3D_<TYPE>& M) {
        this->phi= acos(static_cast<TYPE>(0.5)*(M.r[0][0] + M.r[1][1] + M.r[2][2]-1));

        TYPE x = 1/(2*sin(phi))* (M.r[2][1] - M.r[1][2]);
        TYPE y = 1/(2*sin(phi))* (M.r[0][2] - M.r[2][0]);
        TYPE z = 1/(2*sin(phi))* (M.r[1][0] - M.r[0][1]);
        Vector3D_<TYPE> u(x,y,z);
        this->u = u.normalize();
    }

    AngleAxis_(const YPR_<TYPE>& ypr) { // DEPRECATED
    	TYPE y = ypr.yaw;
    	TYPE p = ypr.pitch;
    	TYPE r = ypr.roll;

		// Fast version
		TYPE c1 = cos(y / 2);
		TYPE c2 = cos(p / 2);
		TYPE c3 = cos(r / 2);
		TYPE s1 = sin(y / 2);
		TYPE s2 = sin(p / 2);
		TYPE s3 = sin(r / 2);
		TYPE phi = 2 * acos(c1*c2*c3 + s1*s2*s3);
		TYPE u_x =c1*c2*s3 - s1*s2*c3;
		TYPE u_y =c1*s2*c3 + s1*c2*s3;
		TYPE u_z =s1*c2*c3 - c1*s2*s3;
        Vector3D_<TYPE> u(u_x,u_y,u_z);

        this->phi = phi;
        this->u = u.normalize();

    }

	bool equals(const AngleAxis_<TYPE>& other) const {
	    if (isAlmost0(FMod2p(this->phi - other.phi + M_PI) - M_PI) && this->u.equals(other.u)) return true;

	    if (isAlmost0(FMod2p(this->phi + other.phi + M_PI) - M_PI) && this->u.equals(-1 * other.u)) return true;

	    return false;
	}

    Quaternion_<TYPE> toQuaternion() const {
    	TYPE q0  = cos(this->phi/2);
        Vector3D_<TYPE> q = this->u.scale(sin(this->phi/2));
        Quaternion_<TYPE> quat(q0,q);
        return quat;
    }

    Matrix3D_<TYPE> toMatrix3D() const { // allgemeine Rotationsmatrix
        Matrix3D_<TYPE> R;
        Vector3D_<TYPE> u = this->u;
        TYPE phi = this->phi;

        // 1 Spalte
        R.r[0][0]= u.x *u.x *(1-cos(phi)) +cos(phi);
        R.r[1][0]= u.x *u.y *(1-cos(phi)) +u.z*sin(phi);
        R.r[2][0]= u.x *u.z *(1-cos(phi)) -u.y*sin(phi);

        // 2 Spalte
        R.r[0][1]= u.x *u.y *(1-cos(phi)) -u.z*sin(phi);
        R.r[1][1]= u.y *u.y *(1-cos(phi)) +cos(phi);
        R.r[2][1]= u.z *u.y *(1-cos(phi)) +u.x*sin(phi);

        // 3 Spalte
        R.r[0][2]= u.x *u.z *(1-cos(phi)) +u.y*sin(phi);
        R.r[1][2]= u.y *u.z *(1-cos(phi)) -u.x*sin(phi);
        R.r[2][2]= u.z *u.z *(1-cos(phi)) +cos(phi);

        return R;
    }

    YPR_<TYPE> toYPR() const {
        YPR_<TYPE> ypr;
        Vector3D_<TYPE> u = this->u;
        TYPE phi = this->phi;
        TYPE m21 = u.x *u.y *(1-cos(phi)) + u.z*sin(phi);
        TYPE m11 = u.x *u.x *(1-cos(phi)) + cos(phi);
        TYPE m31 = u.x *u.z *(1-cos(phi)) - u.y*sin(phi);
        TYPE m32 = u.y *u.z *(1-cos(phi)) + u.x*sin(phi);
        TYPE m33 = u.z *u.z *(1-cos(phi)) + cos(phi);

        ypr.pitch = atan2(-m31,sqrt(m11*m11+m21*m21));
        ypr.yaw = atan2(m21 , m11);
        ypr.roll = atan2(m32 , m33);

        return ypr;
    }

	template <typename TYPE2>
	AngleAxis_<TYPE> operator=(const AngleAxis_<TYPE2> &other) {
		this->u.x = other.u.x;
		this->u.y = other.u.y;
		this->u.z = other.u.z;
		this->phi = other.phi;
		return *this;
	}

    AngleAxis_<TYPE>& operator=(const AngleAxis_<TYPE> &other) = default;

    void print() const {
        PRINTF("[%3.3lf*PI \t %3.3lf \t %3.3f \t %3.3lf]\n",static_cast<double>(phi/M_PI), static_cast<double>(u.x), static_cast<double>(u.y), static_cast<double>(u.z));
    }

    static AngleAxis_<TYPE> rotor(const Vector3D_<TYPE>& fromVector, const Vector3D_<TYPE>& toVector) {
    	Vector3D_<TYPE> from = fromVector.normalize();
    	Vector3D_<TYPE> to   = toVector.normalize();
        TYPE cosAngle = dotProduct  (from, to);
        Vector3D_<TYPE> axis = crossProduct(from, to);
        if(isAlmost0(axis.getLen())) { cosAngle = axis.x = axis.y = axis.z = 1; }
        axis = axis.normalize();
        return AngleAxis_<TYPE>(acos(cosAngle), axis);
    }

    bool resetIfNAN() {
        bool error = !isfinite(phi) || !isfinite(u.x) || !isfinite(u.y) || !isfinite(u.z);
        if(error) {
        	u.x = u.y = u.z = 1;
        	phi = 0;
        }
        return error;
    }

    bool isNoRotation() { return isAlmost0(1 - phi)  || isAlmost0(u.getLen()) || (fabs(phi) > 1); }

};




#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* ANGLEAXIS_H_ */
