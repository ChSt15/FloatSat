/*
 * quaternion.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template <typename TYPE>
class Vector3D_;
template <typename TYPE>
class Matrix3D_;
template <typename TYPE>
class AngleAxis_;
template <typename TYPE>
class YPR_;

template <typename TYPE = double>
class Quaternion_ {

public:
    TYPE q0;
    Vector3D_<TYPE> q;

    Quaternion_() {
        Vector3D_<TYPE> q;
        this->q0 = 1;
        this->q  = q;
    }

    Quaternion_(const Quaternion_& other) {
        this->q0 = other.q0;
        this->q  = other.q;
    }

    Quaternion_(const TYPE &q0, const TYPE &q1, const TYPE &q2, const TYPE &q3) {
        Vector3D_<TYPE> q(q1, q2, q3);
        this->q0 = q0;
        this->q  = q;
    }

    Quaternion_(TYPE* arr) {
        Vector3D_<TYPE> vec(arr[1], arr[2], arr[3]);
        this->q0 = arr[0];
        this->q  = vec;
    }

    Quaternion_(const TYPE &q0,const Vector3D_<TYPE>& q) {
        this->q = q;
        this->q0 = q0;

    }

    Quaternion_(const AngleAxis_<TYPE>& other) {
        this->q0 = cos(other.u.getLen()*other.phi/2);
        this->q  = other.u.normalize().scale(other.u.getLen()*sin(other.phi/2));
    }

    explicit Quaternion_(const Matrix3D_<TYPE>& other) {  //Algorithmus 1

        TYPE q0,q1,q2,q3;
        q0 = q1 = q2 = q3 = 0;
        TYPE c1 = 1+ other.r[0][0] + other.r[1][1] + other.r[2][2];
        TYPE c2 = 1+ other.r[0][0] - other.r[1][1] - other.r[2][2];
        TYPE c3 = 1- other.r[0][0] + other.r[1][1] - other.r[2][2];
        TYPE c4 = 1- other.r[0][0] - other.r[1][1] + other.r[2][2];
        TYPE c = c1;
        // Finde das Maximum
        if(c2>c) c = c2;
        if(c3>c) c = c3;
        if(c4>c) c = c4;
        //Fallunterscheidung
        if(c==c1) {
            c = static_cast<TYPE>(0.5) * sqrt(c);
            q0 = c;
            q1 = (other.r[2][1]-other.r[1][2])/(4*c);
            q2 = (other.r[0][2]-other.r[2][0])/(4*c);
            q3 = (other.r[1][0]-other.r[0][1])/(4*c);
        }

        if(c==c2) {
            c = static_cast<TYPE>(0.5) * sqrt(c);
            q0 = (other.r[2][1]-other.r[1][2])/(4*c);
            q1 = c;
            q2 = (other.r[1][0]+other.r[0][1])/(4*c);
            q3 = (other.r[0][2]+other.r[2][0])/(4*c);
        }

        if(c==c3) {
            c = static_cast<TYPE>(0.5) * sqrt(c);
            q0 = (other.r[0][2]-other.r[2][0])/(4*c);
            q1 = (other.r[1][0]+other.r[0][1])/(4*c);
            q2 = c;
            q3 = (other.r[2][1]+other.r[1][2])/(4*c);
        }

        if(c==c4) {
            c = static_cast<TYPE>(0.5) * sqrt(c);
            q0 = (other.r[1][0]-other.r[0][1])/(4*c);
            q1 = (other.r[0][2]+other.r[2][0])/(4*c);
            q2 = (other.r[2][1]+other.r[1][2])/(4*c);
            q3 = c;
        }
        Vector3D_<TYPE> q(q1,q2,q3);

        this->q0 = q0;
        this->q = q;
    }

    Quaternion_(const YPR_<TYPE>& other) { // DEPRECATED
    	TYPE a = other.roll/2;
    	TYPE b = other.pitch/2;
    	TYPE c = other.yaw/2;

    	TYPE cdx = cos(a);
    	TYPE sdx = sin(a);
    	TYPE cdy = cos(b);
        TYPE sdy = sin(b);
        TYPE cdz = cos(c);
        TYPE sdz = sin(c);

        //% Transforms RPY EulerAngles into Quaternion_
        //% dx angle of rotation about x-axis (using RAD) | cdx = cos(dx) etc.
        //% dy angle of rotation about y-axis
        //% dz angle of rotation about z-axis

        TYPE q0 = cdz*cdy*cdx + sdz*sdy*sdx;
        TYPE q1 = cdz*cdy*sdx - sdz*sdy*cdx;
        TYPE q2 = cdz*sdy*cdx + sdz*cdy*sdx;
        TYPE q3 = sdz*cdy*cdx - cdz*sdy*sdx;

        TYPE len  = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

        q0 = q0 / len;
        q1 = q1 / len;
        q2 = q2 / len;
        q3 = q3 / len;

        Vector3D_<TYPE> q(q1, q2, q3);

        this->q0 = q0;
        this->q  = q;

    }

    bool resetIfNAN() {
        bool error = !isfinite(q0) || !isfinite(q.x) || !isfinite(q.y) || !isfinite(q.z);
        if(error) { q0 = 1;   q.x = q.y = q.z = 0; }
        return error;
    }

    Vector3D_<TYPE> getVec() const  {
        Vector3D_<TYPE> u= this->q.normalize();
        return u;
    }

    TYPE getAngle() const {
        return 2*acos(q0);
    }

    Quaternion_<TYPE> qAdd(const Quaternion_& other) const {
        Quaternion_<TYPE> sum;
        sum.q0 = this->q0 + other.q0;
        sum.q  = this->q.vecAdd(other.q);
        return sum;
    }

    Quaternion_<TYPE> qSub(const Quaternion_& other) const {
        Quaternion_<TYPE> diff;
        diff.q0 = this->q0 - other.q0;
        diff.q  = this->q.vecSub(other.q);
        return diff;
    }

    Quaternion_<TYPE> scale(const TYPE &factor) const {
        Quaternion_<TYPE> scale;
        scale.q0 = this->q0 *factor;
        scale.q  = this->q.scale(factor);
        return scale;
    }

    Quaternion_<TYPE> qMult(const Quaternion_<TYPE>& other) const {
        Quaternion_<TYPE> mult, left, right;
        left = (*this);
        right = other;

        if(left.q.equals(right.q)) {
            mult.q0 = left.q0 + right.q0;
            mult.q = left.q;
        }

        // mult.q0 = left.q0*right.q0 - left.q.dot(right.q);
        // mult.q  = left.q.scale(right.q0).vecAdd(right.q.scale(left.q0))
        //         .vecAdd(left.q.cross(right.q));

        mult.q0 = left.q0 * right.q0 - left.q.dot(right.q);

        mult.q  = left.q  * right.q0 + // Warning: vectro ops
                  right.q * left.q0  + // warning: operator +, * etc
                  left.q.cross(right.q);
        return mult;
    }

    Quaternion_<TYPE> invert() const {
        TYPE quads = q0*q0 + q.x*q.x + q.y*q.y + q.z*q.z;
        return this->conjugate().scale(static_cast<TYPE>(1)/quads);
    }

    Quaternion_<TYPE> conjugate() const {
        Quaternion_<TYPE> conjugate;
        conjugate.q0 = this->q0;
        conjugate.q = this->q.scale(-1);
        return conjugate;
    }

    Quaternion_<TYPE> qDivide(const Quaternion_<TYPE>& other) const {
        Quaternion_<TYPE> division = other;
        Quaternion_<TYPE> invert = division.conjugate();
        division = this->qMult(invert);
        return division;
    }

    TYPE getLen() const {
    	TYPE quads = q0*q0 + q.x*q.x + q.y*q.y + q.z*q.z;
        return sqrt(quads);
    }

    Quaternion_<TYPE> normalize() const {
        Quaternion_<TYPE> unit;
        unit = this->scale(1/this->getLen());
        return unit;
    }

    bool isNormalized() const {
    	TYPE len=this->getLen();
        return isAlmost0(len-1);
    }

    bool equals(const Quaternion_<TYPE>& other) const {
        return(isAlmost0(q0 - other.q0) && q.equals(other.q));
    }

    AngleAxis_<TYPE> toAngleAxis() const {
        AngleAxis_<TYPE> u_phi(this->getAngle(), this->getVec());
        return u_phi;
    }

    Matrix3D_<TYPE> toMatrix3D() const { // return matrix representation of quaternion
    	TYPE q0sq2 = 2*this->q0  * this->q0;
    	TYPE qxsq2 = 2*this->q.x * this->q.x;
    	TYPE qysq2 = 2*this->q.y * this->q.y;
    	TYPE qzsq2 = 2*this->q.z * this->q.z;

    	TYPE q0qx2 = 2*this->q0  * this->q.x;
    	TYPE q0qy2 = 2*this->q0  * this->q.y;
    	TYPE q0qz2 = 2*this->q0  * this->q.z;

    	TYPE qyqx2 = 2*this->q.y * this->q.x;
    	TYPE qyqz2 = 2*this->q.y * this->q.z;

    	TYPE qzqx2 = 2*this->q.z * this->q.x;

        Matrix3D_<TYPE> R;
        //column1
        R.r[0][0] = q0sq2-1 + qxsq2;
        R.r[1][0] = qyqx2   + q0qz2;
        R.r[2][0] = qzqx2   - q0qy2;

        //column 2
        R.r[0][1] = qyqx2   - q0qz2;
        R.r[1][1] = q0sq2-1 + qysq2;
        R.r[2][1] = qyqz2   + q0qx2;

        //column 3
        R.r[0][2] = qzqx2   + q0qy2;
        R.r[1][2] = qyqz2   - q0qx2;
        R.r[2][2] = q0sq2-1 + qzsq2;
        return R;
    }

    YPR_<TYPE> toYPR() const { // DEPRECATED
        YPR_<TYPE> ypr;

        ypr.roll = atan2(2*(q0*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y));

        // Limit the sin of pitch between [-1,1]
        TYPE sinPitch = 2*(q0*q.y-q.z*q.x);
        if(sinPitch > 1) sinPitch = 1;
        if(sinPitch < -1) sinPitch = -1;
        ypr.pitch = asin(sinPitch);

        ypr.yaw = atan2(2*(q0*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));

        return ypr;
    }

    static Quaternion_<TYPE> qint(TYPE dt, Vector_<3,TYPE> omega){
    	Quaternion_<TYPE> ret;
    	float len =  omega.getLen();
    	ret.q0 = cos(len*dt/2);
    	float sin_omega = sin(len*dt/2);
    	ret.q.x = omega.r[0][0]/len*sin_omega;
    	ret.q.y = omega.r[1][0]/len*sin_omega;
    	ret.q.z = omega.r[2][0]/len*sin_omega;
    	return ret;
    }

    void print() const {
        PRINTF("[ %3.3lf \t (%3.3lf \t %3.3lf \t %3.3lf)]\n",static_cast<TYPE>(q0),static_cast<TYPE>(q.x),static_cast<TYPE>(q.y),static_cast<TYPE>(q.z));
    }

    inline friend Quaternion_<TYPE> operator+(const Quaternion_<TYPE> &left, const Quaternion_<TYPE> &right)  { return left.qAdd(right); }
    inline friend Quaternion_<TYPE> operator-(const Quaternion_<TYPE> &left, const Quaternion_<TYPE> &right)  { return left.qSub(right); }
    inline friend Quaternion_<TYPE> operator*(const Quaternion_<TYPE> &left, const Quaternion_<TYPE> &right)  { return left.qMult(right); }
    inline friend Quaternion_<TYPE> operator*(const TYPE     		 &left, const Quaternion_<TYPE> &right)  { return right.scale(left); }
    inline friend Quaternion_<TYPE> operator*(const Quaternion_<TYPE> &left, const TYPE     &right)  		 { return left.scale(right); }
    inline friend Quaternion_<TYPE> operator*(const Matrix_<4,4,TYPE>  &mat, const Quaternion_<TYPE> &q)   {
    	Quaternion_<TYPE> res;
    	res.q0  = mat.r[0][0] * q.q0 + mat.r[0][1] * q.q.x + mat.r[0][2] * q.q.y + mat.r[0][3] * q.q.z;
    	res.q.x = mat.r[1][0] * q.q0 + mat.r[1][1] * q.q.x + mat.r[1][2] * q.q.y + mat.r[1][3] * q.q.z;
    	res.q.y = mat.r[2][0] * q.q0 + mat.r[2][1] * q.q.x + mat.r[2][2] * q.q.y + mat.r[2][3] * q.q.z;
    	res.q.z = mat.r[3][0] * q.q0 + mat.r[3][1] * q.q.x + mat.r[3][2] * q.q.y + mat.r[3][3] * q.q.z;
    	return res;
    }
    inline friend Vector3D_<TYPE>   operator*(const Quaternion_<TYPE> &left, const Vector3D_<TYPE>  &right)   { return right.qRotate(left); }
    inline friend Quaternion_<TYPE> operator/(const Quaternion_<TYPE> &left, const TYPE     &right)  		{ return left.scale(1/right); }
    inline friend Quaternion_<TYPE> operator-(const Quaternion_<TYPE> &right)                   				{ return right.conjugate(); }

    inline friend Quaternion_<TYPE> qX(const TYPE &phi) { return Quaternion_<TYPE>(cos(phi/2), sin(phi/2), 0, 0); }
    inline friend Quaternion_<TYPE> qY(const TYPE &phi) { return Quaternion_<TYPE>(cos(phi/2), 0, sin(phi/2), 0); }
    inline friend Quaternion_<TYPE> qZ(const TYPE &phi) { return Quaternion_<TYPE>(cos(phi/2), 0, 0, sin(phi/2)); }
//    inline friend Quaternion_<TYPE> q1()           		 { return Quaternion_<TYPE>(1,          0.0, 0.0, 0.0); }

	template <typename TYPE2>
	Quaternion_<TYPE> operator=(const Quaternion_<TYPE2> &other) {
		this->q.x = other.q.x;
		this->q.y = other.q.y;
		this->q.z = other.q.z;
		this->q0 = other.q0;
		return *this;
	}

    Quaternion_<TYPE>& operator=(const Quaternion_<TYPE> &other) = default;
};

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* QUATERNION_H_ */
