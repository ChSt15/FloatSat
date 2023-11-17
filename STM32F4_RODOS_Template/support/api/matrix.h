/*
 * matrix.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include "generic.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template <typename TYPE>
class Vector3D_;
template <typename TYPE>
class Vector6D_;
template <typename TYPE>
class YPR_;
template <typename TYPE>
class AngleAxis_;
template <typename TYPE>
class Quaternion_;

// 3x3-Matrix
template <typename TYPE = double>
class Matrix3D_ : public Matrix_<3,3,TYPE> {
public:

	Matrix3D_() : Matrix_<3,3,TYPE>() {} ///< identity matrix

	Matrix3D_(const Matrix_<3,3,TYPE> &other) : Matrix_<3,3,TYPE>(other){}

	Matrix3D_(const Vector3D_<TYPE>& column1, const Vector3D_<TYPE>& column2, const Vector3D_<TYPE>& column3) : Matrix_<3,3,TYPE>(){
		this->setColumn(0,(Vector_<3,TYPE>) column1);
		this->setColumn(1,(Vector_<3,TYPE>) column2);
		this->setColumn(2,(Vector_<3,TYPE>) column3);
	}

	Matrix3D_(TYPE* arr) : Matrix_<3,3,TYPE>(arr) {}             	///< from left to rigth from top to button

	Matrix3D_(const Matrix3D_<TYPE>& other) : Matrix_<3,3,TYPE>(other){}

	Matrix3D_(const Vector3D_<TYPE>& init) : Matrix_<3,3,TYPE>(init) {}   	///< diagonalmatrix of vector

	Matrix3D_(const YPR_<TYPE>& ypr) : Matrix_<3,3,TYPE>() {			  	///< rotation matrix from Euler
		TYPE cy = cos(ypr.yaw);
		TYPE cp = cos(ypr.pitch);
		TYPE cr = cos(ypr.roll);
		TYPE sy = sin(ypr.yaw);
	    TYPE sp = sin(ypr.pitch);
	    TYPE sr = sin(ypr.roll);

	    this->r[0][0]= cy*cp;
	    this->r[0][1]= cy*sp*sr - sy*cr;
	    this->r[0][2]= cy*sp*cr + sy*sr;

	    this->r[1][0]= sy*cp;
	    this->r[1][1]= sy*sp*sr + cy*cr;
	    this->r[1][2]= sy*sp*cr - cy*sr;

	    this->r[2][0]= -sp;
	    this->r[2][1]= cp*sr;
	    this->r[2][2]= cp*cr;
	}

	Matrix3D_(const AngleAxis_<TYPE>& other) : Matrix_<3,3,TYPE>() { 		///< corresponding rotation matrix
		Vector3D_<TYPE> u = other.u;
		TYPE phi = other.phi;
		TYPE cp  = cos(phi);
		TYPE sp  = sin(phi);

		// 1 Spalte
		this->r[0][0]= u.x *u.x *(1-cp) + cp;
		this->r[1][0]= u.x *u.y *(1-cp) + u.z*sp;
		this->r[2][0]= u.x *u.z *(1-cp) - u.y*sp;

		// 2 Spalte
		this->r[0][1]= u.x *u.y *(1-cp) - u.z*sp;
		this->r[1][1]= u.y *u.y *(1-cp) + cp;
		this->r[2][1]= u.z *u.y *(1-cp) + u.x*sp;

		// 3 Spalte
		this->r[0][2]= u.x *u.z *(1-cp) + u.y*sp;
		this->r[1][2]= u.y *u.z *(1-cp) - u.x*sp;
		this->r[2][2]= u.z *u.z *(1-cp) + cp;
	}

	Matrix3D_(const Quaternion_<TYPE>& other) { ///< corresponding rotation matrix
		//column1
		this->r[0][0] = 2*other.q0*other.q0-1 + 2*other.q.x * other.q.x;
		this->r[1][0] = 2*other.q.y*other.q.x  + 2*other.q0*other.q.z;
		this->r[2][0] = 2*other.q.z*other.q.x  - 2*other.q0*other.q.y;

		//column 2
		this->r[0][1] = 2*other.q.y*other.q.x - 2*other.q0*other.q.z;
		this->r[1][1] = 2*other.q0*other.q0-1 + 2*other.q.y*other.q.y;
		this->r[2][1] = 2*other.q.y*other.q.z + 2*other.q0*other.q.x;

		//column 3
		this->r[0][2] = 2*other.q.z*other.q.x + 2*other.q0*other.q.y;
		this->r[1][2] = 2*other.q.y*other.q.z - 2*other.q0*other.q.x;
		this->r[2][2] = 2*other.q0*other.q0-1 + 2*other.q.z * other.q.z;
	}

	static Matrix3D_ skew(Vector3D_<TYPE> vec){
		Matrix3D_<TYPE> ret;
		ret.r[0][0] = 0;
		ret.r[0][1] = -vec.z;
		ret.r[0][2] = vec.y;
		ret.r[1][0] = vec.z;
		ret.r[1][1] = 0;
		ret.r[1][2] = -vec.x;
		ret.r[2][0] = -vec.y;
		ret.r[2][1] = vec.x;
		ret.r[2][2] = 0;
		return ret;
	}

	TYPE getAngle() const {
	    TYPE angle;
	    angle = acos(static_cast<TYPE>(0.5)*(this->r[0][0]+this->r[1][1]+this->r[2][2]-1));
	    return angle;
	}

	Vector3D_<TYPE> getVec() const {
	    TYPE x,y,z,angle;
	    angle = this->getAngle();
	    x = 1/(2*sin(angle)) * (this->r[2][1]-this->r[1][2]);
	    y = 1/(2*sin(angle)) * (this->r[0][2]-this->r[2][0]);
	    z = 1/(2*sin(angle)) * (this->r[1][0]-this->r[0][1]);
	    Vector3D_<TYPE> u(x,y,z);

	    return u;
	}

	void rotationX(const TYPE &angle) {
	    Vector3D_<TYPE> c1(1, 0, 0);
	    this->setColumn(0,(Vector_<3,TYPE>) c1);
	    Vector3D_<TYPE> c2(0, cos(angle), sin(angle));
	    this->setColumn(1,(Vector_<3,TYPE>) c2);
	    Vector3D_<TYPE> c3(0, -sin(angle), cos(angle));
	    this->setColumn(2,(Vector_<3,TYPE>) c3);
	}

	void rotationY(const TYPE &angle) {
		Vector3D_<TYPE> c1(cos(angle), 0, -sin(angle));
		this->setColumn(0,(Vector_<3,TYPE>) c1);
	    Vector3D_<TYPE> c2(0, 1, 0);
	    this->setColumn(1,(Vector_<3,TYPE>) c2);
	    Vector3D_<TYPE> c3(sin(angle), 0, cos(angle));
	    this->setColumn(2,(Vector_<3,TYPE>) c3);
	}

	void rotationZ(const TYPE &angle) {
	    Vector3D_<TYPE> c1(cos(angle), sin(angle), 0);
	    this->setColumn(0,(Vector_<3, TYPE>) c1);
	    Vector3D_<TYPE> c2(-sin(angle), cos(angle), 0);
	    this->setColumn(1,(Vector_<3, TYPE>) c2);
	    Vector3D_<TYPE> c3(0, 0, 1);
	    this->setColumn(2,(Vector_<3, TYPE>) c3);
	}

	Quaternion_<TYPE> toQuaternion() const {

		TYPE q0,q1,q2,q3;
	    q0 = q1 = q2 = q3 = 0;
	    TYPE c1 = 1+ this->r[0][0] + this->r[1][1] + this->r[2][2];
	    TYPE c2 = 1+ this->r[0][0] - this->r[1][1] - this->r[2][2];
	    TYPE c3 = 1- this->r[0][0] + this->r[1][1] - this->r[2][2];
	    TYPE c4 = 1- this->r[0][0] - this->r[1][1] + this->r[2][2];
	    TYPE c = c1;
	    // Finde das Maximum
	    if(c2>c) c = c2;
	    if(c3>c) c = c3;
	    if(c4>c) c = c4;
	    //Fallunterscheidung
	    if(c==c1) {
	        c = static_cast<TYPE>(0.5) * sqrt(c);
	        q0 = c;
	        q1 = (this->r[2][1]-this->r[1][2])/(4*c);
	        q2 = (this->r[0][2]-this->r[2][0])/(4*c);
	        q3 = (this->r[1][0]-this->r[0][1])/(4*c);
	    }

	    if(c==c2) {
	        c = static_cast<TYPE>(0.5) * sqrt(c);
	        q0 = (this->r[2][1]-this->r[1][2])/(4*c);
	        q1 = c;
	        q2 = (this->r[1][0]+this->r[0][1])/(4*c);
	        q3 = (this->r[0][2]+this->r[2][0])/(4*c);
	    }

	    if(c==c3) {
	        c = static_cast<TYPE>(0.5) * sqrt(c);
	        q0 = (this->r[0][2]-this->r[2][0])/(4*c);
	        q1 = (this->r[1][0]+this->r[0][1])/(4*c);
	        q2 = c;
	        q3 = (this->r[2][1]+this->r[1][2])/(4*c);
	    }

	    if(c==c4) {
	        c = static_cast<TYPE>(0.5) * sqrt(c);
	        q0 = (this->r[1][0]-this->r[0][1])/(4*c);
	        q1 = (this->r[0][2]+this->r[2][0])/(4*c);
	        q2 = (this->r[2][1]+this->r[1][2])/(4*c);
	        q3 = c;
	    }
	    Quaternion_<TYPE> q(q0,q1,q2,q3);

	    return q;

	}

	YPR_<TYPE> toYPR() const { // DEPRECATED
	    YPR_<TYPE> y;
	    TYPE m21 = this->r[1][0];
	    TYPE m11 = this->r[0][0];
	    TYPE m31 = this->r[2][0];
	    TYPE m32 = this->r[2][1];
	    TYPE m33 = this->r[2][2];
	    y.pitch = atan2(-m31, sqrt(m11*m11 + m21*m21));
	    y.yaw   = atan2(m21, m11);
	    y.roll  = atan2(m32, m33);

	    return y;
	}

	AngleAxis_<TYPE> toAngleAxis() const {
		TYPE phi = acos(static_cast<TYPE>(0.5)*(this->r[0][0] + this->r[1][1] + this->r[2][2]-1));
		TYPE sp = sin(phi);

		TYPE x = 1/(2*sp)* (this->r[2][1] - this->r[1][2]);
		TYPE y = 1/(2*sp)* (this->r[0][2] - this->r[2][0]);
		TYPE z = 1/(2*sp)* (this->r[1][0] - this->r[0][1]);
	    Vector3D_<TYPE> u(x,y,z);
	    AngleAxis_<TYPE> u_phi(phi, u.normalize());
	    return u_phi;
	}

	Matrix3D_<TYPE>& operator=(const Matrix3D_<TYPE>& other) = default;

};

template <typename TYPE>
class Matrix6D_ : public Matrix_<6,6,TYPE>{
public:
	Matrix6D_() :
			Matrix_<6,6,TYPE>(){
	}

	Matrix6D_(const Matrix_<6,6,TYPE>& other) :
			Matrix_<6,6,TYPE>(other){
	}

	Matrix6D_(const TYPE* arr) :
			Matrix_<6,6>(arr){
	}

	Matrix6D_(const Vector6D_<TYPE>& diag) :
			Matrix_<6,6,TYPE>(diag){
	}

	Matrix6D_(const Matrix3D_<TYPE> &upperLeft, const Matrix3D_<TYPE> &upperRight, const Matrix3D_<TYPE> &lowerLeft, const Matrix3D_<TYPE> &lowerRight) :
			Matrix_<6,6,TYPE>() {
	    for ( int i = 0; i < 3; ++i ) {
	        for ( int j = 0; j < 3; ++j ) {
	            this->r[  i][  j] =  upperLeft.r[i][j];
	            this->r[  i][3+j] = upperRight.r[i][j];
	            this->r[3+i][  j] =  lowerLeft.r[i][j];
	            this->r[3+i][3+j] = lowerRight.r[i][j];
	        }
	    }
	}

	Matrix3D_<TYPE> upperLeft() const {
	    Matrix3D_<TYPE> P;
	    for(size_t i = 0 ; i<3; ++i) {
	        for(size_t j = 0; j<3; ++j) {
	            P.r[i][j] = this->r[  i][  j];
	        }
	    }
	    return P;
	}

	Matrix3D_<TYPE> upperRight() const {
	    Matrix3D_<TYPE> Q;
	    for(size_t i = 0 ; i<3; ++i) {
	        for(size_t j = 0; j<3; ++j) {
	            Q.r[i][j] = this->r[  i][3+j];
	        }
	    }
	    return Q;
	}

	Matrix3D_<TYPE> lowerLeft() const {
	    Matrix3D_<TYPE> R;
	    for(size_t i = 0 ; i<3; ++i) {
	        for(size_t j = 0; j<3; ++j) {
	            R.r[i][j] = this->r[3+i][  j];
	        }
	    }
	    return R;
	}

	Matrix3D_<TYPE> lowerRight() const {
	    Matrix3D_<TYPE> S;
	    for(size_t i = 0 ; i<3; ++i) {
	        for(size_t j = 0; j<3; ++j) {
	            S.r[i][j] = this->r[3+i][3+j];
	        }
	    }
	    return S;
	}

};


/*******************        functions       **********************/
template <typename TYPE>
Matrix3D_<TYPE> skewSymmetricMatrix( const Vector3D_<TYPE> &v ) {
	TYPE r[9];
    r[0] =  0;
    r[1] = -v.z;
    r[2] =  v.y;
    r[3] =  v.z;
    r[4] =  0;
    r[5] = -v.x;
    r[6] = -v.y;
    r[7] =  v.x;
    r[8] =  0;
    return Matrix3D_<TYPE>(r);
}

template <typename TYPE>
Matrix6D_<TYPE> dyadic(const Vector6D_<TYPE>& left, const Vector6D_<TYPE>& right) {
	Matrix6D_<TYPE> Mat;
	for(size_t i=0; i<6; ++i) {
		for(size_t j=0; j<6; ++j) {
			Mat.r[i][j] = left.v[i]*right.v[j];
		}
	}
	return Mat;
}
//bool ludcmp(Matrix6D_ &a, Vector6D_ &indx, float &d);
//void lubksb(Matrix6D_ &a, Vector6D_ &indx, Vector6D_ &b);

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* MATRIX_H_ */
