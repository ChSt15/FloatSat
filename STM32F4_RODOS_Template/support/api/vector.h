/*
 * vector.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef VECTOR_H_
#define VECTOR_H_

#include "generic.h"
#include "math_support.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template <typename TYPE>
class YPR_;
template <typename TYPE>
class Polar_;
template <typename TYPE>
class HCoord_;
template <typename TYPE>
class AngleAxis_;
template <typename TYPE>
class Quaternion_;
template <typename TYPE>
class Matrix3D_;
template <typename TYPE>
class Matrix6D_;

template <size_t ROW, size_t COL, typename TYPE>
class Matrix_;

template <size_t ROW, typename TYPE>
class Vector_;

template <typename TYPE = double>
class Vector3D_ {
public:
	TYPE x;
	TYPE y;
	TYPE z;

	Vector3D_() {
	    x = 0;
	    y = 0;
	    z = 0;
	}

	Vector3D_(const Vector3D_ &) = default;

	Vector3D_(const Vector_<3,TYPE> &other) {
		x = other.r[0][0];
		y = other.r[1][0];
		z = other.r[2][0];
	}

	Vector3D_(const Matrix_<3,1,TYPE> &other) {
		x = other.r[0][0];
		y = other.r[1][0];
		z = other.r[2][0];
	}

	Vector3D_ (const TYPE &x, const TYPE &y, const TYPE &z) {
	    this->x = x;
	    this->y = y;
	    this->z = z;
	}

	Vector3D_ (TYPE* arr) {
	    x = arr[0];
	    y = arr[1];
	    z = arr[2];
	}

	bool resetIfNAN() {
	    bool error = !isfinite(x) || !isfinite(y) || !isfinite(z);
	    if(error) x = y = z = 0;
	    return error;
	}

	Vector3D_<TYPE> vecAdd(const Vector3D_<TYPE>& other) const {
		Vector3D_<TYPE> sum;
	    sum.x= x + other.x;
	    sum.y= y + other.y;
	    sum.z= z + other.z;
	    return sum;
	}

	Vector3D_<TYPE> vecSub(const Vector3D_<TYPE>& other) const {
		Vector3D_<TYPE> diff;
	    diff.x= x - other.x;
	    diff.y= y - other.y;
	    diff.z= z - other.z;
	    return diff;
	}

	Vector3D_<TYPE>  scale(const TYPE &factor) const {
		Vector3D_<TYPE> scale;
	    scale.x = x * factor;
	    scale.y = y * factor;
	    scale.z = z * factor;
	    return scale;
	}

	Vector3D_<TYPE> cross(const Vector3D_<TYPE>& other) const {
		Vector3D_<TYPE> cross;
	    cross.x = this->y*other.z - this->z*other.y;
	    cross.y = this->z*other.x - this->x*other.z;
	    cross.z = this->x*other.y - this->y*other.x;
	    return cross;
	}

	TYPE dot(const Vector3D_<TYPE>& other) const {
	    TYPE product;
	    product = x*other.x + y*other.y + z*other.z;
	    return product;
	}

	TYPE getLen() const {
	    TYPE len;
	    len = sqrt(x*x + y*y + z*z);
	    return len;
	}

	TYPE distance(const Vector3D_<TYPE>& other) const {
		Vector3D_<TYPE> diff = *this - other;
	    return diff.getLen();
	}

	Vector3D_<TYPE> normalize() const {
		Vector3D_<TYPE> norm;
	    TYPE len = this->getLen();
//	    if(isAlmost0(len)) return Vector3D_<TYPE>(0,0,0); // avoid division by 0
	    norm.x = x/len;
	    norm.y = y/len;
	    norm.z = z/len;
	    return norm;
	}

	bool isNormalized() const {
	    TYPE len = this->getLen();
	    return isAlmost0(len-1);
	}

	bool equals(const Vector3D_<TYPE>& other) const {
	    return isAlmost0(x - other.x) && isAlmost0(y - other.y) &&  isAlmost0(z - other.z);
	}

	TYPE getAngle( const Vector3D_<TYPE>& other) const {
	    TYPE angle,product,len;
	    len = this->getLen() * other.getLen() ;
	    product = this->dot(other);
	    angle = acos(product/len);
	    return angle;   // radians
	}

	bool isOrthogonal(const Vector3D_<TYPE>& other) const {
	    TYPE cosAngle = this->dot(other);
	    return isAlmost0(cosAngle);
	}

	Polar_<TYPE> carToPolar() const {  // polar(r,phi,theta)
	    TYPE r     = this->getLen();
	    TYPE phi   = atan2(y, x);
	    TYPE theta = acos(z/r);
	    Polar_<TYPE> polar(r,phi,theta);
	    return polar;
	}

	Vector3D_<TYPE> matVecMult(const Matrix3D_<TYPE>& r) const { // M*v
		Vector3D_<TYPE> temp = r.getRow(0).transpose();
	    TYPE x = temp.dot(*this);
	    temp = r.getRow(1).transpose();
	    TYPE y = temp.dot(*this);
	    temp = r.getRow(2).transpose();
	    TYPE z = temp.dot(*this);
	    Vector3D_<TYPE> result(x, y, z);
	    return result;
	}

	Vector3D_<TYPE> qRotate(const Quaternion_<TYPE>& rotQuat) const { // w = qvq^* Vektor rotiert um Frame
	    Quaternion_<TYPE> v(0,(*this)); // ohne normalize()
	    Quaternion_<TYPE> w  = rotQuat * v * -rotQuat;

	    return w.q;
	}

	Vector3D_<TYPE> mRotate(const Matrix3D_<TYPE>& r) const { // w = M*v
		Vector3D_<TYPE> w(this->matVecMult(r));
	    return w;
	}

	Vector3D_<TYPE> yprRotate(const YPR_<TYPE>& ypr) const {
		Matrix3D_<TYPE> rotor = ypr.toMatrix3D();
		Vector3D_<TYPE> w(this->matVecMult(rotor));
	    return w;
	}

	Vector3D_<TYPE> aRotate(const AngleAxis_<TYPE>& u_phi) const { // Rodriguez-Formula
		Vector3D_<TYPE> temp = this->scale(cos(u_phi.phi)).vecAdd(u_phi.u.cross(this->scale(sin(u_phi.phi))));
		Vector3D_<TYPE> w = temp.vecAdd(u_phi.u.scale(u_phi.u.dot(*this) * (1-cos(u_phi.phi))));
	    return w;
	}

	void print() const {
		PRINTF("\nVector3D = \n[");
		PRINTF(" %4.5lf\n  %4.5lf\n  %4.5lf ]\n", static_cast<double>(x),static_cast<double>(y),static_cast<double>(z));
	}

	HCoord_<TYPE> toHCoord() const{
		return HCoord_<TYPE>(x,y,z,1);
	}

	Vector3D_<TYPE> elementWiseProduct(const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) {
		Vector3D_<TYPE> res;
	    res.x = left.x * right.x;
	    res.y = left.y * right.y;
	    res.z = left.z * right.z;
	    return res;
	}

	Vector3D_<TYPE> elementWiseDivision(const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) {
		Vector3D_<TYPE> res;
	    if(!isAlmost0(right.x) && !isAlmost0(right.y) && !isAlmost0(right.z)){
	   	    res.x = left.x / right.x;
	   	    res.y = left.y / right.y;
	   	    res.z = left.z / right.z;
	    }
	    return res;
	}

	Vector3D_<TYPE> rotateX(const Vector3D_<TYPE>& s,const TYPE &angle) {
		Vector3D_<TYPE> end;
	    Matrix3D_<TYPE> Rx;
	    Rx.rotationX(angle);
	    end = s.matVecMult(Rx);
	    return end;
	}

	Vector3D_<TYPE> rotateY(const Vector3D_<TYPE>& s,const TYPE &angle) {
		Vector3D_<TYPE> end;
	    Matrix3D_<TYPE> Ry;
	    Ry.rotationY(angle);
	    end = s.matVecMult(Ry);
	    return end;
	}

	Vector3D_<TYPE> rotateZ(const Vector3D_<TYPE>& s,const TYPE &angle) {
		Vector3D_<TYPE> end;
	    Matrix3D_<TYPE> Rz;
	    Rz.rotationZ(angle);
	    end = s.matVecMult(Rz);
	    return end;
	}

	inline friend Vector3D_<TYPE> operator+   (const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) { return left.vecAdd(right); }
	inline friend Vector3D_<TYPE> operator-   (const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) { return left.vecSub(right); }
	inline friend Vector3D_<TYPE> operator*   (const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) { Vector3D_<TYPE> ret; ret.x = left.x * right.x;ret.y = left.y * right.y;ret.z = left.z * right.z; return ret;}
	inline friend Vector3D_<TYPE> operator*   (const TYPE   &left, 	   		const Vector3D_<TYPE> &right) { return right.scale(left); }
	inline friend Vector3D_<TYPE> operator*   (const Vector3D_<TYPE> &left, const TYPE   &right) 				{ return left.scale(right); }
	inline friend Vector3D_<TYPE> operator/   (const Vector3D_<TYPE> &left, const TYPE   &right) 				{ return left.scale(1/right); }
	inline friend bool operator==   (const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) 				{ return (left.x == right.x) && (left.y == right.y) && (left.z == right.z); }
	inline friend bool operator!=   (const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) 				{ return !((left.x == right.x) && (left.y == right.y) && (left.z == right.z)); }

	template <typename TYPE2>
	Vector3D_<TYPE> operator=(const Vector3D_<TYPE2> &other) {
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		return *this;
	}

	Vector3D_<TYPE>& operator=(const Vector3D_<TYPE> &other) = default;
};

template <typename TYPE>
Vector3D_<TYPE> elementWiseProduct (const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) {
	Vector3D_<TYPE>  ret;
	ret.x = left.x * right.x;
	ret.y = left.y * right.y;
	ret.z = left.z * right.z;
	return ret;
}
template <typename TYPE>
Vector3D_<TYPE> elementWiseDivision(const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) {
	Vector3D_<TYPE>  ret;
	ret.x = left.x / right.x;
	ret.y = left.y / right.y;
	ret.z = left.z / right.z;
	return ret;
}
template <typename TYPE>
Vector3D_<TYPE> rotateX( const Vector3D_<TYPE>& s, const TYPE &angle); ///< rotates vector s angle (radians) arround x
template <typename TYPE>
Vector3D_<TYPE> rotateY( const Vector3D_<TYPE>& s, const TYPE &angle); ///< rotates vector s angle (radians) arround y
template <typename TYPE>
Vector3D_<TYPE> rotateZ( const Vector3D_<TYPE>& s, const TYPE &angle); ///< rotates vector s angle (radians) arround z

/*******************        operators       **********************/
template <typename TYPE>
inline TYPE   dotProduct  		  	     (const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) { return left.dot(right);   }
template <typename TYPE>
inline Vector3D_<TYPE> crossProduct(const Vector3D_<TYPE> &left, const Vector3D_<TYPE> &right) { return left.cross(right); }

template <typename TYPE = double>
class Vector6D_ {
public:
    TYPE v[6];

    Vector6D_() {
        for (size_t i=0; i<6; ++i) {
            v[i]=0;
        }
    }

    Vector6D_(const Vector_<6,TYPE>& other) {
    	for (size_t i=0; i<6; ++i) {
    		v[i]=other.r[i][0];
    	}
    }

    Vector6D_(const Matrix_<6,1,TYPE>& other) {
        for (size_t i=0; i<6; ++i) {
            v[i]=other.r[i][0];
        }
    }

    Vector6D_(const TYPE* arr) {
        for (size_t i=0; i<6; ++i) {
            v[i]=arr[i];
        }
    }

    Vector6D_(TYPE x_, TYPE y_, TYPE z_,
                       TYPE u_, TYPE v_, TYPE w_) {
      v[0]=x_; v[1]=y_; v[2]=z_;
      v[3]=u_; v[4]=v_; v[5]=w_;
    }

    Vector6D_( const Vector3D_<TYPE> &upper, const Vector3D_<TYPE> &lower ) {
        v[0] = upper.x;
        v[1] = upper.y;
        v[2] = upper.z;
        v[3] = lower.x;
        v[4] = lower.y;
        v[5] = lower.z;
    }

    TYPE getLen() const {
        return sqrt(dotProduct(*this,*this));
    }

    Vector6D_<TYPE> vecAdd(const Vector6D_<TYPE>& other) const {

        Vector6D_<TYPE> Aux;
        for (size_t i=0; i<6; ++i) {
            Aux.v[i]=v[i]+other.v[i];
        }
        return Aux;
    }

    Vector6D_<TYPE> vecSub(const Vector6D_<TYPE>& other) const {

        Vector6D_<TYPE> Aux;
        for (size_t i=0; i<6; ++i) {
            Aux.v[i]=v[i]-other.v[i];
        }
        return Aux;
    }

    Vector6D_<TYPE> matVecMult(const Matrix6D_<TYPE>& M) const {

        Vector6D_<TYPE> Aux;
        TYPE Sum;
        for (size_t i=0; i<6; ++i) {
            Sum = 0;
            for (size_t j=0; j<6; ++j) {
                Sum += M.r[i][j] * v[j];
            }
            Aux.v[i] = Sum;
        }
        return Aux;
    }

    Vector6D_<TYPE> scale(const TYPE &factor) const {

        Vector6D_<TYPE> Aux;
        for (size_t i=0; i<6; ++i) {
            Aux.v[i]=factor*v[i];
        }
        return Aux;
    }

    Matrix_<1,6,TYPE> transpose() const {
    	Matrix_<1,6,TYPE> ret;
        for (size_t i=0; i<6; ++i) {
        	ret.r[0][i]=v[i];
        }
        return ret;
    }

    inline friend Vector6D_<TYPE> operator+(const Vector6D_<TYPE>& left, const Vector6D_<TYPE>& right) { return left.vecAdd(right); }
    inline friend Vector6D_<TYPE> operator-(const Vector6D_<TYPE>& left, const Vector6D_<TYPE>& right) { return left.vecSub(right); }
    inline friend Vector6D_<TYPE> operator*(TYPE value, 				 const Vector6D_<TYPE>& right) { return right.scale(value); }
    inline friend Vector6D_<TYPE> operator*(const Vector6D_<TYPE>& left, const TYPE &value) 		   { return left.scale(value); }
    inline friend Vector6D_<TYPE> operator/(const Vector6D_<TYPE> &left, const TYPE &right) 		   { return left.scale(1/right); }
	inline friend bool operator==   (const Vector6D_<TYPE> &left, const Vector6D_<TYPE> &right){
		for (size_t i = 0; i < 6; ++i) {
			if(!isAlmost0(left.v[i]-right.v[i]))
				return false;
		}
		return true;
	}
	inline friend bool operator!=   (const Vector6D_<TYPE> &left, const Vector6D_<TYPE> &right) {
		for (size_t i = 0; i < 6; ++i) {
			if(!isAlmost0(left.v[i]-right.v[i]))
				return true;
		}
		return false;
	}
	template <typename TYPE2>
	Vector6D_<TYPE> operator=(const Vector6D_<TYPE2> &other) {
		this->v[0] = other.v[0];
		this->v[1] = other.v[1];
		this->v[2] = other.v[2];
		this->v[3] = other.v[3];
		this->v[4] = other.v[4];
		this->v[5] = other.v[5];
		return *this;
	}

};


/*******************        functions       **********************/
template <typename TYPE>
TYPE dotProduct(const Vector6D_<TYPE>& left, const Vector6D_<TYPE>& right) {

    TYPE Sum = 0;
    for (size_t i=0; i<6; ++i) {
        Sum+=left.v[i]*right.v[i];
    }
    return Sum;
}

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* VECTOR_H_ */
