/*
 * coordinateframe3d.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef COORDINATEFRAME3D_H_
#define COORDINATEFRAME3D_H_

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template<typename TYPE>
class Vector3D_;
template<typename TYPE>
class Matrix3D_;
template<typename TYPE>
class HTransform_;
template<typename TYPE>
class Quaternion_;

/** karstesisches Koordinatensystems im euklidischem Raum
      3 orthogonale Einheitsvektoren x,y und z + ein Urpsrung */
template<typename TYPE = double>
class CoordinateFrame3D_ {
public:
	Vector3D_<TYPE> x; ///< Einheitsvektor in x-Richtung
	Vector3D_<TYPE> y; ///< Einehitsvektor in y-Richtung
	Vector3D_<TYPE> z; ///< Einheitsvektor in z-Richtung
	Vector3D_<TYPE> origin; ///< Ursprung des Koordinatensystems


	CoordinateFrame3D_(){
	    Vector3D_<TYPE> x(1,0,0);
	    Vector3D_<TYPE> y(0,1,0);
	    Vector3D_<TYPE> z(0,0,1);
	    Vector3D_<TYPE> o;

	    this->x=x;
	    this->y=y;
	    this->z=z;
	    this->origin = o;
	}

	CoordinateFrame3D_(const Vector3D_<TYPE>& x, const Vector3D_<TYPE>& y,const Vector3D_<TYPE>& z, const Vector3D_<TYPE>& origin) {
	    this->x = x.normalize();
	    this->y = y.normalize();
	    this->z = z.normalize();
	    this->origin=origin;
	}

	CoordinateFrame3D_(const Vector3D_<TYPE>& x, const Vector3D_<TYPE>& y,const Vector3D_<TYPE>& origin) {
	    this->x=x.normalize();
	    this->y= x.cross(y).normalize();
	    this->z = x.cross(this->y).normalize();
	    this->origin = origin;
	}

	HTransform_<TYPE> mapTo(const CoordinateFrame3D_<TYPE>& rotated) const { // derives rotation matrix A : [v]r = A[v]f [v]f given
	    Matrix3D_<TYPE> R;
	    //column1
	    R.r[0][0] = this->x.dot(rotated.x);
	    R.r[1][0] = this->y.dot(rotated.x);
	    R.r[2][0] = this->z.dot(rotated.x);


	    //column 2
	    R.r[0][1] = this->x.dot(rotated.y);
	    R.r[1][1] = this->y.dot(rotated.y);
	    R.r[2][1] = this->z.dot(rotated.y);


	    //column 3
	    R.r[0][2] = this->x.dot(rotated.z);
	    R.r[1][2] = this->y.dot(rotated.z);
	    R.r[2][2] = this->z.dot(rotated.z);

	    Vector3D_<TYPE> trans;
	    trans = rotated.origin.vecSub(this->origin);
	    HTransform_<TYPE> Rot(R,trans);

	    return Rot;

	}

	CoordinateFrame3D_<TYPE> translate(const Vector3D_<TYPE>& trans) const {
	    CoordinateFrame3D_<TYPE> T((*this));
	    T.origin = this->origin.vecAdd(trans);

	    return T;
	}

	CoordinateFrame3D_<TYPE> rotate(const Matrix3D_<TYPE>& rot) const {
	    CoordinateFrame3D_<TYPE> R;
	    R.x = this->x.mRotate(rot);
	    R.y = this->y.mRotate(rot);
	    R.z = this->z.mRotate(rot);
	    R.origin= this->origin;

	    return R;
	}

	CoordinateFrame3D_<TYPE> rotate(const Quaternion_<TYPE>& q) const {
	    CoordinateFrame3D_<TYPE> R;
	    R.x = this->x.qRotate(q);
	    R.y = this->y.qRotate(q);
	    R.z = this->x.qRotate(q);
	    R.origin= this->origin;

	    return R;
	}

	template <typename TYPE2>
	CoordinateFrame3D_<TYPE> operator=(const CoordinateFrame3D_<TYPE2> &other) {
		this->x.x = other.x.x;
		this->x.y = other.x.y;
		this->x.z = other.x.z;

		this->y.x = other.x.x;
		this->y.y = other.x.y;
		this->y.z = other.x.z;

		this->z.x = other.z.x;
		this->z.y = other.z.y;
		this->z.z = other.z.z;

		this->origin.x = other.origin.x;
		this->origin.y = other.origin.y;
		this->origin.z = other.origin.z;
		return *this;
	}

};

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* COORDINATEFRAME3D_H_ */
