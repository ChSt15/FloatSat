/*
 * homogenous.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef HOMOGENOUS_H_
#define HOMOGENOUS_H_

#include "generic.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

// Homogenous Transformation
template <typename TYPE = double>
class HTransform_ : public Matrix_<4,4,TYPE> {
public:
	HTransform_() : Matrix_<4,4,TYPE>("Htransform"){
	}

	HTransform_(const Matrix_<4,4,TYPE>& other) : Matrix_<4,4,TYPE>(other){
	}

	HTransform_(const Matrix3D_<TYPE>& rot, const Vector3D_<TYPE>& trans) : Matrix_<4,4,TYPE>("Htransform"){
	    for(size_t i = 0 ; i<3; ++i) {
	        for(size_t j = 0; j<3; ++j) {
	            this->r[i][j]=rot.r[i][j];
	        }
	    }
	    //column 4
	    this->r[0][3] = trans.x;
	    this->r[1][3] = trans.y;
	    this->r[2][3] = trans.z;
	    this->r[3][3] = 1;
	    //row 4
	    this->r[3][0]=0;
	    this->r[3][1]=0;
	    this->r[3][2]=0;
	}

	HTransform_(TYPE* arr) : Matrix_<4,4,TYPE>(arr, "Htransform"){ 				///< arr[16], left to right, top  to bottom
	}

	Matrix3D_<TYPE> getRotation() const {
	    Matrix3D_<TYPE> R;
	    for(size_t i = 0 ; i<3; ++i) {
	        for(size_t j = 0; j<3; ++j) {
	            R.r[i][j]=this->r[i][j];
	        }
	    }
	    return R;
	}

	Vector3D_<TYPE> getTranslation() const {
	    Vector3D_<TYPE> trans(this->r[0][3],this->r[1][3],this->r[2][3]);
	    return trans;

	}

	HTransform_<TYPE> hinvert() const {
	    Matrix3D_<TYPE> transpose = this->getRotation().transpose();
	    Vector3D_<TYPE> invtrans  = this->getTranslation().matVecMult(transpose);
	    invtrans = invtrans.scale(-1);
	    HTransform_<TYPE> inverse(transpose,invtrans);

	    return inverse;
	}
	
};

template <typename TYPE>
class HCoord_ : public Vector_<4,TYPE>{
public:


	HCoord_() : Vector_<4,TYPE>("HCoord"){
	}


	HCoord_ (const TYPE &x, const TYPE &y, const TYPE &z, const TYPE &scale = 1)  : Vector_<4,TYPE>("HCoord") {
		this->r[0][0] = x;
		this->r[1][0] = y;
		this->r[2][0] = z;
		this->r[3][0] = scale;
	}

	
	HCoord_(const Vector_<4,TYPE>& other) : Vector_<4,TYPE>(other, "HCoord"){
	}
	
	HCoord_(const Matrix_<4,1,TYPE>& other) : Vector_<4,TYPE>(other, "HCoord"){
	}

	HCoord_ (TYPE arr[4]) : Vector_<4,TYPE>(arr, "HCoord"){
	}

	Vector3D_<TYPE>to3D() const {
		Vector3D_<TYPE> v(this->r[0][0]/ this->r[3][0], this->r[1][0]/ this->r[3][0], this->r[2][0]/ this->r[3][0]);
		return v;
	}
	
	HCoord_<TYPE>& operator=(const Matrix_<4,1,TYPE>& other){
		this->r[0][0] = other.r[0][0];
		this->r[1][0] = other.r[1][0];
		this->r[2][0] = other.r[2][0];
		this->r[3][0] = other.r[3][0];
		return *this;
	}
};


#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* HOMOGENOUS_H_ */
