/*
 * polar.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef POLAR_H_
#define POLAR_H_

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template <typename TYPE>
class Vector3D_;

template <typename TYPE = double>
class Polar_ {
public:
	TYPE r;
	TYPE phi; 		///< radians
	TYPE theta; 		///< radians

	Polar_() {
	    this->r     = 0;
	    this->phi   = 0;
	    this->theta = 0;
	}

	Polar_(const TYPE &r,const TYPE &phi,const TYPE &theta) {
	    this->r     = r;
	    this->phi   = phi;
	    this->theta = theta;
	}

	Polar_(const Polar_<TYPE>& other) {
	    this->r     = other.r;
	    this->phi   = other.phi;
	    this->theta = other.theta;
	}

	Polar_(const Vector3D_<TYPE>& other) {
	    this->r     = sqrt(other.x*other.x + other.y*other.y + other.z*other.z) ;
	    this->phi   = atan2(other.y,other.x);
	    this->theta = acos(other.z/this->r);
	}

	Vector3D_<TYPE> toCartesian() const {
		TYPE x = this->r*sin(this->theta)*cos(this->phi);
		TYPE y = this->r*sin(this->theta)*sin(this->phi);
		TYPE z = this->r*cos(this->theta);
	    Vector3D_<TYPE> cartesian(x,y,z);

	    return cartesian;
	}

	void print() const {
	    PRINTF("[r= %3.3lf \t phi= %3.3lf*PI \t theta= %3.3lf*PI]\n", static_cast<double>(r), static_cast<double>(phi/M_PI), static_cast<double>(theta/M_PI));
	}

	template <typename TYPE2>
	Polar_<TYPE> operator=(const Polar_<TYPE2> &other) {
		this->r = other.r;
		this->phi = other.phi;
		this->theta = other.theta;
		return *this;
	}

};

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* POLAR_H_ */
