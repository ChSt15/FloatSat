/*
 * complex
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef COMPLEX_
#define COMPLEX_

#include "matlib.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template<typename TYPE = double>
class Complex_ {
public:
	TYPE Re;
	TYPE Im;

	Complex_() {
	    this->Re = 0;
	    this->Im = 0;
	}

	Complex_(const TYPE &Re, const TYPE &Im) {
	    this->Re = Re;
	    this->Im = Im;
	}

	Complex_<TYPE> cAdd(const Complex_<TYPE>& other) const {
		Complex_<TYPE> z;
	    z.Re = other.Re + this->Re;
	    z.Im = other.Im + this->Im;
	    return z;
	}

	Complex_<TYPE> cSub(const Complex_<TYPE>& other) const {
		Complex_<TYPE> z;
	    z.Re = other.Re - this->Re;
	    z.Im = other.Im - this->Im;
	    return z;
	}

	Complex_<TYPE> cScale(const TYPE &scale) const {
		Complex_<TYPE> z =(*this);
	    z.Re *= scale;
	    z.Im *= scale;
	    return z;
	}

	Complex_<TYPE> cMult(const Complex_<TYPE>& other) const {
		Complex_<TYPE> z;
	    z.Re = this->Re*other.Re -(other.Im*this->Im);
	    z.Im = this->Re*other.Im +this->Im*other.Re;

	    return z;
	}

	Complex_<TYPE> cPow(const int &exponent) const {
		Complex_<TYPE> z;
	    if(exponent ==0) {
	        z.Re = 1;
	        return z;
	    }
	    z= (*this);
	    for(int i = 1; i<exponent; ++i) {
	        z=z.cMult(*this);
	    }
	    return z;
	}

	Complex_<TYPE> cExp() const {
		Complex_<TYPE> z;
	    z.Re = exp(this->Re)*cos(this->Im);
	    z.Im = exp(this->Re)*sin(this->Im);
	    return z;
	}

	template <typename TYPE2>
	Complex_<TYPE> operator=(const Complex_<TYPE2> &other) {
		this->Re = other.Re;
		this->Im = other.Im;
		return *this;
	}

    inline friend Complex_<TYPE> operator+ (const Complex_<TYPE> &left, const Complex_<TYPE> &right) { return left.cAdd(right); }
    inline friend Complex_<TYPE> operator- (const Complex_<TYPE> &left, const Complex_<TYPE> &right) { return right.cSub(left); }
    inline friend Complex_<TYPE> operator* (const TYPE  &left, const Complex_<TYPE> &right) { return right.cScale(left); }
    inline friend Complex_<TYPE> operator* (const Complex_<TYPE> &left, const TYPE  &right) { return left.cScale(right); }
    inline friend Complex_<TYPE> operator* (const Complex_<TYPE> &left, const Complex_<TYPE> &right) { return left.cMult(right); }
    inline friend Complex_<TYPE> operator/ (const Complex_<TYPE> &left, const TYPE  &right) { return left.cScale(1/right); }
    inline friend bool operator==   (const Complex_<TYPE> &left, const Complex_<TYPE> &right) { return (isAlmost0(left.Re - right.Re) && isAlmost0(left.Im - right.Im));}
    inline friend bool operator!=   (const Complex_<TYPE> &left, const Complex_<TYPE> &right) { return !(left == right);}

};

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* COMPLEX_ */
