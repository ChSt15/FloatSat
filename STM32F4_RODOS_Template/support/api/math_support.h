/*
 * math_support.h
 *
 *  Created on: Oct 14, 2016
 *      Author: strohmi
 */

#ifndef MATH_SUPPORT_H_
#define MATH_SUPPORT_H_


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern const double EPSILON;

template <typename TYPE>
inline TYPE sgn(TYPE val) {
    return static_cast<TYPE>((static_cast<TYPE>(0) < val) - (val < static_cast<TYPE>(0)));
}

template <typename TYPE>
inline bool   isAlmost0(const TYPE &a)   { return (a * sgn(a)) <= static_cast<TYPE>(EPSILON); }

template <typename TYPE>
inline double grad2Rad(const TYPE &grad) {return grad * M_PI / 180.0; }

template <typename TYPE>
inline double rad2Grad(const TYPE &rad)  {return rad * 180.0 / M_PI; }

template <typename TYPE>
inline TYPE square( const TYPE &x )    { return x*x; }

template <typename TYPE>
inline TYPE frac( const TYPE &x )      {
	if(x >= 0) {
		return (TYPE) (x - floor(x));
	} else {
		return (TYPE) (x - ceil(x));
	}
}

template <typename TYPE, typename TYPE2>
inline TYPE mod( const TYPE &dividend, const TYPE2 &divisor ) {
	if(isAlmost0(divisor)){
		return 0;
	} else {
		return dividend - (divisor * int(dividend/divisor));
	}
}
int64_t faculty(const int &x);
double FMod2p( const double &x); ///< doubleing point rest, after division with 2Pi

#ifndef NO_RODOS_NAMESPACE
}
#endif

#include "typedefs.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif


/**
jd means Julian Date, which describes the days from 1st January −4712 (4713 bc) 12:00 o' clock
*/
extern const double JD2000; //days  -- Julian Date on 01.01.2000 12'o clock
extern const double A_WGS84; // m
extern const double F_WGS84;

double R_n(const double degreeOfLatitude); ///< earthradius in meter at given latitude (rad)

/**
Reference Coordinate Systems

ECEF - Earth Centered Earth Fixed, x-Axis pointing to International Reference Meridian
ECI - Earth Centered Inertial, x-Axis aligned to Earth's Mean Equator and Equinox at 12:00 Terrestrial Time on 1 January 2000
Geodetic System, based on WGS84 Ellipsoid, defined by latitude, longitude, altitude
*/
double daysSinceY2k(int year,int month,int day,int hour,int minute,double second);
double utcToJD(int year, int month, int date, int hour, int minute, double second);
double jd2GAST(double jd);
double jd2GMST(double jd);
Matrix3D eciToECEFrotmat(double jd);
double eciToECEF(int64_t utcNanoseconds);  //DEPRECATED
Vector3D eciToECEF(const Vector3D ecef, int64_t  utcNanoseconds); ///<(meter, meter, meter) -> (meter, meter, meter)
Vector3D ecefToECI(const Vector3D eci, int64_t  utcNanoseconds);///<(meter, meter, meter) -> (meter, meter, meter)
Vector3D geodeticToECEF(const Polar& polar); ///< Warning! Earth-surface (meter, rad, rad) -> Eart-center (meter, meter, meter)

/**
Converts cartesian coordinates to ellipsoidal.
   Uses iterative alogithm with Heiskanen and Moritz's Method (1967)
   This method has been proved fastest when convergence of both
   latitude and ellipsoidal height is required (see H. S. Fok and H.
   Baki Iz "A Comparative Analysis of the Performance of Iterative
   and Non-iterative Solutions to the Cartesian to Geodetic
   Coordinate Transformation")
   Computation time is only slightly worse than for the
   Bowring direct formula but accuracy is better.
*/
Polar    ecefToGeodetic(const Vector3D& other);     ///< Warning! Eart-center (meter, meter, meter) -> Earth-surface (meter, rad, rad)

/* cos_direction_matrix_from_vectors
 *  when we use only one vector to rotate a body, we get an ambiguous rotation
 *  which has one degree of freedom open.
 *  To get an unambiguous rotation we have to use two not co-linear vectors of the
 *  body. This functions finds the unit rotation which satisfied the rotation
 *  of these both vectors.
 *  But! The body may not be formatted. The geometric relationship of the
 *  two vectors has to be the same before and after the rotation.
 */
Matrix3D direction_cos_matrix_from_vectors(Vector3D fromA, Vector3D toA, Vector3D fromB, Vector3D toB);

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* MATH_SUPPORT_H_ */
