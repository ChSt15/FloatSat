#include "matlib.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template<>
Matrix_<1,1,double> Matrix_<1,1,double>::invert() const{
	double arr[1] = {1/this->r[0][0]};
	return Matrix_<1,1,double>(arr);
}

template<>
Matrix_<2,2,double> Matrix_<2,2,double>::invert() const{
	double arr[4];
	double det = this->determinant();
	arr[0] = {this->r[1][1] / det};
	arr[1] = {-this->r[0][1] / det};
	arr[2] = {-this->r[1][0] / det};
	arr[3] = {this->r[0][0] / det};
	return Matrix_<2,2,double>(arr);
}

template<>
Matrix_<3,3,double> Matrix_<3,3,double>::invert() const{
	double arr[9];
	double a = this->r[0][0];
	double b = this->r[0][1];
	double c = this->r[0][2];
	double d = this->r[1][0];
	double e = this->r[1][1];
	double f = this->r[1][2];
	double g = this->r[2][0];
	double h = this->r[2][1];
	double i = this->r[2][2];
	double det = this->determinant();
	arr[0] = {(e*i-f*h) / det};
	arr[1] = {(c*h-b*i) / det};
	arr[2] = {(b*f-c*e) / det};
	arr[3] = {(f*g-d*i) / det};
	arr[4] = {(a*i-c*g) / det};
	arr[5] = {(c*d-a*f) / det};
	arr[6] = {(d*h-e*g) / det};
	arr[7] = {(b*g-a*h) / det};
	arr[8] = {(a*e-b*d) / det};
	return Matrix_<3,3,double>(arr);
}

template <>
double Matrix_<1,1,double>::determinant() const{
	return r[0][0];
}

template <>
double Matrix_<2,2,double>::determinant() const{
	return r[0][0]*r[1][1] - r[1][0]*r[0][1];
}

template <>
double Matrix_<3,3,double>::determinant() const{
	double a = this->r[0][0];
	double b = this->r[0][1];
	double c = this->r[0][2];
	double d = this->r[1][0];
	double e = this->r[1][1];
	double f = this->r[1][2];
	double g = this->r[2][0];
	double h = this->r[2][1];
	double i = this->r[2][2];
	return a*e*i+b*f*g+c*d*h-g*e*c-h*f*a-i*d*b;
}

template<>
Matrix_<1,1,float> Matrix_<1,1,float>::invert() const{
	float arr[1] = {1/this->r[0][0]};
	return Matrix_<1,1,float>(arr);
}

template<>
Matrix_<2,2,float> Matrix_<2,2,float>::invert() const{
	float arr[4];
	float det = this->determinant();
	arr[0] = {this->r[1][1] / det};
	arr[1] = {-this->r[0][1] / det};
	arr[2] = {-this->r[1][0] / det};
	arr[3] = {this->r[0][0] / det};
	return Matrix_<2,2,float>(arr);
}

template<>
Matrix_<3,3,float> Matrix_<3,3,float>::invert() const{
	float arr[9];
	float a = this->r[0][0];
	float b = this->r[0][1];
	float c = this->r[0][2];
	float d = this->r[1][0];
	float e = this->r[1][1];
	float f = this->r[1][2];
	float g = this->r[2][0];
	float h = this->r[2][1];
	float i = this->r[2][2];
	float det = this->determinant();
	arr[0] = {(e*i-f*h) / det};
	arr[1] = {(c*h-b*i) / det};
	arr[2] = {(b*f-c*e) / det};
	arr[3] = {(f*g-d*i) / det};
	arr[4] = {(a*i-c*g) / det};
	arr[5] = {(c*d-a*f) / det};
	arr[6] = {(d*h-e*g) / det};
	arr[7] = {(b*g-a*h) / det};
	arr[8] = {(a*e-b*d) / det};
	return Matrix_<3,3,float>(arr);
}

template <>
float Matrix_<1,1,float>::determinant() const{
	return r[0][0];
}

template <>
float Matrix_<2,2,float>::determinant() const{
	return r[0][0]*r[1][1] - r[1][0]*r[0][1];
}

template <>
float Matrix_<3,3,float>::determinant() const{
	float a = this->r[0][0];
	float b = this->r[0][1];
	float c = this->r[0][2];
	float d = this->r[1][0];
	float e = this->r[1][1];
	float f = this->r[1][2];
	float g = this->r[2][0];
	float h = this->r[2][1];
	float i = this->r[2][2];
	return a*e*i+b*f*g+c*d*h-g*e*c-h*f*a-i*d*b;
}

//=====================================================Implementierung Globale Methoden============================================

int64_t faculty(const int &x) {
    int64_t faculty = 1;
    for (int i = 2; i <= x; ++i)
        faculty *= i;
    return faculty;
}

//_________________________________________________________________

double FMod2p( const double &x) {
    double rval = fmod(x, 2*M_PI);
    if(rval < 0.0) rval+= 2*M_PI;
    return rval;
}

//===Rn nach WGS84
// angle in rad
double R_n(const double angle) {
    double a,e2,f;
    double Rn;
    a = A_WGS84; //m
    f = F_WGS84;

    e2 =1-(1-f)*(1-f);

    double phi = angle;
    double sinp = sin(phi);

    double tmp = sqrt(1-e2*sinp*sinp);
    Rn = a/tmp;
    return Rn;

}
//_________________________________________________________________

double daysSinceY2k(int year,int month,int day,int hour,int minute,double second) {
    double days2k;
    double temp1,temp2;
    temp1 = -floor(0.25*7*(year+floor(1.0/12*(month+9))));
    temp2 = floor(275*month/9.0)+(1.0/24*(hour+minute/60.0+second/3600.0));
    days2k = 367* year +temp1 + temp2 + day - 730531.5;
    return days2k;
}

//===ECEF2ECI

double utcToJD(int year, int month, int date, int hour, int minute, double second){
	double JD;
	double d2k = daysSinceY2k(year,month,date,hour,minute,second);
	JD = d2k + JD2000;
	return JD;
}

/*
* Sidereal Time differs from solar time because of earth's precession
* GMST means Greenwhich Mean Sidereal Time defined as the hour angle of the vernal equinox
* GAST means Greenwhich Apparent Sidereal Time
* Both angles differ as the true vernal equinox' position is not uniformly but varies with nutation
*/

/*
* The functions jd2GMST and jd2GAST are borrowed
* from matlab versions
* http://de.mathworks.com/matlabcentral/fileexchange/28176-julian-date-to-greenwich-mean-sidereal-time
* http://de.mathworks.com/matlabcentral/fileexchange/28232-convert-julian-date-to-greenwich-apparent-sidereal-time
* Copyright (c) 2010, Darin Koblick
* All rights reserved.
*/

double jd2GMST(double jd){
	double gmst;
	double jdmin = floor(jd) - 0.5;
	double jdmax = floor(jd) + 0.5;
	double jd0 = jd;
	if(jd > jdmax) jd0 = jdmax;
	else jd0 = jdmin;
	double h = (jd - jd0) * 24;
    double d = jd - JD2000;
	double d0 = jd0 - JD2000;
	double t = d/36525;
	gmst = fmod(6.697374558 + 0.06570982441908*d0
					+ 1.00273790935*h + 0.000026*(t*t),24);
	gmst = grad2Rad(gmst * 15);
	return gmst;
}

double jd2GAST(double jd){
	double gast;
	double thetam = jd2GMST(jd);
	double t = (jd-JD2000)/36525;

	double epsilonm = grad2Rad(23.439291-0.0130111*t - 1.64E-07*(t*t) + 5.04E-07*(t*t*t));
	double l = grad2Rad(280.4665 + 36000.7698 * t);
	double dL = grad2Rad(218.3165 + 481267.8813*t);
	double omega = grad2Rad(125.04452 - 1934.136261*t);

	double dPSI = -17.20*sin(omega) - 1.32*sin(2*l) - 0.23*sin(2*dL)
			+ 0.21*sin(2*omega);
	double dEPSILON = 9.20*cos(omega) + 0.57*cos(2*l) + 0.10*cos(2*dL)
			-0.09*cos(2*omega);

	dPSI = grad2Rad(dPSI*(1./3600));
	dEPSILON = grad2Rad(dEPSILON*(1./3600));
	gast = fmod(thetam+dPSI*cos(epsilonm+dEPSILON),2*M_PI);
	return gast;
}

Matrix3D eciToECEFrotmat(double jd){
	double theta = jd2GAST(jd);
	Matrix3D rot;
    rot.rotationZ(theta);
	return rot;
}

//====ECEF (meter, meter, meter) to ECI (meter, meter, meter)
Vector3D ecefToECI(const Vector3D ecef, int64_t utcNanoseconds){
	double jd =  ((double)utcNanoseconds / (double)DAYS) + JD2000;
	Matrix3D rot = eciToECEFrotmat(jd);
	return  ecef.matVecMult(rot);
}

//====ECI (meter, meter, meter) to ECEF (meter, meter, meter)
Vector3D eciToECEF(const Vector3D eci, int64_t utcNanoseconds){
	double jd =  ((double)utcNanoseconds / (double)DAYS) + JD2000;
	Matrix3D rot = eciToECEFrotmat(jd);
	return eci.matVecMult(rot.transpose());
}


double eciToECEF(int64_t utcNanoseconds) {  //DEPRECATED

    double days2k = (double)utcNanoseconds / (double)DAYS;
    double angle  = (280.46061837+360.98564736628*days2k)/180*M_PI;
    angle         = FMod2p(angle);
    return angle;
}

//===Geodetic (height, longitude, latitude) to ECEF
Vector3D geodeticToECEF(const Polar& polar) {
    double x,y,z;
    double e2,f;
    f = F_WGS84;
    e2 = 1-(1-f)*(1-f);

    double h      = polar.r;
    double phi    = polar.phi;
    double lambda = polar.theta;
    double rn = R_n(phi);

    x = (rn+h) * cos(phi) * cos(lambda);
    y = (rn+h) * cos(phi) * sin(lambda);
    z = (rn*(1-e2)+h) * sin(phi);

    Vector3D ecef(x,y,z);
    return ecef;
}

//====ECEF to Geodetic===  latitude longitude height (phi,lambda,h)from x,y,z

Polar ecefToGeodetic(const Vector3D& other) {
    double lambda,phi,h;
    double a,b,e2,f;
    a = A_WGS84; //m
    f = F_WGS84;
    b = a*(1-f);
    e2 = 1-(b*b)/(a*a);

    double x,y,z;
    x = other.x;
    y = other.y;
    z = other.z;

    double p = sqrt(x*x+y*y);
    double phi0 = atan2(z,(p*(1-e2)));
    double n0 = R_n(phi0);
    double h0 = (p / cos(phi0)) - n0;

    double eht = 1e-5;
    double ephi = 1e-12;

    h = 0;
    phi = 0;
    double dh = 1;
    double dphi = 1;
    double n = 0;

    while((dh>eht) | (dphi > ephi)){
		phi = phi0;
		n = n0;
		h = h0;
		phi0 = atan2(z,p*(1-e2*(n/(n+h))));
		n0 = R_n(phi);
		h0 = p/cos(phi) - n;
		dh = fabs(h0-h);
		dphi = fabs(phi0-phi);
    }

    lambda = atan2(y,x);

    Polar llh(h, phi,lambda);

    return llh;
}

// Intern function, DO NOT use directly, use only  direction_cos_matrix_from_vectors(...)
Matrix3D _DCM_generator(Vector3D fromA, Vector3D toA, Vector3D fromB, Vector3D toB) {

    fromA = fromA.normalize();
    toA   = toA.normalize();
    fromB = fromB.normalize();
    toB   = toB.normalize();

    //Help vector to define a Cartesian coordinates system
    Vector3D fromX = fromA;
    Vector3D fromZ = fromA.cross(fromB);
    Vector3D fromY = fromZ.cross(fromX);

    Vector3D toX = toA;
    Vector3D toZ = toA.cross(toB);
    Vector3D toY = toZ.cross(toX);

    // normalize
    fromX = fromX.normalize();
    fromY = fromY.normalize();
    fromZ = fromZ.normalize();

    toX = toX.normalize();
    toY = toY.normalize();
    toZ = toZ.normalize();

    Matrix3D result = Matrix3D();

    result.r[0][0] = fromX.dot(toX);
    result.r[0][1] = fromX.dot(toY);
    result.r[0][2] = fromX.dot(toZ);

    result.r[1][0] = fromY.dot(toX);
    result.r[1][1] = fromY.dot(toY);
    result.r[1][2] = fromY.dot(toZ);

    result.r[2][0] = fromZ.dot(toX);
    result.r[2][1] = fromZ.dot(toY);
    result.r[2][2] = fromZ.dot(toZ);

    return result;
}

/* cos_direction_matrix_from_vectors
 *  when we use only one vector to rate a body, we get an ambiguous rotation
 *  which has one degree of freedom open.
 *  To get an unambiguous rotation we have to use two not co-linear vectors of the
 *  body. This functions finds the unit rotation which satisfied the rotation
 *  of these both vectors.
 *  But! The body may not be formatted. The geometric relationship of the
 *  two vectors has to be the same before and after the rotation.
 */


// WARNING!!! First we have to rotate to (1,0,0) and (0,1,0) and then continue else it does not work, WHY?!?!?!
Matrix3D direction_cos_matrix_from_vectors(Vector3D fromA, Vector3D toA, Vector3D fromB, Vector3D toB) {
    Vector3D x(1,0,0);
    Vector3D y(0,1,0);

    // first rotate to unit vectors x and y and then to final vector.
    // Else it does not work and we do not know why :(

    Matrix3D m1 = _DCM_generator(fromA, x, fromB, y);
    Matrix3D m2 = _DCM_generator(x, toA, y, toB);

    return m2*m1;
}


#ifndef NO_RODOS_NAMESPACE
}
#endif
