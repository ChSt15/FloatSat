/*
 * generic.h
 *
 *  Created on: Jun 29, 2016
 *      Author: strohmi
 */

#ifndef GENERIC_H_
#define GENERIC_H_

#include "rodos.h"
#include "math_support.h"
#include "vector.h"

extern const double EPSILON;

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

template <size_t ROW, size_t COL, typename TYPE = double>
class Matrix_;

template <size_t ROW, typename TYPE = double>
class Vector_;

template <size_t ROW, size_t COL, typename TYPE>
class Matrix_ {
public:
	const char *label;
	TYPE r[ROW][COL];

	Matrix_(const char* label = "matrix") {
		static_assert((ROW > 0 && COL > 0), "Matrix_ dimensions must be 1x1 at least");
		this->label = label;
		for(size_t i = 0; i < ROW; ++i){
			for(size_t j = 0; j < COL; ++j){
				r[i][j] = 0;
			}
		}
	}

	Matrix_(const Matrix_<ROW, COL, TYPE>& other, const char* label = "matrix"){
		static_assert((ROW > 0 && COL > 0), "Matrix_ dimensions must be 1x1 at least");
		this->label = label;
		for(unsigned int i = 0; i < ROW; ++i){
			for(unsigned int j = 0; j < COL; ++j){
				r[i][j] = other.r[i][j];
			}
		}
	}

	Matrix_(const Vector_<ROW, TYPE>& other, const char* label = "matrix"){
		static_assert((ROW > 0), "Matrix_ dimensions must be 1x1 at least");
		this->label = label;
		for(size_t i = 0; i < ROW; ++i){
			if(COL > 1)
				r[i][i] = other.r[i][i];
			else
				r[i][0] = other.r[i][0];
		}
	}

	Matrix_(const TYPE* arr, const char* label = "matrix"){
		static_assert((ROW > 0 && COL > 0), "Matrix_ dimensions must be 1x1 at least");
		this->label = label;
		size_t k = 0;
		for(size_t i = 0; i < ROW; ++i){
			for(size_t j = 0; j < COL; ++j){
				r[i][j] = arr[k];
				k++;
			}
		}
	}

	static Matrix_<ROW, COL, TYPE> eye(TYPE val = 1){
		Matrix_<ROW,COL, TYPE> ret;
		size_t diag = ROW < COL ? ROW : COL;
		for(size_t i = 0; i < diag; ++i){
			ret.r[i][i] = 1 * val;
		}
		return ret;
	}

	static Matrix_<ROW, COL, TYPE> diag(TYPE* array){
		Matrix_<ROW,COL, TYPE> ret;
		size_t diag = ROW < COL ? ROW : COL;
		for(size_t i = 0; i < diag; ++i){
			ret.r[i][i] = array[i];
		}
		return ret;
	}

	explicit Matrix_(const Vector3D_<TYPE> &other, const char* label = "matrix"){
		if(ROW == 3 && COL == 1){
			this->label = label;
			r[0][0] = other.x;
			r[1][0] = other.y;
			r[2][0] = other.z;
		} else {
			PRINTF("Invalid use of Matrix_ constructor -- Vector3D_\n");
		}
	}

	explicit Matrix_(const Vector6D_<TYPE> &other, const char* label = "matrix"){
		if(ROW == 6 && COL == 1){
		this->label = label;
			for(size_t i = 0; i < 6; ++i){
				r[i][0] = other.v[i];
			}
		} else {
			PRINTF("Invalid use of Matrix_ constructor -- Vector6D_\n");
		}
	}

	Matrix_<ROW, 1, TYPE> diag() const {
		static_assert((ROW == COL), "Matrix_ must be NxN in diag()");
		Matrix_<ROW, 1> dia;
		for (size_t i = 0; i < ROW; ++i) {
			dia.r[i][0] = r[i][i];
		}
		return dia;
	}

	TYPE trace() const {
		static_assert((ROW == COL), "Matrix_ must be NxN in trace()");
		TYPE trace = 0;
		for (size_t i = 0; i < ROW; ++i) {
			trace += r[i][i];
		}
		return trace;
	}

	TYPE determinant() const {
		static_assert((ROW == COL), "Matrix_ must be NxN in determinant()");
		TYPE det = 0;
		Matrix_<ROW - 1, COL - 1, TYPE> rek;
		for (size_t j1 = 0; j1 < ROW; ++j1) {
			for (size_t i = 1; i < ROW; ++i) {
				size_t j2 = 0;
				for (size_t j = 0; j < ROW; ++j) {
					if (j == j1)
						continue;
					rek.r[i - 1][j2] = r[i][j];
					j2++;
				}
			}
			det += pow(-1.0, j1 + 2.0) * r[0][j1] * rek.determinant();
		}
		return det;
	}

	bool isOrthogonal() const {
		static_assert((ROW == COL), "Matrix_ must be NxN in orthognal()");
		Matrix_<ROW, COL, TYPE> Id = Matrix_<ROW, COL, TYPE>::eye(1);
	    if(this->mMult(this->transpose()).equals(Id)) {
	        return true;
	    }
	    return false;
	}

	template<size_t ROW2, size_t COL2>
	Matrix_<ROW2, COL2, TYPE> getSub(size_t start_row, size_t end_row, size_t start_col, size_t end_col) const {
		Matrix_<ROW2, COL2, TYPE> res;
		for(size_t i = start_row; i <= end_row; ++i){
			for(size_t j = start_col; j <= end_col; ++j){
				res.r[i-start_row][j-start_col] = r[i][j];
			}
		}
		return res;
	}

	Matrix_<ROW, 1, TYPE> getColumn(const size_t &j) const {
		if(j < 0 || j >= COL) {
			PRINTF("Column %d does not exist\n", j);
			return Matrix_<ROW, 1, TYPE>();
		} else {
			TYPE arr[ROW];
			for(size_t i = 0; i < ROW; ++i){
				arr[i] = r[i][j];
			}
			return Matrix_<ROW, 1, TYPE>(arr);
		}
	}

	Matrix_<1, COL, TYPE> getRow(const size_t &i) const {
		if(i >= ROW) {
			PRINTF("Row %d does not exist\n", (int)i);
			Matrix_<1, COL, TYPE> ret;
			return ret;
		} else {
			TYPE arr[COL];
			for(size_t j = 0; j < COL; ++j){
				arr[j] = r[i][j];
			}
			return Matrix_<1, COL, TYPE>(arr);
		}
	}

	template<size_t ROW2, size_t COL2>
	void setAt(size_t row, size_t col, Matrix_<ROW2, COL2, TYPE> sub){
		if(ROW2+row > ROW) return;
		if(COL2+col > COL) return;
		for(size_t i = row; i < ROW2+row; ++i){
			for(size_t j = col; j < COL2+col; ++j){
				r[i][j] = sub.r[i-row][j-col];
			}
		}
	}

	void setColumn(const size_t &j, const Matrix_<ROW,1, TYPE>& column) {
		if(j >= COL) {
			PRINTF("Column %d does not exist\n", (int)j);
		} else {
			for(size_t i = 0; i < ROW; ++i){
				r[i][j] =  column.r[i][0];
			}
		}
	}

	void setRow(const size_t &i, const Matrix_<1,COL, TYPE>& row) {
		if(i < 0 || i >= ROW) {
			PRINTF("Row does not exist\n");
		} else {
			for(size_t j = 0; j < COL; ++j){
				r[i][j] = row.r[0][j];
			}
		}
	}

	bool equals(const Matrix_<ROW, COL, TYPE>& other) const {
		for(size_t i = 0; i < ROW; ++i)
			for(size_t j = 0; j < COL; ++j)
				if(!isAlmost0(r[i][j] - other.r[i][j]))
					return false;
		return true;
	}



	Matrix_<ROW, COL, TYPE> cofac() const{
		static_assert((ROW == COL), "Matrix_ must be NxN in cofac()");

		Matrix_<ROW, COL, TYPE> cofac;
		Matrix_<ROW-1,COL-1, TYPE> c;

		for(size_t i = 0; i < ROW; ++i) {
			for(size_t j = 0; j < COL; ++j) {
				/* Form the adjoint a_ij */
				size_t i1 = 0;
				for(size_t ii = 0; ii < ROW; ++ii) {
					if (ii == i)
						continue;
					size_t j1 = 0;
					for(size_t jj = 0; jj < COL; ++jj) {
						if (jj == j)
							continue;
						c.r[i1][j1] = r[ii][jj];
						j1++;
					}
					i1++;
				}

				/* Calculate the determinate */
				TYPE det = c.determinant();

				/* Fill in the elements of the cofactor */
				cofac.r[i][j] = pow(-1.0, i + j + 2.0) * det;
			}
		}
		return cofac;
	}

	Matrix_<ROW, COL, TYPE> adjoint() const{
		static_assert((ROW == COL), "Matrix_ must be NxN in adjoint()");
		Matrix_<ROW, COL, TYPE> cofac;
		cofac = this->cofac();
		return cofac.transpose();
	}

	Matrix_<COL, ROW, TYPE> transpose() const {
		Matrix_<COL, ROW, TYPE> trans;
		for(size_t i = 0; i < ROW; ++i)
			for(size_t j = 0; j < COL; ++j)
				trans.r[j][i] = r[i][j];
		return trans;
	}

	Matrix_<ROW, COL, TYPE> invert() const {
		static_assert((ROW == COL), "Matrix_ must be NxN in invert()");
		Matrix_<ROW, COL, TYPE> inv;
		TYPE det = this->determinant();
		if(det == 0){
			PRINTF("Inverse does not exist\n");
			return inv;
		} else {
			inv = this->adjoint();
			inv = inv.scale(1/det);
			return inv;
		}
	}

	Matrix_<ROW, COL, TYPE> scale(const TYPE &factor) const{
		Matrix_<ROW,COL, TYPE> scaled;
		for(size_t i = 0; i < ROW; ++i)
			for(size_t j = 0; j < COL; ++j){
				scaled.r[i][j] = r[i][j] * factor;
			}
		return scaled;
	}

	Matrix_<ROW, COL, TYPE> mAdd(const Matrix_<ROW, COL, TYPE>& other) const {
		Matrix_<ROW, COL, TYPE> sum;
		for(size_t i = 0; i < ROW; ++i)
			for(size_t j = 0; j < COL; ++j)
				sum.r[i][j] = r[i][j] + other.r[i][j];
		return sum;
	}

	Matrix_<ROW, COL, TYPE> mSub(const Matrix_<ROW, COL, TYPE>& other) const {
		Matrix_<ROW, COL, TYPE> diff;
		for(size_t i = 0; i < ROW; ++i)
			for(size_t j = 0; j < COL; ++j)
				diff.r[i][j] = r[i][j] - other.r[i][j];
		return diff;
	}

	template <size_t COL2>
	Matrix_<ROW, COL2, TYPE> mMult(const Matrix_<COL, COL2, TYPE>& other) const {
		Matrix_<ROW, COL2, TYPE> prod;
		for(size_t i = 0; i < ROW; ++i)
			for(size_t j = 0; j < COL2; ++j) {
				for(size_t k = 0; k < COL; ++k)
					prod.r[i][j] = prod.r[i][j] + r[i][k] * other.r[k][j];
			}
		return prod;
	}

	Matrix_<ROW, COL, TYPE> mDivide(const Matrix_<ROW, COL, TYPE>& other) const {
		static_assert((ROW == COL), "Matrix_ is not NxN");
	    Matrix_<ROW, COL, TYPE> inverse, divide;
	    inverse = other.invert();
	    divide  = this->mMult(inverse);
	    return divide;
	}

	void print() const {
		size_t i,j;
		if(label != NULL)
			xprintf("\n%s = \n[", label);
		else
			xprintf("\nMatrix<%d,%d> = \n[", static_cast<int>(ROW),static_cast<int>(COL));
		for (i = 0; i < ROW; ++i) {
			for (j = 0; j < COL; ++j) {
				if(j!=0||i!=0)
					xprintf(" ");

				if(r[i][j] != r[i][j])
					xprintf(" NaN ");
				else {
					if(r[i][j] < 1e-3){
						double cov = (double) r[i][j];
						size_t exp = 0;
						while (fabs(cov) < 1 && exp < 15){
							exp++;
							cov *= 10;
						}
						xprintf("%4.4lfe-%d ", cov, exp);
					} else if (r[i][j] > 1e4){
						double cov = (double) r[i][j];
						size_t exp = 0;
						while (fabs(cov) > 1 && exp < 15){
							exp++;
							cov /= 10;
						}
						xprintf("%4.4lfe%d ", cov, exp);
					} else {
						xprintf(" %4.4lf ", (double) r[i][j]);
					}
				}
			}
			if(i < ROW - 1)
				xprintf("\n");
		}
		xprintf("]\n");
	}

	inline size_t rows(){return ROW;};
	inline size_t cols(){return COL;};

	inline friend Matrix_<ROW,COL,TYPE> operator+(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<ROW,COL,TYPE> &right) { return left.mAdd(right); }
	inline friend Matrix_<ROW,COL,TYPE> operator-(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<ROW,COL,TYPE> &right) { return left.mSub(right); }
	inline friend Matrix_<ROW,COL,TYPE> operator-(const Matrix_<ROW,COL,TYPE> &left) { return left.scale(-1); }
	template<size_t COL2>
	inline friend Matrix_<ROW,COL2,TYPE> operator*(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<COL,COL2,TYPE> &right) { return left.mMult(right); }
	inline friend bool operator==(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<ROW,COL,TYPE> &right) { return left.equals(right); }
	inline friend bool operator!=(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<ROW,COL,TYPE> &right) { return !(left.equals(right)); }

	inline Matrix_<3,1,TYPE> operator= (const Vector3D_<TYPE> &other) {*this = (Matrix_<3,1,TYPE>)other; return *this; }
	inline Matrix_<6,1,TYPE> operator= (const Vector6D_<TYPE> &other) {*this = (Matrix_<6,1,TYPE>)other; return *this; }

	Matrix_<ROW, COL, TYPE> operator=(const Matrix_<ROW, COL, TYPE> &other) {
		size_t i, j;
		for (i = 0; i < ROW; ++i) {
			for (j = 0; j < COL; ++j) {
				this->r[i][j] = other.r[i][j];
			}
		}
		return *this;
	}

	template <typename TYPE2>
	Matrix_<ROW, COL, TYPE> operator=(const Matrix_<ROW, COL, TYPE2> &other) {
		size_t i, j;
		for (i = 0; i < ROW; ++i) {
			for (j = 0; j < COL; ++j) {
				this->r[i][j] = other.r[i][j];
			}
		}
		return *this;
	}

	inline friend Matrix_<ROW,COL,TYPE> operator*(const TYPE   &left, const Matrix_<ROW,COL,TYPE> &right) { return right.scale(left); }
	inline friend Matrix_<ROW,COL,TYPE> operator*(const Matrix_<ROW,COL,TYPE> &left, const TYPE   &right) { return left.scale(right); }
	inline friend Matrix_<ROW,COL,TYPE> operator/(const Matrix_<ROW,COL,TYPE> &left, const TYPE   &right) { return left.scale(1/right); }
};

template<> Matrix_<1,1,double> Matrix_<1,1,double>::invert() const;
template<> Matrix_<2,2,double> Matrix_<2,2,double>::invert() const;
template<> Matrix_<3,3,double> Matrix_<3,3,double>::invert() const;
template<> double Matrix_<1,1,double>::determinant() const;
template<> double Matrix_<2,2,double>::determinant() const;
template<> double Matrix_<3,3,double>::determinant() const;

template<> Matrix_<1,1,float> Matrix_<1,1,float>::invert() const;
template<> Matrix_<2,2,float> Matrix_<2,2,float>::invert() const;
template<> Matrix_<3,3,float> Matrix_<3,3,float>::invert() const;
template<> float Matrix_<1,1,float>::determinant() const;
template<> float Matrix_<2,2,float>::determinant() const;
template<> float Matrix_<3,3,float>::determinant() const;

//template<size_t ROW, size_t COL, typename TYPE>
//inline Matrix_<ROW,COL,TYPE> operator+(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<ROW,COL,TYPE> &right) { return left.mAdd(right); }
//template<size_t ROW, size_t COL, typename TYPE>
//inline Matrix_<ROW,COL,TYPE> operator-(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<ROW,COL,TYPE> &right) { return left.mSub(right); }
//template<size_t ROW, size_t COL, size_t COL2, typename TYPE>
//inline Matrix_<ROW,COL2,TYPE> operator*(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<COL,COL2,TYPE> &right) { return left.mMult(right); }
//template<size_t ROW, size_t COL, typename TYPE>
//inline Matrix_<ROW,COL,TYPE> operator==(const Matrix_<ROW,COL,TYPE> &left, const Matrix_<ROW,COL,TYPE> &right) { return left.equals(right); }a


//template<size_t ROW, size_t COL, typename TYPE>
//inline Matrix_<ROW,COL,TYPE> operator*(const TYPE   &left, const Matrix_<ROW,COL,TYPE> &right) { return right.scale(left); }
//template<size_t ROW, size_t COL, typename TYPE>
//inline Matrix_<ROW,COL,TYPE> operator*(const Matrix_<ROW,COL,TYPE> &left, const TYPE   &right) { return left.scale(right); }
//template<size_t ROW, size_t COL, typename TYPE>
//inline Matrix_<ROW,COL,TYPE> operator/(const Matrix_<ROW,COL,TYPE> &left, const TYPE   &right) { return left.scale(1.0/right); }


template<size_t ROW, typename TYPE>
class Vector_ : public Matrix_<ROW, 1, TYPE> {
public:
	Vector_(const char* label = "vector") : Matrix_<ROW,1,TYPE>(label) {}             						///< generates (0,0,0)
	Vector_(const Matrix_<ROW,1,TYPE>& other, const char* label = "vector") : Matrix_<ROW,1,TYPE>(other, label){}

	Vector_(TYPE* arr, const char* label = "vector") : Matrix_<ROW,1,TYPE>(arr, label){}        				///< like Vector3D_ (arr[0],arr[1],arr[2])

	Vector_(const Vector3D_<TYPE>& other, const char* label = "Vector3D_") : Matrix_<3,1,TYPE>(label){
		this->r[0][0] = other.x;
		this->r[1][0] = other.y;
		this->r[2][0] = other.z;
	}
	Vector_(const Vector6D_<TYPE>& other, const char* label = "Vector6D_") : Matrix_<6,1,TYPE>(other.v, label){}

	inline Vector_<ROW, TYPE> vecAdd(const Vector_<ROW, TYPE>& other) const {return this->mAdd(other);}
	inline Vector_<ROW, TYPE> vecSub(const Vector_<ROW, TYPE>& other) const {return this->mSub(other);}

	TYPE getAngle(const Vector_<ROW, TYPE>& other) const {
	    TYPE angle,product,len;
	    len = this->getLen() * other.getLen() ;
	    product = this->dot(other);
	    angle = acos(product/len);
	    return angle;   // radians
	}

	bool isOrthogonal(const Vector_<ROW, TYPE>& other) const {
	    TYPE cosAngle = this->dot(other);
	    return (fabs(cosAngle) < EPSILON);
	}

	bool resetIfNAN(){	///< sets to  {0} if any component is infinite or NAN
		for(size_t i = 0; i < ROW; ++i){
			if(!isfinite(this->r[i][0])){
				for(size_t i = 0; i < ROW; ++i)
					this->r[i][0] = 0;
				return true;
			}
		}
		return false;
	}

	TYPE dot(const Vector_<ROW, TYPE>& other) const {
		TYPE dot = 0;
		for(size_t i = 0; i < ROW; ++i){
			dot += (this->r[i][0] * other.r[i][0]);
		}
		return dot;
	}

	TYPE getLen() const {
		TYPE len = 0;
		for(size_t i = 0; i < ROW; ++i){
			len += (this->r[i][0] * this->r[i][0]);
		}
		return sqrt(len);
	}

	TYPE distance(const Vector_<ROW, TYPE>& other) const{
		return (vecSub(other)).getLen();
	}

	Vector_<ROW, TYPE> normalize() const {
		Vector_<ROW, TYPE> vec = this->scale(1/getLen());
		return vec;
	}

	bool isNormalized() const {
		TYPE len = this->getLen();
		return (fabs(len) < EPSILON);
	}
};
// Overload operator* in order to work with old matlib classes
template<size_t ROW, typename TYPE>
inline Matrix_<ROW,1,TYPE> operator*(const Matrix_<ROW,3,TYPE> &left, const Vector3D_<TYPE> &right) { return left.mMult((Vector_<3,TYPE>) right); }
template<size_t COL, typename TYPE>
inline Matrix_<3,COL,TYPE> operator*(const Vector3D_<TYPE> &left, const Matrix_<1,COL,TYPE> &right) { return ((Vector_<3,TYPE>)left).mMult(right); }

template<size_t ROW, size_t COL, typename TYPE>
inline Matrix_<ROW,1,TYPE> operator*(const Matrix_<ROW,6> &left, const Vector6D_<TYPE> &right) { return left.mMult((Vector_<6,TYPE>) right); }
template<size_t COL, typename TYPE>
inline Matrix_<6,COL,TYPE> operator*(const Vector6D_<TYPE> &left, const Matrix_<1,COL,TYPE> &right) { return ((Vector_<6,TYPE>)left).mMult(right); }


#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* GENERIC_H_ */
