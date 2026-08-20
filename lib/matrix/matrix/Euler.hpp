/**
 * @file Euler.hpp
 *
 * 所有旋转和坐标系遵循右手定则。
 *
 * 本类的实例定义了从坐标系1到坐标系2的旋转。
 * 采用 3-2-1 本征（intrinsic）Tait-Bryan 旋转序列约定。
 * 为了从帧1到帧2，依次应用如下旋转：
 * 1) 关于初始的 Z 轴旋转角度 _psi。
 * 2) 关于新生成的 Y' 轴旋转角度 _theta。
 * 3) 关于新生成的 X'' 轴旋转角度 _phi。
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Quaternion;

/**
 * Euler angles class
 *
 * This class describes the rotation from frame 1
 * to frame 2 via 3-2-1 intrinsic Tait-Bryan rotation sequence.
 */
template<typename Type>
class Euler : public Vector<Type, 3>
{
public:
	/**
	 * Standard constructor
	 */
	Euler() = default;

	/**
	 * Copy constructor
	 *
	 * @param other vector to copy
	 */
	Euler(const Vector<Type, 3> &other) :
		Vector<Type, 3>(other)
	{
	}

	/**
	 * Constructor from Matrix31
	 *
	 * @param other Matrix31 to copy
	 */
	Euler(const Matrix<Type, 3, 1> &other) :
		Vector<Type, 3>(other)
	{
	}

	/**
	 * Constructor from euler angles
	 *
	 * Instance is initialized from an 3-2-1 intrinsic Tait-Bryan
	 * rotation sequence representing transformation from frame 1
	 * to frame 2.
	 *
	 * @param phi_ rotation angle about X axis
	 * @param theta_ rotation angle about Y axis
	 * @param psi_ rotation angle about Z axis
	 */
	Euler(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
	{
		phi() = phi_;
		theta() = theta_;
		psi() = psi_;
	}

	/**
	 * Constructor from DCM matrix
	 *
	 * Instance is set from Dcm representing transformation from
	 * frame 2 to frame 1.
	 * This instance will hold the angles defining the 3-2-1 intrinsic
	 * Tait-Bryan rotation sequence from frame 1 to frame 2.
	 *
	 * @param dcm Direction cosine matrix
	*/
	Euler(const Dcm<Type> &dcm)
	{
		theta() = asin(-dcm(2, 0));

		if ((fabs(theta() - Type(M_PI / 2))) < Type(1.0e-3)) {
			phi() = 0;
			psi() = atan2(dcm(1, 2), dcm(0, 2));

		} else if ((fabs(theta() + Type(M_PI / 2))) < Type(1.0e-3)) {
			phi() = 0;
			psi() = atan2(-dcm(1, 2), -dcm(0, 2));

		} else {
			phi() = atan2(dcm(2, 1), dcm(2, 2));
			psi() = atan2(dcm(1, 0), dcm(0, 0));
		}
	}

	/**
	 * Constructor from quaternion instance.
	 *
	 * Instance is set from a quaternion representing transformation
	 * from frame 2 to frame 1.
	 * This instance will hold the angles defining the 3-2-1 intrinsic
	 * Tait-Bryan rotation sequence from frame 1 to frame 2.
	 *
	 * @param q quaternion
	*/
	Euler(const Quaternion<Type> &q) : Vector<Type, 3>(Euler(Dcm<Type>(q)))
	{
	}

	inline Type phi() const
	{
		return (*this)(0);
	}
	inline Type theta() const
	{
		return (*this)(1);
	}
	inline Type psi() const
	{
		return (*this)(2);
	}

	inline Type &phi()
	{
		return (*this)(0);
	}
	inline Type &theta()
	{
		return (*this)(1);
	}
	inline Type &psi()
	{
		return (*this)(2);
	}

};

using Eulerf = Euler<float>;
using Eulerd = Euler<double>;

} // namespace matrix
