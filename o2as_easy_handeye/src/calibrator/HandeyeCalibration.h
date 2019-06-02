/*!
  \file		HandeyeCalibration.h
  \brief	Algorithmts for handeye calibration.

  \author	Toshio UESHIBA(AIST)
*/
#ifndef HANDEYECALIBRATION_H
#define HANDEYECALIBRATION_H

#include "Transform.h"

namespace TU
{
namespace detail
{
/************************************************************************
*  static functions							*
************************************************************************/
#if 1
template <class T> static Matrix<T, 4, 4>
qdiff_matrix(const Quaternion<T>& a, const Quaternion<T>& b)
{
    Matrix<T, 4, 4>	C;
    C[0][0]	      = a.scalar() - b.scalar();
    slice(C[0], 1, 3) = b.vector() - a.vector();
    C[1][0]	      = -C[0][1];
    C[2][0]	      = -C[0][2];
    C[3][0]	      = -C[0][3];
    C(1, 3, 1, 3)     = skew(a.vector() + b.vector());
    C[1][1]	     += C[0][0];
    C[2][2]	     += C[0][0];
    C[3][3]	     += C[0][0];

    return C;
}
#else
template <class T> static Matrix<T, 3, 4>
qdiff_matrix(const Quaternion<T>& a, const Quaternion<T>& b)
{
    Matrix<T, 3, 4>	C;
    const auto	sdiff = a.scalar() - b.scalar();
    const auto	vdiff = a.vector() - b.vector();
    C[0][0]	     = vdiff[0];
    C[1][0]	     = vdiff[1];
    C[2][0]	     = vdiff[2];
    C(0, 3, 1, 3)    = skew(a.vector() + b.vector());
    C[0][1]	    += sdiff;
    C[1][2]	    += sdiff;
    C[2][3]	    += sdiff;

    return C;
}
#endif

template <class T> static DualNumber<Quaternion<T> >
computeDualQuaternion(const range<const T*>& up, const range<const T*>& ud,
		      const range<const T*>& vp, const range<const T*>& vd)
{
    auto	a = up * ud;
    auto	b = up * vd + vp * ud;
    auto	c = vp * vd;

    if (std::abs(a) > std::abs(c))
    {
	c /= a;
	b /= 2*a;

	const auto	x0 = (b > 0 ? -b - std::sqrt(b*b - c)
				    : -b + std::sqrt(b*b - c));
	const auto	x1 = c/x0;
	const auto	l0 = length(x0*up + vp);
	const auto	l1 = length(x1*up + vp);

	if (l0 > l1)
	    return {(x0/l0)*up + (1/l0)*vp, (x0/l0)*ud + (1/l0)*vd};
	else
	    return {(x1/l1)*up + (1/l1)*vp, (x1/l1)*ud + (1/l1)*vd};
    }
    else
    {
	a /= c;
	b /= 2*c;

	const auto	y0 = (b > 0 ? -b - std::sqrt(b*b - a)
				    : -b + std::sqrt(b*b - a));
	const auto	y1 = a/y0;
	const auto	l0 = length(up + y0*vp);
	const auto	l1 = length(up + y1*vp);

	if (l0 > l1)
	    return {(1/l0)*up + (y0/l0)*vp, (1/l0)*ud + (y0/l0)*vd};
	else
	    return {(1/l1)*up + (y1/l1)*vp, (1/l1)*ud + (y1/l1)*vd};
    }
}

}	// namespace detail
/************************************************************************
*  global functions							*
************************************************************************/
template <class T> Transform<T>
cameraToEffectorSingle(const std::vector<Transform<T> >& cMo,
		       const std::vector<Transform<T> >& wMe)
{
    if (cMo.size() != wMe.size())
	throw std::runtime_error("transformations with different sizes("
				 + std::to_string(cMo.size()) + "!="
				 + std::to_string(wMe.size()) + ").");
    else if (cMo.size() < 2)
	throw std::runtime_error("too few transformations("
				 + std::to_string(cMo.size()) + ").");

    const auto	nposes = cMo.size();

  // Compute rotation.
    Matrix<T, 4, 4>	M;	// 4x4 matrix initialized with zeros.
    for (size_t i = 0; i < nposes; ++i)
	for (size_t j = i + 1; j < nposes; ++j)
	{
	    const auto	A = wMe[j].inverse() * wMe[i];
	    const auto	B = cMo[j] * cMo[i].inverse();
	    const auto	C = detail::qdiff_matrix(A.primary(), B.primary());
	    M += transpose(C) * C;
	}
    Vector<T, 4>	evalues;
    Quaternion<T>	q(eigen(M, evalues)[3]);	// computed rotation
#ifdef DEBUG
    std::cerr << "--- Eigenvalues ---\n" << evalues << std::endl;
#endif

  // Compute translation.
    Matrix<T, 3, 3>	N;
    Vector<T, 3>	t;
    for (size_t i = 0; i < nposes; ++i)
	for (size_t j = i + 1; j < nposes; ++j)
	{
	    const auto	A  = wMe[j].inverse() * wMe[i];
	    const auto	B  = cMo[j] * cMo[i].inverse();
	    auto	Rt = A.Rt();
	    Rt[0][0] -= 1;
	    Rt[1][1] -= 1;
	    Rt[2][2] -= 1;
	    N += Rt*transpose(Rt);
	    t += Rt*(q.R()*B.t() - A.t());
	}
    solve(N, t);

    return {t, q};
}

template <class T> Transform<T>
cameraToEffectorDual(const std::vector<Transform<T> >& cMo,
		     const std::vector<Transform<T> >& wMe)
{
    if (cMo.size() != wMe.size())
	throw std::runtime_error("transformations with different sizes("
				 + std::to_string(cMo.size()) + "!="
				 + std::to_string(wMe.size()) + ").");
    else if (cMo.size() < 2)
	throw std::runtime_error("too few transformations("
				 + std::to_string(cMo.size()) + ").");

    const auto	nposes = cMo.size();

  // Compute rotation.
    Matrix<T, 8, 8>	M;	// 4x4 matrix initialized with zeros.
    for (size_t i = 0; i < nposes; ++i)
	for (size_t j = i + 1; j < nposes; ++j)
	{
	    const auto	A = wMe[j].inverse() * wMe[i];
	    const auto	B = cMo[j] * cMo[i].inverse();
	    const auto	C = detail::qdiff_matrix(A.primary(), B.primary());
	    const auto	D = detail::qdiff_matrix(A.dual(),    B.dual());
	    M(0, 4, 0, 4) += transpose(C) * C + transpose(D) * D;
	    M(4, 4, 0, 4) += transpose(C) * D;
	    M(4, 4, 4, 4) += transpose(C) * C;
	}
    symmetrize(M);

    Vector<T, 8>	evalues;
    const auto		U = eigen(M, evalues);	// computed rotation
#ifdef DEBUG
    std::cerr << "--- Eigenvalues ---\n" << evalues << std::endl;
#endif
    const Transform<T>	eMc(detail::computeDualQuaternion(slice(U[6], 0, 4),
							  slice(U[6], 4, 4),
							  slice(U[7], 0, 4),
							  slice(U[7], 4, 4)));
    return eMc;
}

template <class T> Transform<T>
objectToWorld(const std::vector<Transform<T> >& cMo,
	      const std::vector<Transform<T> >& wMe, const Transform<T>& eMc)
{
    using vector4_type	= typename Quaternion<T>::vector4_type;

    const auto		nposes = cMo.size();
    vector4_type	p, d;
    for (size_t i = 0; i < nposes; ++i)
    {
      // Transformation from object to world computed from estimated eMc.
	const auto	M = wMe[i] * eMc * cMo[i];
	p += vector4_type(M.primary());
	d += vector4_type(M.dual());
    }
    normalize(p);
    d /= nposes;
    d -= p*(p*d);

    return DualNumber<Quaternion<T> >(p, d);
}

template <class T> void
evaluateAccuracy(std::ostream& out,
		 const std::vector<Transform<T> >& cMo,
		 const std::vector<Transform<T> >& wMe,
		 const Transform<T>& eMc, const Transform<T>& wMo)
{
    const auto	nposes = cMo.size();
    T		tdiff_mean = 0;	// mean translational difference
    T		tdiff_max  = 0;	// max. translational difference
    T		adiff_mean = 0;	// mean angular difference
    T		adiff_max  = 0;	// max. angular difference

    for (size_t i = 0; i < nposes; ++i)
    {
	const auto	AX    = wMe[i] * eMc;
	const auto	YBinv = wMo * cMo[i].inverse();
	const auto	tdiff = AX.translational_difference(YBinv);
	const auto	adiff = AX.angular_difference(YBinv);

	if (tdiff > tdiff_max)
	    tdiff_max = tdiff;
	if (adiff > adiff_max)
	    adiff_max = adiff;

	tdiff_mean += tdiff*tdiff;
	adiff_mean += adiff*adiff;
    }
    tdiff_mean = std::sqrt(tdiff_mean/nposes);
    adiff_mean = std::sqrt(adiff_mean/nposes);

    constexpr T	degree = 180.0/M_PI;
    out << "trans. err(m)  : (mean, max) = ("
	<< tdiff_mean << ", " << tdiff_max
	<< ")\nangle err(deg.): (mean, max) = ("
	<< adiff_mean * degree << ", " << adiff_max * degree << ')'
	<< std::endl;
    std::cout << "=== estimated eMc: ====\n";
    eMc.print(std::cout);
    std::cout << "=== estimated wMo: ===\n";
    wMo.print(std::cout);
}
}	// namespace TU
#endif	// !HANDEYECALIBRATION_H
