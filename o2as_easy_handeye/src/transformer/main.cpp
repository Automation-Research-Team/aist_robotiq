#include <iostream>
#include <array>
#include <tf/transform_datatypes.h>

using Matrix4x4 = std::array<std::array<tfScalar, 4>, 4>;
using Vector6	= std::array<tfScalar, 6>;

constexpr Vector6	bTs{0.175, -0.03, 0.035, 0.0, -0.20507619, 0.0};


Matrix4x4
operator *(const Matrix4x4& a, const Matrix4x4& b)
{
    Matrix4x4	c;
    for (int i = 0; i < 4; ++i)
	for (int j = 0; j < 4; ++j)
	    for (int k = 0; k < 4; ++k)
		c[i][j] = a[i][k]*b[k][j];
    return c;
}

template <class T, size_t D> std::ostream&
operator <<(std::ostream& out, const std::array<T, D>& a)
{
    for (const auto& x : a)
	out << x << ' ';
    return out << std::endl;
}
    
std::ostream&
operator <<(std::ostream& out, const Vector6& a)
{
    constexpr tfScalar	degree = 180.0/M_PI;

    return out << "<origin xyz=\""
	       << a[0] << ' '
	       << a[1] << " ${bots_z + "
	       << a[2] << "}\" rpy=\"${"
	       << a[3]*degree << "*pi/180} ${"
	       << a[4]*degree << "*pi/180} ${"
	       << a[5]*degree << "*pi/180}\"/>"
	       << std::endl;
}
    
static tf::Vector3
getRPY(const tf::Quaternion& q)
{
    const tf::Matrix3x3	rot(q);
    tfScalar		roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);

    return {roll, pitch, yaw};
}
    
static Matrix4x4
getMatrix4x4(const tf::Transform& trns)
{
    const tf::Vector3	org(trns.getOrigin());
    const tf::Matrix3x3	rot(trns.getRotation());
    Matrix4x4		mat;

    for (int i = 0; i < 3; ++i)
    {
	for (int j = 0; j < 3; ++j)
	    mat[i][j] = rot[i][j];
	mat[i][3] = org[i];
	mat[3][i] = 0;
    }
    mat[3][3] = 1;

    return mat;
}

static Vector6
getVector6(const tf::Transform& trns)
{
    const tf::Vector3	org(trns.getOrigin());
    const tf::Vector3	rpy(getRPY(trns.getRotation()));

    return {org[0], org[1], org[2], rpy[0], rpy[1], rpy[2]};
}

static tf::Transform
getTransform(const Vector6& xyz_rpy)
{
    tf::Quaternion	q = tf::createQuaternionFromRPY(xyz_rpy[3],
							xyz_rpy[4],
							xyz_rpy[5]);
    tf::Vector3		t(xyz_rpy[0], xyz_rpy[1], xyz_rpy[2]);

    return tf::Transform(q, t);
}

void
convert(const tf::Transform& trns)
{
    std::cout << getMatrix4x4(trns);
    std::cout << getVector6(trns) << std::endl;
}


    
int
main()
{
    constexpr tfScalar	degree = 180.0/M_PI;
    tfScalar		qx, qy, qz, qw, x, y, z;
    
    std::cerr << "> ";
    std::cin >> x >> y >> z >> qx >> qy >> qz >> qw;

    tf::Vector3		org(x, y, z);
    tf::Quaternion	rot(qx, qy, qz, qw);
    tf::Transform	trns;
    trns.setOrigin(org);
    trns.setRotation(rot);

  // Convert to 4x4 matrix
    convert(trns);

    std::cerr << "===================" << std::endl;


    convert((getTransform(bTs)*trns).inverse());


    return 0;
}
