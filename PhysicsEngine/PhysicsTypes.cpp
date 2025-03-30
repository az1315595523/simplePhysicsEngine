#include "PhysicsTypes.h"
#include "PhysicsEngine.h"
namespace pEngine
{
	Vector2::Vector2(double vx, double vy, std::string desc) :vx(vx), vy(vy), desc(desc)
	{
		double cos_theta = vx / getMagnitude();
		this->dir = std::fmod(std::atan2(vy, vx) + 2 * _pi, 2 * _pi);
	}
	double Vector2::getMagnitude() const
	{
		double magnitude = std::sqrt(vx * vx + vy * vy);
		return magnitude;
	}
	double Vector2::getDir() const
	{
		return dir;
	}
	double Vector2::getVx() const
	{
		return vx;
	}
	double Vector2::getVy() const
	{
		return vy;
	}
	std::string Vector2::getDesc() const
	{
		return this->desc;
	}
	void Vector2::setVx(double vx)
	{
		this->vx = vx;
	}
	void Vector2::setVy(double vy)
	{
		this->vy = vy;
	}
	void Vector2::setDesc(std::string desc)
	{
		this->desc = desc;
	}
	double Vector2::getProjection(double projectDir) const
	{
		if (projectDir == 4)
			projectDir = this->dir;
		double angleDiff_rad = (this->dir - projectDir);
		double projection = getMagnitude() * std::cos(angleDiff_rad);
		if (std::abs(projection) < 1e-5)
			return 0.0;
		return projection;
	}

	Vector2 operator+(const Vector2& v1, const Vector2& v2)
	{
		double v1x = v1.getVx();
		double v2x = v2.getVx();
		double v1y = v1.getVy();
		double v2y = v2.getVy();

		Vector2 v(v1x + v2x, v1y + v2y);
		return v;
	}
	Vector2 operator-(const Vector2& v1, const Vector2& v2)
	{
		double v1x = v1.getVx();
		double v2x = v2.getVx();
		double v1y = v1.getVy();
		double v2y = v2.getVy();

		Vector2 v(v1x - v2x, v1y - v2y);
		return v;
	}

	Vector2 operator-(const Vector2& v)
	{
		return Vector2(-v.getVx(), -v.getVy());
	}

	Vector2 operator*(const Vector2& v, const double& num)
	{
		return Vector2(v.getVx()*num,v.getVy()*num);
	}
	Vector2 operator/(const Vector2& v, const double& num)
	{
		return Vector2(v.getVx() / num, v.getVy() / num);
	}
	double Distance(const Vector2& p1, const Vector2& p2)
	{
		double x1 = p1.getVx();
		double x2 = p2.getVx();
		double y1 = p1.getVy();
		double y2 = p2.getVy();

		double ans = pow(x1 - x2, 2) + pow(y1 - y2, 2);

		return std::sqrt(ans);
	}

	double Dot(const Vector2& p1, const Vector2& p2)
	{
		double x1 = p1.getVx();
		double x2 = p2.getVx();
		double y1 = p1.getVy();
		double y2 = p2.getVy();

		return x1 * x2 + y1 * y2;
	}

	double Cross(const Vector2& p1, const Vector2& p2)
	{
		double x1 = p1.getVx();
		double x2 = p2.getVx();
		double y1 = p1.getVy();
		double y2 = p2.getVy();
		return x1 * y2 - y1 * x2;
	}

	Vector2 Min(const Vector2& p1, const Vector2& p2)
	{
		double x1 = p1.getVx();
		double x2 = p2.getVx();
		double y1 = p1.getVy();
		double y2 = p2.getVy();
		return Vector2(std::min(x1,x2),std::min(y1,y2));
	}

	Vector2 Max(const Vector2& p1, const Vector2& p2)
	{
		double x1 = p1.getVx();
		double x2 = p2.getVx();
		double y1 = p1.getVy();
		double y2 = p2.getVy();
		return Vector2(std::max(x1,x2),std::max(y1,y2));
	}
	
	Vector2 Vector2::Clamp(const Vector2& min, const Vector2& max) const
	{
		return Vector2(
			std::max(min.getVx(), std::min(getVx(), max.getVx())),
			std::max(min.getVy(), std::min(getVy(), max.getVy()))
		);
	}
	Vector2 Vector2::Normalized()
	{
		double len = getMagnitude();
		return Vector2(getVx() / len, getVy() / len);
	}
}