#pragma once
#include "PhysicsFwd.h"
#include <string>
#include <cmath>
namespace pEngine
{
	class Vector2
	{
	public:
		Vector2(double vx = 0.0, double vy = 0.0, std::string desc = "");
		double getMagnitude() const;
		double getDir() const;
		double getVx() const; 
		double getVy() const;
		std::string getDesc() const;
		void setVx(double vx);
		void setVy(double vy);
		void setDesc(std::string desc);
		double getProjection(double projectDir = 4) const;
		Vector2 Clamp(const Vector2& min, const Vector2& max) const;
		Vector2 Normalized();
	private:
		double vx;
		double vy;
		//dir means the radAngle,which is between 0 and 2¦Ð
		double dir;
		std::string desc;
	};
	Vector2 operator+(const Vector2& v1, const Vector2& v2);
	Vector2 operator-(const Vector2& v1, const Vector2& v2);
	Vector2 operator-(const Vector2& v);
	Vector2 operator*(const Vector2& v, const double& num);
	Vector2 operator/(const Vector2& v, const double& num);
	double Distance(const Vector2& p1, const Vector2& p2);
	double Dot(const Vector2& p1, const Vector2& p2);
	double Cross(const Vector2& p1, const Vector2& p2);
	Vector2 Min(const Vector2& p1, const Vector2& p2);
	Vector2 Max(const Vector2& p1, const Vector2& p2);
}