#pragma once
#include <cmath>
#include <string>
#include "PhysicsFwd.h"
#include "PhysicsTypes.h"

namespace pEngine
{
//t-s   v-m/s    a-m/s^2   m-kg  F- kg*m/s^2
#define _gravity 9.80
#define _pi 3.14159265358979323846
#define _IFINITY -1.0	


	double calSByVT(const Vector2& velocity, double t);
	double calXByVT(const Vector2& velocity, double t, double dir);

	//Axis-alignied bounding box
	struct  AABB
	{
		Vector2 min;
		Vector2 max;
		bool Overlaps(const AABB& other) const;
	};
	//Oriented bounding box
	struct OBB {
		Vector2 center;
		Vector2 extents;    // 半宽和半高
		Vector2 axes[2];    // 局部x轴和y轴(归一化)
		double angle;       // 旋转角度

		bool Overlaps(const OBB& other) const;
	};
	struct CollisionInfo {
		bool isColliding = false;
		Vector2 normal;          // 碰撞法线
		Vector2 contactPoint;    // 接触点
		double penetration;      // 穿透深度
	};

	class Force 
	{
	public:
		Force(const Vector2& forceVector= Vector2(), double time = 0.0,Vector2 applyPoint = Vector2());
		Force(double fx = 0.0, double fy = 0.0, double time = 0.0,Vector2 applyPoint = Vector2());

		Vector2 getForceVector() const { return forceVector; }
		Vector2 getApplyPoint() const { return applyPoint; }
		double getDurationTime() const { return durationTime; }

		void setForceVector(Vector2 forceVector) { this->forceVector = forceVector; }
		void setApplyPoint(Vector2 applyPoint) { this->applyPoint = applyPoint; }
		void setDurationTime(double durationTime) { this->durationTime = durationTime; }

	private:
		Vector2 forceVector;
		double durationTime;
		Vector2 applyPoint; // offsetValue,base on the RigidBody's centerOfMass
	};
	class Collider
	{
	public:
		Collider(pObject::RigidBody* rb);
		virtual ~Collider() = default;

		virtual CollisionInfo CheckCollision(const Collider* collider) const = 0;
		virtual CollisionInfo CheckCollisionWith(const CircleCollider& other) const = 0;
		virtual CollisionInfo CheckCollisionWith(const BoxCollider& other) const = 0;
		//virtual Vector2 CalculateCollisionNormal(const Collider* other) const = 0;

		virtual AABB GetAABB() const = 0;
		virtual double CalculateMoment(double mass) const = 0;

		pObject::RigidBody* getBody() const { return body; }
		Vector2 getOffset() { return offset; }
	protected:
		pObject::RigidBody* body;
		Vector2 offset;
	};

	class CircleCollider : public Collider
	{
	public:
		CircleCollider(pObject::RigidBody* rb,Vector2 offset,double radius);
		CollisionInfo CheckCollision(const Collider* collider) const override;
		double CalculateMoment(double mass) const override;
		AABB GetAABB() const override;
		double getRadius() const;
		CollisionInfo CheckCollisionWith(const CircleCollider& other) const;
		CollisionInfo CheckCollisionWith(const BoxCollider& other) const;
		//Vector2 CalculateCollisionNormal(const Collider* other) const;

	private:
		double radius;

	};
	class BoxCollider : public Collider {
	public:
		BoxCollider(pObject::RigidBody* rb, Vector2 offset, double width, double height);
		CollisionInfo CheckCollision(const Collider* other) const override;
		AABB GetAABB() const override;
		OBB GetOBB() const;
		double CalculateMoment(double mass) const override;
		CollisionInfo CheckCollisionWith(const CircleCollider&) const;
		CollisionInfo CheckCollisionWith(const BoxCollider&) const;

		Vector2 GetExtents() const;

	private:
		double width;
		double height;
	};
}