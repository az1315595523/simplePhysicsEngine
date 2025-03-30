#include "PhysicsObject.h" 
#include "PhysicsEngine.h"

namespace pEngine
{
	Collider::Collider(pObject::RigidBody* rb) : body(rb)
	{
	}

	CircleCollider::CircleCollider(pObject::RigidBody* rb, Vector2 offset, double radius) : Collider(rb),radius(radius)
	{
		this->offset = offset;
	}
	double CircleCollider::CalculateMoment(double mass) const
	{
		return 0.5 * mass * radius * radius;
	}
	CollisionInfo CircleCollider::CheckCollision(const Collider* other) const
	{
		return  other->CheckCollisionWith(*this);
	}
	AABB CircleCollider::GetAABB() const
	{
		Vector2 center = body->getPosition();
		return {
			{center.getVx() - radius, center.getVy() - radius},
			{center.getVx() + radius, center.getVy() + radius}
		};
	}
	double CircleCollider::getRadius() const
	{
		return this->radius;
	}
	CollisionInfo CircleCollider::CheckCollisionWith(const CircleCollider& other) const
	{
		CollisionInfo res;
		Vector2 pos1 = getBody()->getCenterOfMass();
		Vector2 pos2 = other.getBody()->getCenterOfMass();
		Vector2 delta = pos1 - pos2;
		double minDist = radius + other.radius;
		double distance = delta.getMagnitude();
		res.isColliding = distance <= minDist;
		if (res.isColliding)
		{
			res.normal = delta.Normalized();
			res.contactPoint = pos1 + delta * (radius /minDist);
			res.penetration = minDist - distance;
		}

		return res;
	}
	CollisionInfo CircleCollider::CheckCollisionWith(const BoxCollider& other) const
	{
		CollisionInfo info = other.CheckCollisionWith(*this);
		info.normal = -info.normal;
		return info;
	}
	//Vector2 CircleCollider::CalculateCollisionNormal(const Collider* other) const
	//{
	//	if (auto circle = dynamic_cast<const CircleCollider*>(other)) {
	//		Vector2 dir = body->getPosition() - circle->body->getPosition();
	//		return dir.Normalized();
	//	}

	//	if (auto box = dynamic_cast<const BoxCollider*>(other)) {
	//		//Vector2 boxCenter = box->getBody()->getPosition();
	//		//Vector2 circlePos = getBody()->getPosition();

	//		//Vector2 delta = circlePos - boxCenter;
	//		//Vector2 closestPoint = delta.Clamp(- box->GetExtents(), box->GetExtents());
	//		//Vector2 normal = (body->getPosition() - closestPoint).Normalized();
	//		//return normal;
	//		return other->CalculateCollisionNormal(this);
	//	}
	//	return Vector2(0, 1); // 默认法线
	//}
	//
	
	BoxCollider::BoxCollider(pObject::RigidBody* rb, Vector2 offset, double width, double height) :Collider(rb), width(width), height(height)
	{
		this->offset = offset;
	}
	CollisionInfo BoxCollider::CheckCollision(const Collider* other) const
	{
		return other->CheckCollisionWith(*this);
	}
	AABB BoxCollider::GetAABB() const
	{
		OBB obb = GetOBB();
		Vector2 points[4] = {
			obb.center + obb.axes[0] * obb.extents.getVx() + obb.axes[1] * obb.extents.getVy(),
			obb.center - obb.axes[0] * obb.extents.getVx() + obb.axes[1] * obb.extents.getVy(),
			obb.center - obb.axes[0] * obb.extents.getVx() - obb.axes[1] * obb.extents.getVy(),
			obb.center + obb.axes[0] * obb.extents.getVx() - obb.axes[1] * obb.extents.getVy()
		};

		AABB aabb;
		aabb.min = aabb.max = points[0];
		for (int i = 1; i < 4; ++i) {
			aabb.min = Min(aabb.min, points[i]);
			aabb.max = Max(aabb.max, points[i]);
		}

		return aabb;
	}
	OBB BoxCollider::GetOBB() const
	{
		OBB obb;
		obb.center = body->getCenterOfMass();
		obb.extents = GetExtents();
		obb.angle = body->getAngle();

		double cosA = cos(obb.angle);
		double sinA = sin(obb.angle);
		obb.axes[0] = Vector2(cosA, sinA);
		obb.axes[1] = Vector2(-sinA, cosA);

		return obb;
	}
	double BoxCollider::CalculateMoment(double mass) const
	{
		return (mass * (width*width+height*height)) / 12.0;
	}
	CollisionInfo BoxCollider::CheckCollisionWith(const CircleCollider& other) const
	{
		CollisionInfo res;
		OBB obb = GetOBB();

		Vector2 circlePos = other.getBody()->getCenterOfMass();
		double radius = other.getRadius();

		Vector2 localPos = circlePos - body->getCenterOfMass();
		double x = Dot(localPos, obb.axes[0]);
		double y = Dot(localPos, obb.axes[1]);

		Vector2 localClosest = Vector2(x, y).Clamp(-GetExtents(), GetExtents());

		Vector2 worldClosest = body->getCenterOfMass() +
			obb.axes[0] * localClosest.getVx() +
			obb.axes[1] * localClosest.getVy();

		Vector2 difference = circlePos - worldClosest;
		double distance = difference.getMagnitude();

		if (distance <= radius) {
			res.isColliding = true;
			res.penetration = radius - distance;
			res.normal = (distance < FLT_EPSILON) ? obb.axes[0] : difference.Normalized();
			res.contactPoint = worldClosest;
		}

		return res;
	}
	CollisionInfo BoxCollider::CheckCollisionWith(const BoxCollider& other) const
	{
		CollisionInfo info;

		Vector2 axes[4] = {
			GetOBB().axes[0], GetOBB().axes[1], other.GetOBB().axes[0], other.GetOBB().axes[1]
		};

		double minOverlap = INFINITY;
		Vector2 smallestAxis;
		Vector2 pos1 = body->getCenterOfMass();
		Vector2 pos2 = other.body->getCenterOfMass();

		for (auto& axis : axes) {
			if (axis.getMagnitude() * axis.getMagnitude() < FLT_EPSILON) continue;
			axis = axis.Normalized();

			double aProj = GetExtents().getVx() * fabs(Dot(GetOBB().axes[0], axis)) +
				GetExtents().getVy() * fabs(Dot(GetOBB().axes[1], axis));

			double bProj = other.GetExtents().getVx() * fabs(Dot(other.GetExtents(), axis)) +
				other.GetExtents().getVy() * fabs(Dot(other.GetOBB().axes[1], axis));

			double distance = fabs(Dot(pos1 - pos2, axis));
			double overlap = (aProj + bProj) - distance;

			if (overlap <= 0) {
				return info; // 分离轴存在，无碰撞
			}

			if (overlap < minOverlap) {
				minOverlap = overlap;
				smallestAxis = axis;
			}
		}

		info.isColliding = true;
		info.penetration = minOverlap;

		Vector2 dir = pos2 - pos1;
		if (Dot(dir, smallestAxis) < 0) {
			smallestAxis = -smallestAxis;
		}
		info.normal = smallestAxis;

		info.contactPoint = (pos1 + pos2) * 0.5;

		return info;
	}
	Vector2 BoxCollider::GetExtents() const
	{
		return Vector2(width/2.0,height/2.0);
	}
}