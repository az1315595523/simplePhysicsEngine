#include "PhysicsObject.h"

namespace pObject
{
	BaseObject::BaseObject() :mass(0), isGravity(false), position(0, 0), velocity(0, 0)
	{
	}
	BaseObject::BaseObject(double mass, Vector2 velocity, Vector2 position)
		:mass(mass), position(position), velocity(velocity)
	{
	}
	void BaseObject::Update(double delta_t)
	{
		UpdateVelocity(delta_t);
		UpdatePoisition(delta_t);
		UpdateForces(delta_t);
	}
	void BaseObject::UpdatePoisition(double delta_t)
	{
		double px = position.getVx();
		double py = position.getVy();

		double v0x = velocity.getVx();
		double v0y = velocity.getVy();

		double deltaX = v0x * delta_t;
		double deltaY = v0y * delta_t;

		setPosition(px + deltaX, py + deltaY);

	}
	void BaseObject::UpdateVelocity(double delta_t)
	{
		auto it = forces.begin();
		while (it!=forces.end())
		{
			Force* force = it->get();
			double remainingTime = force->getDurationTime();
			
			const bool isInfinite = remainingTime == INFINITY;
			double fx = force->getForceVector().getVx();
			double fy = force->getForceVector().getVy();

			double actualDeltaT = isInfinite ? delta_t : std::min(remainingTime, delta_t);

			Vector2 acceleration = Vector2(fx/mass, fy/mass);
			Vector2 deltaV = acceleration * delta_t;
			setVelocity(velocity + deltaV);
			it++;
		}
	}
	void BaseObject::UpdateForces(double delta_t)
	{
		auto it = forces.begin();
		while (it != forces.end())
		{
			Force* force = it->get();
			double remainingTime = force->getDurationTime();
			const bool isInfinite = (remainingTime == _IFINITY);
			if (!isInfinite)
			{
				force->setDurationTime(remainingTime - delta_t);
				if (remainingTime <= 0)
				{
					it = forces.erase(it); continue;
				}
			}
			it++;
		}
	}
	Vector2 BaseObject::getPosition()
	{
		return this->position;
	}
	Vector2 BaseObject::getVelocity()
	{
		return this->velocity;
	}
	std::vector<std::unique_ptr<Force>>& BaseObject::getForces()
	{
		return this->forces;
	}
	void BaseObject::setVelocity(double vx, double vy)
	{
		this->velocity.setVx(vx);
		this->velocity.setVy(vy);
		MarkDirty();
	}
	Force* BaseObject::setGravity()
	{
		return addForce(0, -mass * _gravity, _IFINITY);
	}
	Force* BaseObject::addForce(double ax, double ay,double time)
	{
		Vector2 fV = Vector2(ax, ay);
		this->forces.push_back(std::make_unique<Force>(fV, time,this->position));
		return forces.back().get();
	}
	void BaseObject::setPosition(double x, double y)
	{
		this->position.setVx(x);
		this->position.setVy(y);
		MarkDirty();
	}
	void BaseObject::RemoveForce(std::unique_ptr<Force>& force)
	{
		auto it = std::find(forces.begin(), forces.end(), force);
		if (it != forces.end()) {
			forces.erase(it);
		}
	}
	void BaseObject::RemoveAllForces()
	{
		forces.clear();
	}
	RigidBody::RigidBody(double mass , Vector2 velocity, Vector2 position, double angularVelocity) 
		: BaseObject(mass, velocity, position),
		angularVelocity(angularVelocity),
		angle(0.0),
		restitution(0.8)
	{
		momentOfInertia = 0.0;
	}
	void RigidBody::AddCircleCollider(double offset, double radius)
	{
		collider = std::make_unique<CircleCollider>(this,offset,radius);
		momentOfInertia = this->collider->CalculateMoment(mass);
	}
	void RigidBody::AddBoxCollider(double offset, double width, double height)
	{
		collider = std::make_unique<BoxCollider>(this, offset, width, height);
		momentOfInertia = this->collider->CalculateMoment(mass);
	}
	void RigidBody::Update(double delta_t)
	{
		UpdateAngularVelocity(delta_t);
		UpdateAngle(delta_t);
		BaseObject::Update(delta_t);
	}
	void RigidBody::UpdateAngularVelocity(double delta_t)
	{

		auto it = forces.begin();
		while (it != forces.end())
		{
			Force* force = it->get();
			double remainingTime = force->getDurationTime();
			//the torque ¦Ó = r ¡Á F
			Vector2 r = force->getApplyPoint();
			double torque = Cross(r, force->getForceVector());
			//the angularAcceleration ¦Á = ¦Ó / I
			double angularAcceleration = torque / momentOfInertia;
			const bool isInfinite = remainingTime == INFINITY;
			double actualDeltaT = isInfinite ? delta_t : std::min(remainingTime, delta_t);

			double deltaV= actualDeltaT * angularAcceleration;
			setAngularVelocity(this->angularVelocity + deltaV);
			it++;
		}
	}

	void RigidBody::UpdateAngle(double delta_t)
	{
		double deltaAngle = angularVelocity * delta_t;
		setAngle(angle + deltaAngle);
	}
	
	Force* RigidBody::addForce(double ax, double ay, double time)
	{
		Vector2 fV = Vector2(ax, ay);
		this->forces.push_back(std::make_unique<Force>(fV, time, Vector2()));
		return forces.back().get();
	}
	Force* RigidBody::addForce(double ax, double ay, double time, Vector2 applyPoint) 
	{
		Vector2 fV = Vector2(ax, ay);
		this->forces.push_back(std::make_unique<Force>(fV, time, applyPoint));
		return forces.back().get();
	}
	void RigidBody::ApplyImpulse(const Vector2& impulse, const Vector2& contactPoint)
	{
		setVelocity(velocity + impulse / mass);
		Vector2 r = contactPoint - getCenterOfMass();
		setAngularVelocity(angularVelocity + Cross(r, impulse) / momentOfInertia);
	}
}