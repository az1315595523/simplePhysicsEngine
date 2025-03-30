#pragma once
#include "PhysicsEngine.h"
#include <vector>
#include <utility>
#include <memory>
namespace pObject
{
#define CACHE_THRESHOLD 1.0
	using namespace pEngine;
	enum class ObjectType
	{
		Base,
		RigidBody
	};
	class BaseObject
	{
	public:

		BaseObject();
		BaseObject(double mass, Vector2 velocity = Vector2(), Vector2 position = Vector2());
		virtual ~BaseObject() = default;
		virtual ObjectType GetType() { return ObjectType::Base; }
		virtual void Update(double delta_t);
		virtual void UpdatePoisition(double delta_t);
		virtual void UpdateVelocity(double delta_t);
		virtual void UpdateForces(double delta_t);
		//getter-you can not fix the value by these func
		Vector2 getPosition();
		Vector2 getVelocity();
		double getMass() { return mass; }
		std::vector<std::unique_ptr<Force>>& getForces();
		//setter
		void setVelocity(double vx, double vy);
		void setVelocity(const Vector2& velocity) { this->velocity = velocity;MarkDirty(); }
		void setMass(double mass) { this->mass = mass; }
		Force* setGravity();
		virtual Force* addForce(double ax, double ay,double time);
		void setPosition(double x, double y);
		void setPosition(const Vector2& position) { this->position = position;	MarkDirty();}
		void RemoveForce(std::unique_ptr<Force>& force);
		void RemoveAllForces();
		void MarkDirty() { isDirty_ = true; }
		void ClearDirty() { isDirty_ = false; }
		bool IsDirty() { return isDirty_; }
	protected:
		double mass;
		bool isGravity;
		Vector2 position;
		Vector2 velocity;
		//the second para means the duration time
		std::vector<std::unique_ptr<Force>> forces;
		bool isDirty_ = false;
	};
	class RigidBody : public BaseObject
	{
	public:
		RigidBody() : BaseObject() { angle = 0.0; angularVelocity = 0.0; restitution = 0.0; momentOfInertia = 0.0; collider = NULL; }
		~RigidBody() = default;
		RigidBody(double mass = 0.0, Vector2 velocity = Vector2(), Vector2 position = Vector2(),
			double angularVelocity = 0.0);
		void AddCircleCollider(double offset=0.0,double radius=0.0);
		void AddBoxCollider(double offset=0.0, double width=0.0, double height=0.0);
		ObjectType GetType() override { return ObjectType::RigidBody; }
		void Update(double delta_t) override;
		void UpdateAngularVelocity(double delta_t);
		void UpdateAngle(double delta_t);
		//void ResolveCollision(RigidBody* other,const CollisionInfo info);
		Collider& GetCollider() { return *collider; }
		bool IsTransformDirty() const { return isDirty_; }
		Force* addForce(double ax, double ay, double time) override;
		Force* addForce(double ax, double ay, double time, Vector2 applyPoint);
		void ApplyImpulse(const Vector2& impulse, const Vector2& contactPoint);

		Vector2 getCenterOfMass() { return collider ?(position + collider->getOffset()): position; }
		double getAngle() { return angle; }
		double getAngularVelocity() { return angularVelocity; }
		double getRestitution() { return restitution; }
		double getMomentOfInertia() { return momentOfInertia; }
		void setAngle(double angle) { this->angle = angle ; angle = fmod(angle, 2 * _pi); MarkDirty(); }
		void setAngularVelocity(double angularVelocity) { this->angularVelocity = angularVelocity; MarkDirty(); }
		void setRestitution(double restitution) { this->restitution = restitution; }

	private:
		double angle;// --2D-base only z-axis rotate 
		double angularVelocity; // --2d-base only z-axis velocity 
		double restitution;    
		double momentOfInertia;
		std::unique_ptr<Collider> collider;
	};
}