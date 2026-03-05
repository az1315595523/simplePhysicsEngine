#include "PhysicsWorld.h"
#include <iostream>

namespace pEngine
{
    void PhysicsWorld::AddObject(pObject::BaseObject* obj)
    {
        obj->SetWorld(this);
        allObjects_.push_back(obj);

        if (obj->GetType() == pObject::ObjectType::RigidBody) {
            pObject::RigidBody* rb = static_cast<pObject::RigidBody*>(obj);
            rigidBodies_.push_back(rb);
            if (rb->IsActive()) {
                collisionTree_.Insert(rb);
            }
        }

        MarkDirty(obj);
    }

    void PhysicsWorld::RemoveObject(pObject::BaseObject* obj)
    {
        if (!obj) return;

        if (obj->GetType() == pObject::ObjectType::RigidBody) {
            collisionTree_.Remove(static_cast<pObject::RigidBody*>(obj));
        }

        switch (obj->GetType()) {
        case pObject::ObjectType::RigidBody: {
            auto rb_it = std::find(rigidBodies_.begin(), rigidBodies_.end(),
                static_cast<pObject::RigidBody*>(obj));
            if (rb_it != rigidBodies_.end()) {
                rigidBodies_.erase(rb_it);
            }
            break;
        }
        default:
            break;
        }

        auto it = std::find_if(allObjects_.begin(), allObjects_.end(),
            [obj](const auto& ptr) { return ptr == obj; });

        if (it != allObjects_.end()) {
            pendingRemoval_.push_back(std::move(*it));
            allObjects_.erase(it);
        }

        dirtyObjects_.erase(obj);
    }

    void PhysicsWorld::Update(double deltaTime)
    {
        ProcessPendingRemovals();

        for (auto& obj : allObjects_) {
            if (obj->IsActive()) {
                obj->Update(deltaTime);
            }
        }

        UpdateDirtyObjects();
        DetectCollisions();
    }

    void PhysicsWorld::DetectCollisions()
    {
        std::unordered_set<std::pair<pObject::RigidBody*, pObject::RigidBody*>,
            RigidBodyPairHash,
            RigidBodyPairEqual> checkedPairs;

        for (size_t i = 0; i < rigidBodies_.size(); ++i) {
            if (!rigidBodies_[i]->IsActive()) continue;

            std::vector<pObject::RigidBody*> potentials;
            collisionTree_.Query(potentials, rigidBodies_[i]->GetCollider().GetAABB());

            for (auto other : potentials) {
                if (!other->IsActive()) continue;
                if (rigidBodies_[i] >= other) continue;

                CollisionInfo info = rigidBodies_[i]->GetCollider().CheckCollision(&other->GetCollider());
                if (!info.isColliding) {
                    continue;
                }

                if (info.shouldTriggerEvent) {
                    rigidBodies_[i]->GetCollider().DispatchCollisionEvent(info, other->GetCollider());
                }

                if (info.shouldResolvePhysics) {
                    ResolveCollision(rigidBodies_[i], other, info);
                }
            }
        }
    }

    void PhysicsWorld::ResolveCollision(pObject::RigidBody* a, pObject::RigidBody* b, const CollisionInfo& info)
    {
        Vector2 normal = info.normal;
        Vector2 contactPoint = info.contactPoint;

        Vector2 r1 = contactPoint - a->getCenterOfMass();
        Vector2 r2 = contactPoint - b->getCenterOfMass();

        Vector2 vel1 = a->getVelocity() + Vector2(-a->getAngularVelocity() * r1.getVy(), a->getAngularVelocity() * r1.getVx());
        Vector2 vel2 = b->getVelocity() + Vector2(-b->getAngularVelocity() * r2.getVy(), b->getAngularVelocity() * r2.getVx());
        Vector2 relativeVel = vel1 - vel2;

        double velAlongNormal = Dot(relativeVel, normal);
        if (velAlongNormal > 0) return;

        double e = std::min(a->getRestitution(), b->getRestitution());
        double numerator = -(1 + e) * velAlongNormal;

        double term1 = 1 / a->getMass();
        double term2 = 1 / b->getMass();
        double term3 = pow(Cross(r1, normal), 2) / a->getMomentOfInertia();
        double term4 = pow(Cross(r2, normal), 2) / b->getMomentOfInertia();

        double j = numerator / (term1 + term2 + term3 + term4);

        Vector2 impulse = normal * j;
        a->ApplyImpulse(impulse, info.contactPoint);
        b->ApplyImpulse(-impulse, info.contactPoint);
    }

    std::vector<pObject::BaseObject*> PhysicsWorld::GetObjects() const
    {
        std::vector<pObject::BaseObject*> result;
        for (auto& obj : allObjects_) {
            result.push_back(obj);
        }
        return result;
    }

    void PhysicsWorld::MarkDirty(pObject::BaseObject* obj)
    {
        dirtyObjects_.insert(obj);
    }

    void PhysicsWorld::UpdateDirtyObjects()
    {
        for (auto obj : dirtyObjects_) {
            if (!obj->IsDirty()) continue;

            if (obj->GetType() == pObject::ObjectType::RigidBody) {
                collisionTree_.Remove(static_cast<pObject::RigidBody*>(obj));
            }

            if (obj->IsActive() && obj->GetType() == pObject::ObjectType::RigidBody) {
                collisionTree_.Insert(static_cast<pObject::RigidBody*>(obj));
            }

            obj->ClearDirty();
        }
        dirtyObjects_.clear();
    }
}
