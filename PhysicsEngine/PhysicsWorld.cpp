#include "PhysicsWorld.h"
#include <iostream>

namespace pEngine
{
    void PhysicsWorld::AddObject(std::unique_ptr<pObject::BaseObject> obj)
    {
        pObject::BaseObject* rawPtr = obj.get();
        allObjects_.push_back(std::move(obj));

        // 分类存储
        if (rawPtr->GetType() == pObject::ObjectType::RigidBody) {
            pObject::RigidBody* rb = static_cast<pObject::RigidBody*>(rawPtr);
            rigidBodies_.push_back(rb);
            collisionTree_.Insert(rb);
        }

        MarkDirty(rawPtr);
    }

    void PhysicsWorld::RemoveObject(pObject::BaseObject* obj) {
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

        // 3. 从主容器移除所有权
        auto it = std::find_if(allObjects_.begin(), allObjects_.end(),
            [obj](const auto& ptr) { return ptr.get() == obj; });

        if (it != allObjects_.end()) {
            pendingRemoval_.push_back(std::move(*it));
            allObjects_.erase(it);
        }

        // 4. 从脏对象集合移除（如果存在）
        dirtyObjects_.erase(obj);
    }

    void PhysicsWorld::Update(double deltaTime)
    {
        ProcessPendingRemovals();
        for (auto& obj : allObjects_) {
            obj->Update(deltaTime);
        }
        // 处理脏对象
        UpdateDirtyObjects();
        DetectCollisions();
    }

    void PhysicsWorld::DetectCollisions()
    {
        std::unordered_set<std::pair<pObject::RigidBody*, pObject::RigidBody*>, 
            RigidBodyPairHash,
            RigidBodyPairEqual> checkedPairs;

        for (size_t i = 0; i < rigidBodies_.size(); ++i) {
            std::vector<pObject::RigidBody*> potentials;
            collisionTree_.Query(potentials, rigidBodies_[i]->GetCollider().GetAABB());

            for (auto other : potentials) {
                if (rigidBodies_[i] >= other) continue;

                CollisionInfo info = rigidBodies_[i]->GetCollider().CheckCollision(&other->GetCollider());
                if (info.isColliding) {
                    std::cout << "collding!" << std::endl;
                    ResolveCollision(rigidBodies_[i], other, info);
                }
            }
        }
    }
    void PhysicsWorld::ResolveCollision(pObject::RigidBody* a,pObject::RigidBody* b, const CollisionInfo& info)
    {
        //
        Vector2 normal = info.normal;
        Vector2 contactPoint = info.contactPoint;

        // relative Velocity
        Vector2 r1 = contactPoint - a->getCenterOfMass();
        Vector2 r2 = contactPoint - b->getCenterOfMass();

        Vector2 vel1 = a->getVelocity() + Vector2(-a->getAngularVelocity() * r1.getVy(), a->getAngularVelocity() * r1.getVx());
        Vector2 vel2 = b->getVelocity() + Vector2(-b->getAngularVelocity() * r2.getVy(), b->getAngularVelocity() * r2.getVx());
        Vector2 relativeVel = vel1 - vel2;

        // normal-dir Magnitude
        double velAlongNormal = Dot(relativeVel, normal);
        if (velAlongNormal > 0) return; 

        // calculate the coefficent of Impulse
        double e = std::min(a->getRestitution(), b->getRestitution()); // e is between 0-1 and 1 means elastic collision , 0 means perfectly inelastic collision
        double numerator = -(1 + e) * velAlongNormal;

        double term1 = 1 / a->getMass();
        double term2 = 1 / b->getMass();
        double term3 = pow(Cross(r1, normal), 2) / a->getMomentOfInertia();
        double term4 = pow(Cross(r2, normal), 2) / b->getMomentOfInertia();

        double j = numerator / (term1 + term2 + term3 + term4);

        // ApplyImpulse
        Vector2 impulse = normal * j;
        a->ApplyImpulse(impulse, info.contactPoint);
        b->ApplyImpulse(-impulse, info.contactPoint);
    }

    std::vector<pObject::BaseObject*> PhysicsWorld::GetObjects() const {
        std::vector<pObject::BaseObject*> result;
        for (auto& obj : allObjects_) {
            result.push_back(obj.get());
        }
        return result;
    }
    void PhysicsWorld::MarkDirty(pObject::BaseObject* obj) {
        dirtyObjects_.insert(obj);
    }
    void PhysicsWorld::UpdateDirtyObjects() {
        for (auto obj : dirtyObjects_) {
            if (!obj->IsDirty()) continue;

            // 从四叉树移除（如果是刚体）
            if (obj->GetType() == pObject::ObjectType::RigidBody) {
                collisionTree_.Remove(static_cast<pObject::RigidBody*>(obj));
            }

            // 重新插入（如果是刚体且active）
            if (obj->GetType() == pObject::ObjectType::RigidBody) {
                collisionTree_.Insert(static_cast<pObject::RigidBody*>(obj));
            }

            obj->ClearDirty();
        }
        dirtyObjects_.clear();
    }
}