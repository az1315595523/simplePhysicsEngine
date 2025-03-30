#pragma once
#include <vector>
#include "PhysicsObject.h"
#include "PhysicsEngine.h"
#include "QuadTree.h"
#include <memory>
#include <unordered_set>
namespace pEngine
{

    struct RigidBodyPairHash {
        size_t operator()(const std::pair<pObject::RigidBody*, pObject::RigidBody*>& p) const {
            // 确保有序化哈希，无论指针顺序如何
            auto a = p.first;
            auto b = p.second;
            if (a > b) std::swap(a, b);
            return std::hash<void*>()(a) ^ std::hash<void*>()(b);
        }
    };

    struct RigidBodyPairEqual {
        bool operator()(const std::pair<pObject::RigidBody*, pObject::RigidBody*>& lhs,
            const std::pair<pObject::RigidBody*, pObject::RigidBody*>& rhs) const {
            // 无序比较
            return (lhs.first == rhs.first && lhs.second == rhs.second) ||
                (lhs.first == rhs.second && lhs.second == rhs.first);
        }
    };

    class PhysicsWorld {
    public:
        
        void AddObject(std::unique_ptr<pObject::BaseObject> obj);
        void RemoveObject(pObject::BaseObject* obj);
        void MarkDirty(pObject::BaseObject* obj);
        
        void Update(double deltaTime);
        void ResolveCollision(pObject::RigidBody* a, pObject::RigidBody* b, const CollisionInfo& info);
        void UpdateDirtyObjects();
        std::vector<pObject::BaseObject*> GetObjects() const;

    private:
        void DetectCollisions();
        std::vector<std::unique_ptr<pObject::BaseObject>> pendingRemoval_;
        void ProcessPendingRemovals() {
            if (!pendingRemoval_.empty()) {
                pendingRemoval_.clear();
            }
        }
        std::vector<std::unique_ptr<pObject::BaseObject>> allObjects_;
        std::vector<pObject::RigidBody*> rigidBodies_;
        std::unordered_set<pObject::BaseObject*> dirtyObjects_;
        Quadtree collisionTree_{ 0, AABB{Vector2(-1000,-1000), Vector2(1000,1000)} };
    };
}
