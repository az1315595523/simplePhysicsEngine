#pragma once
// Quadtree.h
#pragma once
#include "PhysicsObject.h"
#include <memory>
#include <array>
#include <vector>

namespace pEngine {

    class Quadtree {
    public:
        // 节点容量和最大深度
        static constexpr int MAX_OBJECTS = 8;
        static constexpr int MAX_LEVELS = 5;

        Quadtree(int level, const AABB& bounds);

        // 核心接口
        void Insert(pObject::RigidBody* body);
        void Remove(pObject::RigidBody* body);
        void Clear();
        void Query(std::vector<pObject::RigidBody*>& results, const AABB& range);

        //// 调试绘制
        //void DebugDraw() const;
        ~Quadtree();
    private:
        void Split();
        int GetIndex(const AABB& aabb) const;

        int level_;                          // 当前节点深度
        AABB bounds_;                        // 节点边界
        std::vector<pObject::RigidBody*> objects_; // 存储的对象
        std::array<Quadtree*, 4> children_; // 四个子节点
    };
}