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
        // �ڵ�������������
        static constexpr int MAX_OBJECTS = 8;
        static constexpr int MAX_LEVELS = 5;

        Quadtree(int level, const AABB& bounds);

        // ���Ľӿ�
        void Insert(pObject::RigidBody* body);
        void Remove(pObject::RigidBody* body);
        void Clear();
        void Query(std::vector<pObject::RigidBody*>& results, const AABB& range);

        //// ���Ի���
        //void DebugDraw() const;
        ~Quadtree();
    private:
        void Split();
        int GetIndex(const AABB& aabb) const;

        int level_;                          // ��ǰ�ڵ����
        AABB bounds_;                        // �ڵ�߽�
        std::vector<pObject::RigidBody*> objects_; // �洢�Ķ���
        std::array<Quadtree*, 4> children_; // �ĸ��ӽڵ�
    };
}