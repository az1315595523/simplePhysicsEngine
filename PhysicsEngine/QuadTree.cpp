#include "Quadtree.h"

namespace pEngine {

    Quadtree::Quadtree(int level, const AABB& bounds)
        : level_(level), bounds_(bounds), children_{nullptr} {
    }

    void Quadtree::Insert(pObject::RigidBody* body) {
        
        if (children_[0]) {
            int index = GetIndex(body->GetCollider().GetAABB());
            if (index != -1) {
                children_[index]->Insert(body);
                return;
            }
        }

        objects_.push_back(body);
        if (objects_.size() > MAX_OBJECTS && level_ < MAX_LEVELS) {
            if (!children_[0]) Split();
            auto it = objects_.begin();
            while (it != objects_.end()) {
                int index = GetIndex((*it)->GetCollider().GetAABB());
                if (index != -1) {
                    children_[index]->Insert(*it);
                    it = objects_.erase(it);
                }
                else {
                    ++it;
                }
            }
        }
    }

    void Quadtree::Remove(pObject::RigidBody* body)
    {
        // 从当前节点移除
        auto it = std::find(objects_.begin(), objects_.end(), body);
        if (it != objects_.end()) {
            objects_.erase(it);
            return;
        }

        // 递归从子节点移除
        if (children_[0]) {
            for (auto& child : children_) {
                child->Remove(body);
            }
        }
    }

    void Quadtree::Clear()
    {
        objects_.clear();
        if (children_[0]) {
            for (auto& child : children_) {
                child->Clear();
            }
        }
        children_.fill(nullptr); // 重置所有子节点
    }

    void Quadtree::Split() {

        if (children_[0] != nullptr) return;

        const double halfWidth = (bounds_.max.getVx() - bounds_.min.getVx()) / 2;
        const double halfHeight = (bounds_.max.getVy() - bounds_.min.getVy()) / 2;
        const Vector2 center = {
            bounds_.min.getVx() + halfWidth,
            bounds_.min.getVy() + halfHeight
        };

        // 创建四个子节点
        children_[0] = new Quadtree(level_ + 1, // NW
            AABB{ bounds_.min, center });

        children_[1] = new Quadtree(level_ + 1, // NE
            AABB{ {center.getVx(), bounds_.min.getVy()},
                   {bounds_.max.getVx(), center.getVy()} });

        children_[2] = new Quadtree(level_ + 1, // SW
            AABB{ {bounds_.min.getVx(), center.getVy()},
                   {center.getVx(), bounds_.max.getVy()} });

        children_[3] = new Quadtree(level_ + 1, // SE
            AABB{ center, bounds_.max });
    }

    int Quadtree::GetIndex(const AABB& aabb) const {
        Vector2 center = {
            (bounds_.min.getVx() + bounds_.max.getVx()) / 2,
            (bounds_.min.getVy() + bounds_.max.getVy()) / 2
        };

        bool top = aabb.min.getVy() > center.getVy();
        bool bottom = aabb.max.getVy() <= center.getVy();
        bool left = aabb.max.getVx() <= center.getVx();
        bool right = aabb.min.getVx() > center.getVx();

        if (top) {
            if (left) return 0; // NW
            if (right) return 1; // NE
        }
        else if (bottom) {
            if (left) return 2; // SW
            if (right) return 3; // SE
        }
        return -1; // 跨多个区域
    }

    void Quadtree::Query(std::vector<pObject::RigidBody*>& results, const AABB& range) {
        if (!bounds_.Overlaps(range)) return;

        for (auto obj : objects_) {
            if (obj->GetCollider().GetAABB().Overlaps(range)) {
                results.push_back(obj);
            }
        }

        if (children_[0]) {
            for (auto& child : children_) {
                if(child)
                child->Query(results, range);
            }
        }
    }
    Quadtree::~Quadtree()
    {
        for (int i = 0; i < 4; i++)
        {
            delete children_[i];
            children_[i] = nullptr;
        }
    }
}