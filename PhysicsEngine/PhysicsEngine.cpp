#include "PhysicsEngine.h"
namespace pEngine
{
	double calSByVT(const Vector2& velocity, double t)
	{
		return velocity.getMagnitude() * t;
	}
	double calXByVT(const Vector2& velocity, double t, double dir)
	{
		return velocity.getProjection(dir) * t;
	}

	Force::Force(const Vector2& forceVector, double time, Vector2 applyPoint) : forceVector(forceVector), durationTime(time),applyPoint(applyPoint)
	{
	}

	Force::Force(double fx, double fy, double time, Vector2 applyPoint) : durationTime(time),forceVector(Vector2(fx,fy)),applyPoint(applyPoint)
	{
	}
	bool AABB::Overlaps(const AABB& other) const {
		return (max.getVx() > other.min.getVx()) && (min.getVx() < other.max.getVx()) &&
			(max.getVy() > other.min.getVy()) && (min.getVy() < other.max.getVy());
	}

    bool OBB::Overlaps(const OBB& other) const {
        Vector2 axes[4] = {
            axes[0], axes[1], other.axes[0], other.axes[1]
        };

        for (auto& axis : axes) {
            if (axis.getMagnitude() * axis.getMagnitude() < FLT_EPSILON) continue;
            axis = axis.Normalized();
            double proj1 = extents.getVx() * fabs(Dot(axes[0], axis)) +
                extents.getVy() * fabs(Dot(axes[1], axis));

            double proj2 = other.extents.getVx() * fabs(Dot(other.axes[0], axis)) +
                other.extents.getVy() * fabs(Dot(other.axes[1], axis));

            double distance = fabs(Dot(center - other.center, axis));

            if (distance > proj1 + proj2) {
                return false; // ´æÔÚ·ÖÀëÖá
            }
        }

        return true;
    }
}