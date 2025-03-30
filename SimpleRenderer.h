#pragma once
#include "PhysicsEngine/PhysicsWorld.h"
#include <graphics.h>
#include <conio.h>
#include <cmath>
using namespace pEngine;

class SimpleRenderer
{
public :
	SimpleRenderer(Vector2 sizeOfWindow = Vector2(640,480),float pixelsPerMeter = 50.0);
	~SimpleRenderer();

	void RigisterObject(std::unique_ptr<pObject::BaseObject> object) { world.AddObject(std::move(object)); }
	void DeRigisterObject(pObject::BaseObject* object) { world.RemoveObject(object); }

	void UpdateWorld(float delta_t) { world.Update(delta_t); };
	void Render();
	//World and Screen Position are related to normal coordinates system
	Vector2 WorldToScreen(const Vector2& worldPos) const;
	Vector2 ScreenToWorld(const Vector2& screenPos) const;
	Vector2 NormalToEasyX(const Vector2& normal) const;
	Vector2 EasyXToNormal(const Vector2& easyX) const;

	double PixelToMeter(const double& pixels)const;
	double MeterToPixel(const double& meters) const;

	Vector2 PixelToMeter(const Vector2& pixels)const;
	Vector2 MeterToPixel(const Vector2& meters) const;

	// pos - pixel easyX
	void SetTipInScreen(std::string tip, Vector2 tipPos) { this->tip = tip; this->tipPos = tipPos; }
	void ClearTip() { this->tip = ""; }
	std::string GetTip() { return tip; }
	void MoveCamera(Vector2 delta) {
		cameraPosition = cameraPosition + delta;
	}
	void SetCameraPosition(Vector2 position) {
		cameraPosition = position;
	}
	void Zoom(float factor) {
		pixelsPerMeter *= factor;
	}
	void DebugObjSituation();
	void SetCameraFollow(pObject::BaseObject* obj);
private:
	Vector2 cameraPosition;
	Vector2 sizeOfWindow;
	float pixelsPerMeter;
	PhysicsWorld world;
	Vector2 Zero;//(pix,pix)  related to easyX
	pObject::BaseObject* following;
	bool isFollowing;
	
	std::string tip;
	Vector2 tipPos;

	void DrawAxes();
	void DrawPhysicsObject(pObject::BaseObject* obj);
	void DrawTip();
};

