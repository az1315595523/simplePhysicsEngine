#include "SimpleRenderer.h"
#include "iostream"

SimpleRenderer::SimpleRenderer(Vector2 sizeOfWindow, float pixelsPerMeter) : sizeOfWindow(sizeOfWindow), pixelsPerMeter(pixelsPerMeter)
{
    initgraph(sizeOfWindow.getVx(), sizeOfWindow.getVy(),SHOWCONSOLE);
    setbkcolor(WHITE);
    cleardevice();
    Zero = Vector2(sizeOfWindow.getVx() / 2.0f, sizeOfWindow.getVy() / 2.0f);
    SetCameraPosition(Vector2(0,0));

}
SimpleRenderer::~SimpleRenderer() {
    closegraph();
}

void SimpleRenderer::DrawTip()
{
    settextcolor(RED);              
    setbkmode(TRANSPARENT);     
    wchar_t wtext[256];
    MultiByteToWideChar(CP_ACP, 0, tip.c_str(), -1, wtext, 256);

    outtextxy(
        static_cast<int>(tipPos.getVx()), // EasyX 的 x 坐标
        static_cast<int>(tipPos.getVy()), // EasyX 的 y 坐标（左上角为原点）
        wtext
    );
}

void SimpleRenderer::Render()
{
    BeginBatchDraw();
    cleardevice();
    
    DrawAxes();

    DrawTip();

    if (isFollowing)
    {
        SetCameraPosition(MeterToPixel(following->getPosition()));
    }

    for (pObject::BaseObject* obj : world.GetObjects())
    {
        if (obj->IsDirty())
            DrawPhysicsObject(obj);
    }
    FlushBatchDraw();
}

void SimpleRenderer::DrawAxes()
{
    setlinecolor(RGB(200, 200, 200));


    // 计算屏幕上的原点位置（相对相机）
    Vector2 screenZero = NormalToEasyX(WorldToScreen(Vector2(0, 0)));

    // 计算 X 轴和 Y 轴在屏幕上的绘制范围
    Vector2 axisX_start = Vector2(0, screenZero.getVy());
    Vector2 axisX_end = Vector2(sizeOfWindow.getVx(), screenZero.getVy());
    Vector2 axisY_start = Vector2(screenZero.getVx(), 0);
    Vector2 axisY_end = Vector2(screenZero.getVx(), sizeOfWindow.getVy());

    line(axisX_start.getVx(), axisX_start.getVy(), axisX_end.getVx(), axisX_end.getVy());
    line(axisY_start.getVx(), axisY_start.getVy(), axisY_end.getVx(), axisY_end.getVy());
    settextcolor(BLACK);
    setbkmode(TRANSPARENT);


    //pixel
    float xStep = sizeOfWindow.getVx()/10 ;
    float yStep = sizeOfWindow.getVy() / 10;


    float startX = cameraPosition.getVx()- sizeOfWindow.getVx()/2;
    float endX = startX + sizeOfWindow.getVx();

    float startY = cameraPosition.getVy()- sizeOfWindow.getVy()/2;
    float endY = startY + sizeOfWindow.getVy();

    for (float x = floor(startX / xStep) * xStep; x <= endX; x += xStep) {
        float screenX = NormalToEasyX(WorldToScreen(Vector2(x,0))).getVx();
        line(screenX, screenZero.getVy() - 5, screenX, screenZero.getVy() + 5);

        TCHAR text[16];
        _stprintf_s(text, _T("%.1fm"), PixelToMeter(x));
        outtextxy(screenX - 10, screenZero.getVy() + 10, text);
    }

    for (float y = floor(startY / yStep) * yStep; y <= endY; y += yStep) {
        if (y != 0)
        {
            float screenY = NormalToEasyX(WorldToScreen(Vector2(0,y))).getVy();
            line(screenZero.getVx() - 5, screenY, screenZero.getVx() + 5, screenY);

            TCHAR text[16];
            _stprintf_s(text, _T("%.1fm"), PixelToMeter(y));
            outtextxy(screenZero.getVx() - 30, screenY + 5, text);
        }
    }
}

void SimpleRenderer::DrawPhysicsObject(pObject::BaseObject* obj)
{
    if (auto rigidBody = dynamic_cast<pObject::RigidBody*>(obj))
    {
        if (auto circle = dynamic_cast<pEngine::CircleCollider*>(&rigidBody->GetCollider()))
        {
            Vector2 worldPos = MeterToPixel(rigidBody->getCenterOfMass());
            Vector2 screenPos = WorldToScreen(worldPos);
            Vector2 drawPos = NormalToEasyX(screenPos);

            int radius = static_cast<int>(MeterToPixel(circle->getRadius()));
            setfillcolor(RGB(0, 255,0));
            fillcircle(drawPos.getVx(), drawPos.getVy(),radius);
        }
        else if (auto box = dynamic_cast<pEngine::BoxCollider*>(&rigidBody->GetCollider()))
        {

            OBB obb = box->GetOBB();
            Vector2 extents = MeterToPixel(box->GetExtents());
            Vector2 center = MeterToPixel(obb.center);

            Vector2 axes[2] = { obb.axes[0],obb.axes[1] };
            center = NormalToEasyX(WorldToScreen(center));

            float angle = rigidBody->getAngle();
            POINT pts[4];
            pts[0] = {
                static_cast<LONG>(center.getVx() + axes[0].getVx() * extents.getVx() + axes[1].getVx() * extents.getVy()),
                static_cast<LONG>(center.getVy() + axes[0].getVy() * extents.getVx() + axes[1].getVy() * extents.getVy())
            };
            pts[1] = {
                static_cast<LONG>(center.getVx() + axes[0].getVx() * extents.getVx() - axes[1].getVx() * extents.getVy()),
                static_cast<LONG>(center.getVy() + axes[0].getVy() * extents.getVx() - axes[1].getVy() * extents.getVy())
            };
            pts[2] = {
                static_cast<LONG>(center.getVx() - axes[0].getVx() * extents.getVx() - axes[1].getVx() * extents.getVy()),
                static_cast<LONG>(center.getVy() - axes[0].getVy() * extents.getVx() - axes[1].getVy() * extents.getVy())
            };
            pts[3] = {
                static_cast<LONG>(center.getVx() - axes[0].getVx() * extents.getVx() + axes[1].getVx() * extents.getVy()),
                static_cast<LONG>(center.getVy() - axes[0].getVy() * extents.getVx() + axes[1].getVy() * extents.getVy())
            };
            
            setfillcolor(RGB(0, 255, 0));
            fillpolygon(pts, 4);
        }
    }
    else
    {

    }
}

// pos - base on pixel and normal
Vector2 SimpleRenderer::WorldToScreen(const Vector2& worldPos) const
{
    //example  cameraPos = (-2,0) and objWorldPos = (-1,1)  then   objScreenPos = (1,1)
    Vector2 cameraSpacePos = worldPos - cameraPosition;
    return cameraSpacePos;
}
Vector2 SimpleRenderer::ScreenToWorld(const Vector2& screenPos) const
{
    Vector2 worldPos = screenPos + cameraPosition;
    return worldPos;
}
//

Vector2 SimpleRenderer::NormalToEasyX(const Vector2& normal) const
{
    return Vector2(
        Zero.getVx() + normal.getVx(),  
        Zero.getVy() - normal.getVy()   
    );
}

Vector2 SimpleRenderer::EasyXToNormal(const Vector2& easyX) const
{
    return Vector2(
        easyX.getVx() - Zero.getVx(), 
        Zero.getVy() - easyX.getVy()  
    );
}

double SimpleRenderer::PixelToMeter(const double& pixels) const
{
    return pixels*pixelsPerMeter;
}

double SimpleRenderer::MeterToPixel(const double& meters) const
{
    return meters/pixelsPerMeter;
}

Vector2 SimpleRenderer::PixelToMeter(const Vector2& pixels) const
{
    return pixels*pixelsPerMeter;
}

Vector2 SimpleRenderer::MeterToPixel(const Vector2& meters) const
{
    return meters / pixelsPerMeter;
}

void SimpleRenderer::DebugObjSituation()
{
    int count = 0;
    for (pObject::BaseObject* obj : world.GetObjects())
    {
        std::cout << "obj " << count
            << " Position:" << obj->getPosition().getVx() << "," << obj->getPosition().getVy()
            << " Velocity:" << obj->getVelocity().getVx() << "," << obj->getVelocity().getVy();
        if (auto rigidBody = dynamic_cast<pObject::RigidBody*>(obj))
        {
            std::cout << " AngularVelocity:" << rigidBody->getAngularVelocity()
                << " Angle:" << rigidBody->getAngle();
        }
        std::cout << std::endl;
        count++;
    }
}

void SimpleRenderer::SetCameraFollow(pObject::BaseObject* obj)
{
    this->following = obj;
    if (obj == NULL)
    {
        this->isFollowing = false;
    }
    else
    {
        this->isFollowing = true;
    }
}

