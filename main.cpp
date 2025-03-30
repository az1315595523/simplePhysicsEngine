#include <iostream>
#include "SimpleRenderer.h"
#include <chrono>
#include <thread>
#include <Windows.h>

void Example()
{
    SimpleRenderer renderer(pEngine::Vector2(800, 600),0.01);


    std::unique_ptr<pObject::RigidBody> circle1(new pObject::RigidBody(3.0));
    circle1.get()->AddCircleCollider(0.0, 0.6);
    circle1.get()->setPosition(2, 3);
    circle1.get()->setVelocity(-0.3, -0.6);

    std::unique_ptr<pObject::RigidBody> box1(new pObject::RigidBody(4.0));
    box1.get()->AddBoxCollider(0.0, 0.4, 0.6);
    box1.get()->setPosition(-2, 1.5);
    box1.get()->setVelocity(0.3, -0.6);

    pObject::RigidBody* rawBoxPtr1 = circle1.get();
    pObject::RigidBody* rawBoxPtr2 = box1.get();

    //F = ma   a = F/m
    box1.get()->addForce(-0.8, 1, 5);

    renderer.RigisterObject(std::move(circle1));
    renderer.RigisterObject(std::move(box1));

    renderer.SetCameraFollow(rawBoxPtr1);

    auto lastFrameTime = std::chrono::high_resolution_clock::now();
    auto lastPrintTime = lastFrameTime;
    int frameCounter = 0;

    while (1) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float>(currentTime - lastFrameTime).count();
        lastFrameTime = currentTime;
        renderer.UpdateWorld(static_cast<float>(deltaTime));
        renderer.Render();

        frameCounter++;
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedSec = now - lastPrintTime;
        if (elapsedSec.count() >= 1.0) {
            std::cout << "1 second last" << std::endl;
            std::cout << "FPS:  " << frameCounter << std::endl;
            frameCounter = 0;
            lastPrintTime = now;
            renderer.DebugObjSituation();
        }
    }

}

void Game1()
{
    SimpleRenderer renderer(pEngine::Vector2(800, 600), 0.01);


    std::unique_ptr<pObject::RigidBody> circle1(new pObject::RigidBody(1.0));
    circle1.get()->AddCircleCollider(0.0, 0.1);
    circle1.get()->setPosition(0, 10);
    circle1.get()->setVelocity(0, 0);

    pObject::RigidBody* rawBoxPtr1 = circle1.get();

    renderer.RigisterObject(std::move(circle1));

    //std::unique_ptr<pObject::RigidBody> ground(new pObject::RigidBody(99999999999999999999999999.0));
    //ground.get()->AddBoxCollider(0.0, 99999999999999999999999999.0,0.1);
    //ground.get()->setPosition(0, 0);
    //ground.get()->setVelocity(0, 0);

    //renderer.RigisterObject(std::move(ground));

    renderer.SetCameraFollow(rawBoxPtr1);

    auto lastFrameTime = std::chrono::high_resolution_clock::now();


    int gameState = 0; 
    bool isMousePressed = false;
    Force* gravity = NULL;
    double bestScore = 0.0;
    while (1) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float>(currentTime - lastFrameTime).count();
        lastFrameTime = currentTime;
        renderer.UpdateWorld(static_cast<float>(deltaTime));
        renderer.Render();

        bool currentMouseState = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;

        // limit the ball's pos
        Vector2 bounds[2] = {
            Vector2(0,8),
            Vector2(2,12)
        };
        
        ExMessage mouseMsg;
        switch (gameState) 
        {
            case 0:  // UnReady
            {
                if (renderer.GetTip().empty()) 
                {
                    renderer.SetTipInScreen("Press the mouse left-button to throw the circle!", Vector2(300, 100));
                }
                // CheckMouseLeftButton
                if (currentMouseState && !isMousePressed) {
                    gameState = 1;
                    renderer.ClearTip();
                    isMousePressed = true;
                }
                break;
            }
            case 1:  // ReadyToThrow!
            {
                if (renderer.GetTip().empty())
                {
                    renderer.SetTipInScreen("Release the left-buton and throw it out to get the farthest distance!", Vector2(300, 100));
                }
                if (peekmessage(&mouseMsg,EX_MOUSE))
                {
                    if (mouseMsg.message == WM_MOUSEMOVE)
                    {
                        Vector2 cursorPos_screen_easyX_pixels = Vector2(mouseMsg.x, mouseMsg.y);
                        Vector2 cursorPos_screen_normal_pixels = renderer.EasyXToNormal(cursorPos_screen_easyX_pixels);
                        Vector2 cursorPos_world_normal_pixels = renderer.ScreenToWorld(cursorPos_screen_normal_pixels);
                        Vector2 cursorPos_world_normal_meters = renderer.PixelToMeter(cursorPos_world_normal_pixels);
                        std::cout << "Mouse Position: (" << cursorPos_world_normal_meters.getVx() << ", " << cursorPos_world_normal_meters.getVy() << ")\n";

                        Vector2 pos = cursorPos_world_normal_meters.Clamp(bounds[0], bounds[1]);
                        rawBoxPtr1->setPosition(pos);
                    }
                }

                // CheckMouseLeftButtonRelease
                if (!currentMouseState && isMousePressed) 
                {
                    Vector2 pos = rawBoxPtr1->getPosition();
                    Vector2 forcePower = (Vector2(0, 10) - pos)*10 * rawBoxPtr1->getMass();

                    Force* force = rawBoxPtr1->addForce(forcePower.getVx(),forcePower.getVy(),2.0);
                    gravity = rawBoxPtr1->setGravity();

                    gameState = 2;
                    renderer.ClearTip();
                    isMousePressed = false;
                }
                break;
            }
            case 2: // ball is flying!!!
            {
                std::string tip = "x:" + std::to_string(rawBoxPtr1->getPosition().getVx()) + " y:" + std::to_string(rawBoxPtr1->getPosition().getVy());
                renderer.SetTipInScreen(tip, Vector2(300, 100));
                if (rawBoxPtr1->getPosition().getVy() <= 0.0 && rawBoxPtr1->getVelocity().getVy()<0)
                {
                    // simple simulation you can also set a rigidyBody with a collider to simulate
                    rawBoxPtr1->setVelocity(rawBoxPtr1->getVelocity().getVx() * 0.8, -rawBoxPtr1->getVelocity().getVy() * 0.8);
                }
                if (rawBoxPtr1->getPosition().getVy() <= 0.0 && abs(rawBoxPtr1->getVelocity().getVy())<=0.0001f)
                {
                    rawBoxPtr1->RemoveAllForces();
                    rawBoxPtr1->setVelocity(Vector2(0, 0));
                    renderer.ClearTip();
                    gameState = 3;
                }
                break;
            }
            case 3:
            {
                if (renderer.GetTip().empty())
                {
                    double score = -rawBoxPtr1->getPosition().getVx();
                    bestScore = max(score, bestScore);
                    std::string tip = "Your score is:" + std::to_string(score) + " and Your best score is:" + std::to_string(bestScore) 
                        + " Press left-Button to restart!";
                    renderer.SetTipInScreen(tip, Vector2(0, 100));
                }
                if (currentMouseState && !isMousePressed)
                {
                    isMousePressed = true;
                }
                if (!currentMouseState && isMousePressed)
                {
                    gameState = 0;
                    renderer.ClearTip();
                    rawBoxPtr1->setPosition(Vector2(0, 10));
                    isMousePressed = false;
                }
            }
        }
    }

}

int main()
{

    Game1();
    //Example();


	return 0;
}