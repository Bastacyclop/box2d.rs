
#include "Box2D/c_box2d.h"

#include <stdio.h>

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box2D is meant to be used
// with your rendering engine in your game engine.
int main(int argc, char** argv)
{
    // Define the gravity vector.
    box2d_Vec2 gravity = { 0.0f, -10.0f };

    // Construct a world object, which will hold and simulate the rigid bodies.
    box2d_World* world = box2d_World_Create(&gravity);
/*
    // Define the ground body.
    box2d_BodyDef groundBodyDef = box2d_BodyDef_Create();
    groundBodyDef.position.x = 0.0f;
    groundBodyDef.position.y = -10.0f;

    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    box2d_Body* groundBody = box2d_World_CreateBody(world, &groundBodyDef);

    // Define the ground box shape.
    box2d_PolygonShape* groundBox = box2d_PolygonShape_Create();
    box2d_PolygonShape_SetAsBox(groundBox, 50.0f, 10.0f);

    // Add the ground fixture to the ground body.
    box2d_Body_CreateFixture_shape(groundBody, box2d_PolygonShape_Upcast(groundBox), 0.0f);
    //box2d_PolygonShape_Destroy(groundBox);
*/

    // Define the dynamic body. We set its position and call the body factory.
    box2d_BodyDef bodyDef = box2d_BodyDef_Create();
    bodyDef.type = box2d_dynamicBody;
    bodyDef.position.x = 0.0f;
    bodyDef.position.y = 4.0f;
    box2d_Body* body = box2d_World_CreateBody(world, &bodyDef);
    // Define another box shape for our dynamic body.
    box2d_PolygonShape* dynamicBox = box2d_PolygonShape_Create();
    box2d_PolygonShape_SetAsBox(dynamicBox, 1.0f, 1.0f);

    // Define the dynamic body fixture.
    box2d_FixtureDef fixtureDef;
    fixtureDef.shape = box2d_PolygonShape_Upcast(dynamicBox);

    // Set the box density to be non-zero, so it will be dynamic.
    fixtureDef.density = 1.0f;

    // Override the default friction.
    fixtureDef.friction = 0.3f;

    // Add the shape to the body.
    box2d_Body_CreateFixture(body, &fixtureDef);

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float32 timeStep = 1.0f / 60.0f;
    int32 velocityIterations = 6;
    int32 positionIterations = 2;

    int i;
    // This is our little game loop.
    for (i = 0; i < 60; ++i)
    {
        box2d_Vec2 position;
        float32 angle;

        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        box2d_World_Step(world, timeStep, velocityIterations, positionIterations);

        // Now print the position and angle of the body.
        position = *box2d_Body_GetPosition(body);
        angle = box2d_Body_GetAngle(body);

        printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
    }

    // When the world destructor is called, all bodies and joints are freed. This can
    // create orphaned pointers, so be careful about your world management.

    box2d_World_Destroy(world);

    printf("sizeof fixtureDef: %i\n", (int)sizeof(box2d_BodyDef));
    return 0;
}
