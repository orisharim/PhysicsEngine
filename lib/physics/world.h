#ifndef WORLD_H
#define WORLD_H

#include "bodies.h"
#include "vector2.h"

typedef enum {
    RIGIDBODY,
    STATICBODY,
    
} PhysicsObjectType;

typedef struct {
    PhysicsObjectType type;
    union {
        Rigidbody2D* rigidbody;
        Staticbody2D* staticbody;
        
    };
} PhysicsObject;

typedef struct {
    PhysicsObject* objects;
    int count;
    int capacity;

    float gravity_constant;
    Vector2D gravity_dir;
} World2D;

World2D* world_2d_create(int initial_capacity);
void world_2d_set_gravity(World2D* world, float gravity_constant, Vector2D gravity_dir);
void world_2d_destroy(World2D* world);

PhysicsObject* world_2d_add_object(World2D* world, PhysicsObject* obj);
void world_2d_remove_object(World2D* world, PhysicsObject* obj);

PhysicsObject* world_2d_add_rigid_body(World2D* world, Rigidbody2D* rb);
PhysicsObject* world_2d_add_static_body(World2D* world, Staticbody2D* rb);

CollisionResult world_2d_check_objects_collision(PhysicsObject* a, PhysicsObject* b);
CollisionResult world_2d_check_collision_rigid_body_rigid_body(Rigidbody2D* a, Rigidbody2D* b);
CollisionResult world_2d_check_collision_rigid_body_static_body(Rigidbody2D* a, Staticbody2D* b);
CollisionResult world_2d_check_collision_static_body_static_body(Staticbody2D* a, Staticbody2D* b);


void world_2d_step(World2D* world, float delta_time);

#endif
