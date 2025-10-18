#ifndef WORLD_H
#define WORLD_H

#include "bodies.h"
#include "vector2.h"

typedef enum {
    RIGIDBODY,
    STATICBODY,
    SOFTBODY,
    SPRING
} PhysicsObjectType;

typedef struct {
    PhysicsObjectType type;
    union {
        Rigidbody2D* rigidbody;
        Staticbody2D* staticbody;
        Softbody2D* softbody;
        Spring2D* spring;
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
PhysicsObject* world_2d_add_static_body(World2D* world, Staticbody2D* sb);
PhysicsObject* world_2d_add_soft_body(World2D* world, Softbody2D* sb);
PhysicsObject* world_2d_add_spring(World2D* world, Spring2D* sp);

CollisionResult world_2d_check_objects_collision(PhysicsObject* a, PhysicsObject* b);
CollisionResult world_2d_check_collision_rigid_body_rigid_body(Rigidbody2D* a, Rigidbody2D* b);
CollisionResult world_2d_check_collision_rigid_body_static_body(Rigidbody2D* a, Staticbody2D* b);
CollisionResult world_2d_check_collision_static_body_static_body(Staticbody2D* a, Staticbody2D* b);

void world_2d_step(World2D* world, float delta_time);
void world_2d_step_with_substeps(World2D* world, float delta_time, int substep_amount);


#endif
