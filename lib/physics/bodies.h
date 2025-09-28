#ifndef BODIES_H
#define BODIES_H

#include "collision.h"
#include "vector2.h"

typedef struct Material2D{
    float bounciness;
    float friction;
} Material2D;

typedef struct Rigidbody2D{
    float mass;

    Vector2D pos;
    Vector2D vel;
    Vector2D force;

    float angle; //radians

    Collider2D collider;
    Material2D material;
} Rigidbody2D;

void rigid_body_2d_init(Rigidbody2D* body, float mass, Vector2D pos, float angle,  Collider2D collider);
void rigid_body_2d_add_force(Rigidbody2D* body, Vector2D force);
void rigid_body_2d_clear_forces(Rigidbody2D* body);
void rigid_body_2d_set_material(Rigidbody2D* body, Material2D material);
void rigid_body_2d_set_velocity(Rigidbody2D* body, Vector2D velocity);
void rigid_body_2d_update(Rigidbody2D* body, float delta_time);

typedef struct Staticbody2D{
    Vector2D pos;
    float angle;    //radians

    Collider2D collider;
    Material2D material;
} Staticbody2D;

void static_body_2d_init(Staticbody2D* body, Vector2D pos, float angle,  Collider2D collider);

#endif

 