#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "collision.h"
#include "vector2.h"

typedef struct Rigidbody2D{
    float mass;

    Vector2D pos;
    Vector2D vel;
    Vector2D force;

    float angle; //radians

    Collider2D collider;
    float bounciness;
    float friction; //its both the static kinetic coefs for now 
} Rigidbody2D;

void rigid_body_2d_init(Rigidbody2D* body, float mass, Vector2D pos, float angle,  Collider2D collider);
void rigid_body_2d_add_force(Rigidbody2D* body, Vector2D force);
void rigid_body_2d_clear_forces(Rigidbody2D* body);
void rigid_body_2d_set_bounciness(Rigidbody2D* body, float bounciness);
void rigid_body_2d_set_friction(Rigidbody2D* body, float friction);
void rigid_body_2d_set_velocity(Rigidbody2D* body, Vector2D velocity);
void rigid_body_2d_update(Rigidbody2D* body, float delta_time);

#endif

 