#include "bodies.h"
#include "vector2.h"
#include "collision.h"

void rigid_body_2d_init(Rigidbody2D* body, float mass, Vector2D pos, float angle,  Collider2D collider){
    body->mass = mass;
    body->pos = pos;
    body->angle = angle;
    body->collider = collider;
    body->vel = vec_2d(0, 0);
    body->force = vec_2d(0, 0);
    body->material = (Material2D){0, 0};
}

void rigid_body_2d_add_force(Rigidbody2D* body, Vector2D force){
    body->force = vec_2d_add(body->force, force);
}

void rigid_body_2d_clear_forces(Rigidbody2D* body){
    body->force = vec_2d(0, 0);
}

void rigid_body_2d_set_material(Rigidbody2D* body, Material2D material){
    body->material = material;
}

void rigid_body_2d_set_velocity(Rigidbody2D* body, Vector2D velocity){
    body->vel = velocity;
}

void rigid_body_2d_update(Rigidbody2D* body, float delta_time){
    body->vel = vec_2d_add(body->vel, vec_2d_scale(body->force, (1.0f/(body->mass)) * delta_time)); // vel = vel_0 + f/m * dt
    body->pos = vec_2d_add(body->pos, vec_2d_scale(body->vel, delta_time));
    rigid_body_2d_clear_forces(body);
}

void static_body_2d_init(Staticbody2D* body, Vector2D pos, float angle,  Collider2D collider){
    body->pos = pos;
    body->angle = angle;
    body->collider = collider;
}
