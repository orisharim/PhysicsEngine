#include "bodies.h"
#include "vector2.h"
#include "collision.h"
#include <math.h>
#include <stdlib.h>

#define OVERLAP_FIX_RATIO 0.2f
#define OVERLAP_TOLERANCE 0.01f

#define MIN_VELOCITY_THRESHOLD 0.001f

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
    body->vel = vec_2d_add(body->vel, vec_2d_scale(body->force, (1.0f/(body->mass)) * delta_time));
    body->pos = vec_2d_add(body->pos, vec_2d_scale(body->vel, delta_time));
    rigid_body_2d_clear_forces(body);
}

static void correct_overlapping(CollisionResult result,
                                Vector2D* a_pos, float a_inv_mass,
                                Vector2D* b_pos, float b_inv_mass) {
    float total_inv_mass = a_inv_mass + b_inv_mass;
    if (total_inv_mass <= 0.0f) return;

    float correction_mag = fmaxf(result.penetration - OVERLAP_TOLERANCE, 0.0f)
                           / total_inv_mass * OVERLAP_FIX_RATIO;

    Vector2D correction = vec_2d_scale(result.normal_vec, correction_mag);

    if (a_inv_mass > 0.0f)
        *a_pos = vec_2d_sub(*a_pos, vec_2d_scale(correction, a_inv_mass));

    if (b_inv_mass > 0.0f && b_pos)
        *b_pos = vec_2d_add(*b_pos, vec_2d_scale(correction, b_inv_mass));
}

static Vector2D calculate_impulse_after_collision(Vector2D collision_normal,
         float bounciness_a, float bounciness_b, 
         float friction_a, float friction_b,
         Vector2D vel_a, Vector2D vel_b, 
         float inv_mass_a, float inv_mass_b){
    Vector2D relative_vel = vec_2d_sub(vel_b, vel_a);
    float vel_along_normal = vec_2d_dot(relative_vel, collision_normal);

    // objects are separating so no impulse needed
    if (vel_along_normal > 0.0f) {
        return vec_2d(0, 0);
    }

    float bounciness = fmaxf(bounciness_a, bounciness_b);
    float normal_impulse_mag = -(1.0f + bounciness) * vel_along_normal / (inv_mass_a + inv_mass_b);
    Vector2D normal_impulse = vec_2d_scale(collision_normal, normal_impulse_mag);

    Vector2D tangent = vec_2d(-collision_normal.y, collision_normal.x);
    float vel_along_tangent = vec_2d_dot(relative_vel, tangent);
    
    float friction = fminf(friction_a, friction_b);
    
    float friction_impulse_mag = -vel_along_tangent / (inv_mass_a + inv_mass_b);
    
    float max_friction = friction * fabsf(normal_impulse_mag);
    
    if (fabsf(friction_impulse_mag) > max_friction) {
        friction_impulse_mag = (friction_impulse_mag > 0) ? max_friction : -max_friction;
    }

    Vector2D friction_impulse = vec_2d_scale(tangent, friction_impulse_mag);
    
    return vec_2d_add(normal_impulse, friction_impulse);
}

void rigid_body_2d_handle_collision(Rigidbody2D* a, Rigidbody2D* b) {
    CollisionResult result = collision_2d_check(
        a->collider, a->pos, a->angle,
        b->collider, b->pos, b->angle
    );

    if (result.did_collide) {
        
        correct_overlapping(result, &a->pos, 1.0f / a->mass,
                                     &b->pos, 1.0f / b->mass);
        
        Vector2D impulse = calculate_impulse_after_collision(
            result.normal_vec,
            a->material.bounciness, b->material.bounciness,
            a->material.friction, b->material.friction,
            a->vel, b->vel,
            1.0f / a->mass, 1.0f / b->mass
        );

        a->vel = vec_2d_sub(a->vel, vec_2d_scale(impulse, 1.0f / a->mass));
        b->vel = vec_2d_add(b->vel, vec_2d_scale(impulse, 1.0f / b->mass));

    }

}

void static_body_2d_init(Staticbody2D* body, Vector2D pos, float angle,  Collider2D collider){
    body->pos = pos;
    body->angle = angle;
    body->collider = collider;
}

void rigid_body_2d_handle_static_collision(Rigidbody2D* a, Staticbody2D* b) {
    CollisionResult result = collision_2d_check(
        a->collider, a->pos, a->angle,
        b->collider, b->pos, b->angle
    );

    if (result.did_collide) {

        correct_overlapping(result, &a->pos, 1.0f / a->mass, NULL, 0.0f);

        Vector2D impulse = calculate_impulse_after_collision(
            result.normal_vec,
            a->material.bounciness, b->material.bounciness,
            a->material.friction, b->material.friction,
            a->vel, vec_2d(0.0f, 0.0f),
            1.0f / a->mass, 0.0f //we assume static bodies have infinite mass
        );

        a->vel = vec_2d_sub(a->vel, vec_2d_scale(impulse, 1.0f / a->mass));

    }
}

void static_body_2d_set_material(Staticbody2D* body, Material2D material){
    body->material = material;
}

