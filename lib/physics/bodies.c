#include "bodies.h"
#include "vector2.h"
#include "collision.h"
#include <math.h>
#include <stdlib.h>

#define OVERLAP_FIX_RATIO 0.8f
#define OVERLAP_TOLERANCE 0.01f

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

static Vector2D calculate_impulse_after_collision(Vector2D collision_normal, float bounciness_a, float bounciness_b,
         Vector2D vel_a, Vector2D vel_b, float inv_mass_a, float inv_mass_b){
    
    Vector2D relative_vel = vec_2d_sub(vel_b, vel_a);
    float velAlongNormal = vec_2d_dot(relative_vel, collision_normal);

    // separating
    if (velAlongNormal > 0.0f) {
        return vec_2d(0, 0);
    }

    float bounciness = fmaxf(bounciness_a, bounciness_b);
    float impulse_factor = -(1.0f + bounciness) * velAlongNormal / (inv_mass_a + inv_mass_b);

    return vec_2d_scale(collision_normal, impulse_factor);
}


void handle_rigid_body_collision(Rigidbody2D* a, Rigidbody2D* b) {
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
            a->vel, b->vel,
            1.0f / a->mass, 1.0f / b->mass
        );

        a->vel = vec_2d_sub(a->vel, vec_2d_scale(impulse, 1.0f / a->mass));
        b->vel = vec_2d_add(b->vel, vec_2d_scale(impulse, 1.0f / b->mass));

        //remove force along the collision normal vecotr
        float force_along_normal_a = vec_2d_dot(a->force, result.normal_vec);
        if (force_along_normal_a > 0.0f) {
            a->force = vec_2d_sub(a->force, vec_2d_scale(result.normal_vec, force_along_normal_a));
        }

        float force_along_normal_b = vec_2d_dot(b->force, result.normal_vec);
        if (force_along_normal_b < 0.0f) {
            b->force = vec_2d_sub(b->force, vec_2d_scale(result.normal_vec, force_along_normal_b));
        }            
        
    }

}

void static_body_2d_init(Staticbody2D* body, Vector2D pos, float angle,  Collider2D collider){
    body->pos = pos;
    body->angle = angle;
    body->collider = collider;
}

void handle_static_body_collision(Rigidbody2D* a, Staticbody2D* b) {
    CollisionResult result = collision_2d_check(
        a->collider, a->pos, a->angle,
        b->collider, b->pos, b->angle
    );

    if (result.did_collide) {
        correct_overlapping(result, &a->pos, 1.0f / a->mass, NULL, 0.0f);

        
        Vector2D impulse = calculate_impulse_after_collision(
            result.normal_vec,
            a->material.bounciness, b->material.bounciness,
            a->vel, vec_2d(0.0f, 0.0f),
            1.0f / a->mass, 0.0f //we assume static bodies have infinite mass
        );

        a->vel = vec_2d_sub(a->vel, vec_2d_scale(impulse, 1.0f / a->mass));

        float force_along_normal_axis = vec_2d_dot(a->force, result.normal_vec);
        if (force_along_normal_axis > 0.0f) {
            a->force = vec_2d_sub(a->force, vec_2d_scale(result.normal_vec, force_along_normal_axis));
        }

    }
}
