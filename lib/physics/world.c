#include <stdlib.h>
#include <math.h>
#include "world.h"
#include "bodies.h"
#include "collision.h"
#include "geometry_utils.h"

#define OVERLAP_FIX_RATIO 0.8f
#define OVERLAP_TOLERANCE 0.01f

World2D* world_2d_create(int initial_capacity) {
    World2D* world = malloc(sizeof(World2D));
    if (!world) return NULL;

    world->objects = malloc(sizeof(PhysicsObject) * initial_capacity);
    world->count = 0;
    world->capacity = initial_capacity;

    world->gravity_constant = 0;
    world->gravity_dir = vec_2d(0, -1);

    return world;
}

void world_2d_destroy(World2D* world) {
    if (!world) return;

    for (int i = 0; i < world->count; i++) {
        PhysicsObject* obj = &world->objects[i];
        switch (obj->type) {
            case RIGIDBODY:
                free(obj->rigidbody);
                break;
            case STATICBODY:
                free(obj->staticbody);
                break;
        }
    }

    free(world->objects);
    free(world);
}

PhysicsObject* world_2d_add_object(World2D* world, PhysicsObject* obj) {
    if (world->count >= world->capacity) {
        world->capacity *= 2;
        world->objects = realloc(world->objects,
                                 sizeof(PhysicsObject) * world->capacity);
    }
    world->objects[world->count] = *obj;
    world->count++;
    return &world->objects[world->count - 1];
}

void world_2d_remove_object(World2D* world, PhysicsObject* obj) {
    for (int i = 0; i < world->count; i++) {
        if (&world->objects[i] == obj) {
            switch (obj->type) {
                case RIGIDBODY:
                    free(obj->rigidbody);
                    break;
                case STATICBODY:
                    free(obj->staticbody);
                    break;
            }
            world->count--;
            world->objects[i] = world->objects[world->count];
            return;
        }
    }
}

PhysicsObject* world_2d_add_rigid_body(World2D* world, Rigidbody2D* rb) {
    PhysicsObject obj = { .type = RIGIDBODY, .rigidbody = rb };
    return world_2d_add_object(world, &obj);
}

PhysicsObject* world_2d_add_static_body(World2D* world, Staticbody2D* sb) {
    PhysicsObject obj = { .type = STATICBODY, .staticbody = sb };
    return world_2d_add_object(world, &obj);
}

void world_2d_set_gravity(World2D* world, float gravity_constant, Vector2D gravity_dir) {
    world->gravity_constant = gravity_constant;
    world->gravity_dir = gravity_dir;
}

CollisionResult world_2d_check_objects_collision(PhysicsObject* a, PhysicsObject* b) {
    CollisionResult res = {0};

    if (a->type == RIGIDBODY && b->type == RIGIDBODY) 
        res = collision_2d_check(a->rigidbody->collider, a->rigidbody->pos, a->rigidbody->angle, b->rigidbody->collider, b->rigidbody->pos, b->rigidbody->angle);
    else if (a->type == RIGIDBODY && b->type == STATICBODY) 
        res = collision_2d_check(a->rigidbody->collider, a->rigidbody->pos, a->rigidbody->angle, b->staticbody->collider, b->staticbody->pos, b->staticbody->angle);
    else if (a->type == STATICBODY && b->type == RIGIDBODY) 
        res = collision_2d_check(a->staticbody->collider, a->staticbody->pos, a->staticbody->angle, b->rigidbody->collider, b->rigidbody->pos, b->rigidbody->angle);
    else if (a->type == STATICBODY && b->type == STATICBODY) 
        res = collision_2d_check(a->staticbody->collider, a->staticbody->pos, a->staticbody->angle, b->staticbody->collider, b->staticbody->pos, b->staticbody->angle);
    
    return res;
}

CollisionResult world_2d_check_collision_rigid_body_rigid_body(Rigidbody2D* a, Rigidbody2D* b) {
    return collision_2d_check(
        a->collider, a->pos, a->angle,
        b->collider, b->pos, b->angle
    );
}

CollisionResult world_2d_check_collision_rigid_body_static_body(Rigidbody2D* a, Staticbody2D* b) {
    return collision_2d_check(
        a->collider, a->pos, a->angle,
        b->collider, b->pos, b->angle
    );
}

CollisionResult world_2d_check_collision_static_body_static_body(Staticbody2D* a, Staticbody2D* b) {
    return collision_2d_check(
        a->collider, a->pos, a->angle,
        b->collider, b->pos, b->angle
    );
}




static Vector2D get_gravity_force(World2D* world, float mass) {
    return vec_2d_scale(vec_2d_normalize(world->gravity_dir),
                        mass * world->gravity_constant);
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

static void update_rigidbodies_values(World2D* world, float delta_time){
    for (int i = 0; i < world->count; i++) {
        PhysicsObject* obj = &world->objects[i];
        if (obj->type == RIGIDBODY) {
            rigid_body_2d_update(obj->rigidbody, delta_time);
        }
    }
}

static void apply_gravity(World2D* world) {
    for (int i = 0; i < world->count; i++) {
        PhysicsObject* obj = &world->objects[i];
        if (obj->type == RIGIDBODY) {
            Rigidbody2D* rb = obj->rigidbody;
            rb->force = vec_2d_add(rb->force, get_gravity_force(world, rb->mass));
        }
    }
}

static void handle_rigidbody_collision(Rigidbody2D* a, Rigidbody2D* b) {
    CollisionResult result = collision_2d_check(
        a->collider, a->pos, a->angle,
        b->collider, b->pos, b->angle
    );

    if (result.did_collide) {
        correct_overlapping(result, &a->pos, 1.0f / a->mass,
                                     &b->pos, 1.0f / b->mass);
        

         float vel_on_normal_axis = vec_2d_dot(a->vel, result.normal_vec);
        if (vel_on_normal_axis > 0.0f) {
            a->vel = vec_2d_sub(a->vel, vec_2d_scale(result.normal_vec, vel_on_normal_axis));
        }

        float force_along_normal_axis = vec_2d_dot(a->force, result.normal_vec);
        if (force_along_normal_axis > 0.0f) {
            a->force = vec_2d_sub(a->force, vec_2d_scale(result.normal_vec, force_along_normal_axis));
        }                             
        
    }

}

static void handle_static_collision(Rigidbody2D* a, Staticbody2D* b) {
    CollisionResult result = collision_2d_check(
        a->collider, a->pos, a->angle,
        b->collider, b->pos, b->angle
    );

    if (result.did_collide) {
        correct_overlapping(result, &a->pos, 1.0f / a->mass, NULL, 0.0f);

        float vel_on_normal_axis = vec_2d_dot(a->vel, result.normal_vec);
        if (vel_on_normal_axis > 0.0f) {
            a->vel = vec_2d_sub(a->vel, vec_2d_scale(result.normal_vec, vel_on_normal_axis));
        }

        float force_along_normal_axis = vec_2d_dot(a->force, result.normal_vec);
        if (force_along_normal_axis > 0.0f) {
            a->force = vec_2d_sub(a->force, vec_2d_scale(result.normal_vec, force_along_normal_axis));
        }

    }
}

void world_2d_step(World2D* world, float delta_time) {
    apply_gravity(world);

    update_rigidbodies_values(world, delta_time);

    for (int i = 0; i < world->count; i++) {
        PhysicsObject* a = world->objects + i;

        for (int j = 0; j < world->count; j++) {
            PhysicsObject* b = world->objects + j;

            if(a == b)
                continue;

            if (a->type == RIGIDBODY && b->type == RIGIDBODY) {
                handle_rigidbody_collision(a->rigidbody, b->rigidbody);
            } 
            else if (a->type == STATICBODY && b->type == RIGIDBODY) {
                handle_static_collision(b->rigidbody, a->staticbody);
            }
            else if (a->type == RIGIDBODY && b->type == STATICBODY) {
                handle_static_collision(a->rigidbody, b->staticbody);
            } 
            else if (a->type == STATICBODY && b->type == STATICBODY) {
            }
        }
    }
}

