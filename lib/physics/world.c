#include <stdlib.h>
#include <math.h>
#include "world.h"
#include "bodies.h"
#include "collision.h"
#include "geometry_utils.h"


World2D* world_2d_create(int initial_capacity) {
    World2D* world = malloc(sizeof(World2D));
    if (!world) 
        return NULL;

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

void world_2d_step_with_substeps(World2D* world, float delta_time, int substep_amount){
    for(int i = 0; i < substep_amount; i++){
        world_2d_step(world, delta_time / substep_amount);
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
            if (a->type == RIGIDBODY && b->type == RIGIDBODY) 
                rigid_body_2d_handle_collision(a->rigidbody, b->rigidbody); 
            else if (a->type == STATICBODY && b->type == RIGIDBODY) 
                rigid_body_2d_handle_static_collision(b->rigidbody, a->staticbody);
            else if (a->type == RIGIDBODY && b->type == STATICBODY) 
                rigid_body_2d_handle_static_collision(a->rigidbody, b->staticbody); 
            else if (a->type == STATICBODY && b->type == STATICBODY) 
                continue;
        }
    }

}



