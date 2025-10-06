#include "spring.h"

void spring_2d_init(Spring2D* spring, Rigidbody2D* point_a, Rigidbody2D* point_b, float stiffness, float rest_length, float damp_factor){
    spring->point_a = point_a;
    spring->point_b = point_b;
    spring->stiffness = stiffness;
    spring->rest_length = rest_length;
    spring->damp_factor = damp_factor;
}

static Vector2D get_spring_applied_force(Spring2D spring){
    Vector2D distance = vec_2d_sub(spring.point_b->pos, spring.point_a->pos);
    Vector2D spring_force_dir = vec_2d_normalize(distance);

    float spring_force_mag = -1.0f * (vec_2d_length(distance) - spring.rest_length) * spring.stiffness ; //hook's law
    Vector2D spring_force = vec_2d_scale(spring_force_dir, spring_force_mag); 

    Vector2D relative_velocity = vec_2d_sub(spring.point_b->vel, spring.point_a->vel);
    float damping_force_mag = -1.0f * vec_2d_dot(relative_velocity, spring_force_dir) * spring.damp_factor;
    Vector2D damping_force = vec_2d_scale(spring_force_dir, damping_force_mag);
    return vec_2d_add(spring_force, damping_force);
}

void spring_2d_update(Spring2D* spring){
    Vector2D spring_force = get_spring_applied_force(*spring);
    rigid_body_2d_add_force(spring->point_a, spring_force);
    rigid_body_2d_add_force(spring->point_a, vec_2d_scale(spring_force, -1.0f));
}



