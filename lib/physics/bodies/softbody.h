#ifndef SOFTBODY_H
#define SOFTBODY_H

#include "rigidbody.h"
#include "spring.h"
#include "geometry_utils.h"


typedef struct Softbody2D {
    Rigidbody2D** points;
    int point_count;
    Spring2D** springs;
    int spring_count;

} Softbody2D;

void soft_body_init(Softbody2D* body, float mass, Vector2D* points, int points_amount, float point_radius, float spring_stiffness, float spring_damping);


#endif