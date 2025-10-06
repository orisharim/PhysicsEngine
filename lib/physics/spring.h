#ifndef SPRING_H
#define SPRING_H

#include "rigidbody.h"
#include "vector2.h"

typedef struct Spring2D{
    Rigidbody2D* point_a;
    Rigidbody2D* point_b;
    float stiffness;
    float rest_length;
    float damp_factor;
} Spring2D;

void spring_2d_init(Spring2D* spring, Rigidbody2D* point_a, Rigidbody2D* point_b, float stiffness, float rest_length, float damp_factor);
void spring_2d_update(Spring2D* spring);

#endif