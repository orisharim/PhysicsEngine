#ifndef STATICBODY_H
#define STATICBODY_H

#include "collision.h"
#include "vector2.h"

typedef struct Staticbody2D{
    Vector2D pos;
    float angle;    //radians

    Collider2D collider;

    float bounciness;
    float friction;
} Staticbody2D;

void static_body_2d_init(Staticbody2D* body, Vector2D pos, float angle,  Collider2D collider);
void static_body_2d_set_bounciness(Staticbody2D* body, float bounciness);
void static_body_2d_set_friction(Staticbody2D* body, float friction);

#endif