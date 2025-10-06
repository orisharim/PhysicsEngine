#include "staticbody.h"

void static_body_2d_init(Staticbody2D* body, Vector2D pos, float angle,  Collider2D collider){
    body->pos = pos;
    body->angle = angle;
    body->collider = collider;
}

void static_body_2d_set_bounciness(Staticbody2D* body, float bounciness){
    body->bounciness = bounciness;
}

void static_body_2d_set_friction(Staticbody2D* body, float friction){
    body->friction = friction;
}

