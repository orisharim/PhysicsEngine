#ifndef BODIES_H
#define BODIES_H

#include "rigidbody.h"
#include "staticbody.h"
#include "softbody.h"
#include "collision.h"


void bodies_handle_collision_rigid_vs_rigid(Rigidbody2D* a, Rigidbody2D* b);
// void bodies_handle_collision_soft_vs_soft(Softbody2D* a, Softbody2D* b);
void bodies_handle_collision_rigid_vs_static(Rigidbody2D* a, Staticbody2D* b);




#endif