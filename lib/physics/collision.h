#ifndef COLLISION_H
#define COLLISION_H

#include "vector2.h"
#include <stdbool.h>

typedef enum {
    RECT_COLLIDER,
    CIRCLE_COLLIDER,
    POLYGON_COLLIDER
} Collider2DType;

typedef struct RectCollider {
    float width;
    float height;
} RectCollider;

typedef struct CircleCollider {
    float radius;
} CircleCollider;

typedef struct PolygonCollider {
    Vector2D* vertices;
    int vertices_amount;
} PolygonCollider;

typedef struct CollisionResult {
    Vector2D normal_vec;
    float penetration;
    bool did_collide;
} CollisionResult; 

typedef struct Collider2D {

    Collider2DType type;
    union{
        CircleCollider circle;
        RectCollider rect;
        PolygonCollider polygon;
    };

} Collider2D;


CollisionResult collision_2d_check(Collider2D a_collider, Vector2D a_pos, float a_angle,
                                     Collider2D b_collider, Vector2D b_pos, float b_angle);
Collider2D rect_collider_create(float width, float height);
Collider2D circle_collider_create(float radius);
Collider2D polygon_collider_create(Vector2D* vertices, int vertices_amount);

#endif