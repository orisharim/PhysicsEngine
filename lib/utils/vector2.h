#ifndef VECTOR2D_H
#define VECTOR2D_H

typedef struct {
    float x;
    float y;
} Vector2D;

Vector2D vec_2d(float x, float y);
Vector2D vec_2d_add(Vector2D a, Vector2D b);
Vector2D vec_2d_sub(Vector2D a, Vector2D b);
Vector2D vec_2d_scale(Vector2D v, float s);
float vec_2d_dot(Vector2D a, Vector2D b);
float vec_2d_cross(Vector2D a, Vector2D b);
float vec_2d_length(Vector2D v);
Vector2D vec_2d_normalize(Vector2D v);
Vector2D vec_2d_rotate(Vector2D v, float radians);
Vector2D vec_2d_lerp(Vector2D a, Vector2D b, float t);

#endif 



