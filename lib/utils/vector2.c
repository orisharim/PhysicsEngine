#include <math.h>
#include "vector2.h"

Vector2D vec_2d(float x, float y) {
    Vector2D v = {x, y};
    return v;
}

Vector2D vec_2d_add(Vector2D a, Vector2D b) {
    return vec_2d(a.x + b.x, a.y + b.y);
}

Vector2D vec_2d_sub(Vector2D a, Vector2D b) {
    return vec_2d(a.x - b.x, a.y - b.y);
}

Vector2D vec_2d_scale(Vector2D v, float s) {
    return vec_2d(v.x * s, v.y * s);
}

float vec_2d_dot(Vector2D a, Vector2D b) {
    return a.x * b.x + a.y * b.y;
}

float vec_2d_cross(Vector2D a, Vector2D b) {
    return a.x * b.y - a.y * b.x;
}

float vec_2d_length(Vector2D v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

Vector2D vec_2d_normalize(Vector2D v) {
    float len = vec_2d_length(v);
    if (len == 0.0f) return vec_2d(0, 0);
    return vec_2d(v.x / len, v.y / len);
}

Vector2D vec_2d_rotate(Vector2D v, float radians) {
    float c = cosf(radians);
    float s = sinf(radians);
    return vec_2d(v.x * c - v.y * s, v.x * s + v.y * c);
}

Vector2D vec_2d_lerp(Vector2D a, Vector2D b, float t) {
    return vec_2d(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
}
