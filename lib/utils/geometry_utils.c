#include "geometry_utils.h"
#include <math.h>
#define PI 3.14159265358979323846

Vector2D get_edge_normal_vec(Vector2D corner_a, Vector2D corner_b){
    Vector2D edge = vec_2d_sub(corner_a, corner_b);
    edge = vec_2d_rotate(edge, PI / 2.0);
    return vec_2d_normalize(edge);
}


void get_rect_corners(Vector2D pos, float angle, float width, float height, Vector2D out[4]) {
    float hw = width * 0.5f;
    float hh = height * 0.5f;

    Vector2D local[4] = {
        vec_2d(-hw, -hh),
        vec_2d( hw, -hh),
        vec_2d( hw,  hh),
        vec_2d(-hw,  hh)
    };

    for (int i = 0; i < 4; i++) {
        Vector2D rotated = vec_2d_rotate(local[i], angle);
        out[i] = vec_2d_add(rotated, pos);
    }
}
