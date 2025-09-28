#include "collision.h"
#include "vector2.h"
#include "bodies.h"
#include "geometry_utils.h"
#include "float.h"
#include <stdio.h>
#include <math.h>

Collider2D rect_collider_create(float width, float height){
    RectCollider square_collider = (RectCollider) {width, height };
    Collider2D collider;
    collider.rect = square_collider;
    collider.type = RECT_COLLIDER;
    return collider;
}

Collider2D circle_collider_create(float radius){
    CircleCollider circle_collider = (CircleCollider) {radius };
    Collider2D collider;
    collider.circle = circle_collider;
    collider.type = CIRCLE_COLLIDER;
    return collider;
}

CollisionResult collision_check_circles(CircleCollider a_collider, Vector2D a_pos,
                                     CircleCollider b_collider, Vector2D b_pos) {
    CollisionResult result;
    result.normal_vec = vec_2d(0, 0);
    result.did_collide = false;
    result.penetration = 0;

    float radius_a = a_collider.radius;
    float radius_b = b_collider.radius;

    Vector2D dis = vec_2d_sub(a_pos, b_pos);
    float dist = vec_2d_length(dis);

    result.penetration = radius_a + radius_b - dist;

    if(result.penetration > 0){
        result.did_collide = true;

        if(dist == 0.0f){
            //if circles are in the same pos apply force in a random direction
            result.normal_vec = vec_2d(1, 0);
        } else {
            result.normal_vec = vec_2d_normalize(dis);
        }

    }

    return result;
}

// project the corners of a shape on an axis and return the min and max values 
void project_shape_corners_on_axis(Vector2D* corners, int count, Vector2D axis, float* out_min, float* out_max) {
    float min = vec_2d_dot(axis, corners[0]);
    float max = min;

    for(int i = 1; i < count; i++) {
        float projected_corner = vec_2d_dot(axis, corners[i]);
        if (projected_corner < min) 
            min = projected_corner;
        if(projected_corner > max)
            max = projected_corner;
    }

    *out_min = min;
    *out_max = max;
}

CollisionResult collision_check_rects(RectCollider a_collider, Vector2D a_pos, float a_angle,
                                     RectCollider b_collider, Vector2D b_pos, float b_angle) {
    CollisionResult result;
    result.normal_vec = vec_2d(0, 0);
    result.did_collide = false;
    result.penetration = 0;
    
    Vector2D corners_a[4];
    Vector2D corners_b[4];
    
    get_rect_corners(a_pos, a_angle, a_collider.width, a_collider.height, corners_a);
    get_rect_corners(b_pos, b_angle, b_collider.width, b_collider.height, corners_b);

    Vector2D edge_vecs[4]; 
    edge_vecs[0] = get_edge_normal_vec(corners_a[0], corners_a[1]);
    edge_vecs[1] = get_edge_normal_vec(corners_a[1], corners_a[2]);
    edge_vecs[2] = get_edge_normal_vec(corners_b[0], corners_b[1]);
    edge_vecs[3] = get_edge_normal_vec(corners_b[1], corners_b[2]);

    float min_overlap = FLT_MAX;
    Vector2D smallest_axis = vec_2d(0, 0);

    for(int i = 0; i < 4; i++){
        
        float min_a;
        float max_a;
        project_shape_corners_on_axis(corners_a, 4, edge_vecs[i], &min_a, &max_a);

        float min_b;
        float max_b;
        project_shape_corners_on_axis(corners_b, 4, edge_vecs[i], &min_b, &max_b);


        if(max_a < min_b || max_b < min_a){ //checks if there is a gap theres no collision
            return result;
        }  

        float overlap = (max_a < max_b ? max_a : max_b) - (min_a > min_b ? min_a : min_b);
        if (overlap < min_overlap) {
            min_overlap = overlap;
            smallest_axis = edge_vecs[i];
        }
    }

    result.did_collide = true;
    result.penetration = min_overlap;

    Vector2D center_dis = vec_2d_sub(b_pos, a_pos);
    if (vec_2d_dot(center_dis, smallest_axis) < 0) {
        smallest_axis = vec_2d_scale(smallest_axis, -1);
    }

    result.normal_vec = vec_2d_normalize(smallest_axis);
    return result;

}

CollisionResult collision_check_rect_circle(RectCollider rect, Vector2D rect_pos, float rect_angle,
                                            CircleCollider circle, Vector2D circle_pos) {
    CollisionResult result;
    result.did_collide = false;
    result.normal_vec = vec_2d(0, 0);
    result.penetration = 0;

    Vector2D corners[4];
    get_rect_corners(rect_pos, rect_angle, rect.width, rect.height, corners);

    Vector2D axis_x = vec_2d_normalize(vec_2d_sub(corners[1], corners[0]));
    Vector2D axis_y = vec_2d_normalize(vec_2d_sub(corners[3], corners[0]));

    Vector2D rel = vec_2d_sub(circle_pos, rect_pos);

    float half_w = rect.width * 0.5f;
    float half_h = rect.height * 0.5f;

    float proj_x = vec_2d_dot(rel, axis_x);
    float proj_y = vec_2d_dot(rel, axis_y);

    float clamped_x = fmaxf(-half_w, fminf(proj_x, half_w));
    float clamped_y = fmaxf(-half_h, fminf(proj_y, half_h));

    Vector2D closest = vec_2d_add(rect_pos,
                                  vec_2d_add(vec_2d_scale(axis_x, clamped_x),
                                             vec_2d_scale(axis_y, clamped_y)));

    Vector2D diff = vec_2d_sub(circle_pos, closest);
    float dist = vec_2d_length(diff);

    if (dist < circle.radius) {
        result.did_collide = true;
        result.penetration = circle.radius - dist;

        if (dist == 0) {
            result.normal_vec = vec_2d(1, 0);
        } else {
            result.normal_vec = vec_2d_scale(diff, 1.0f / dist);
        }
    }

    return result;
}

CollisionResult collision_2d_check(Collider2D a_collider, Vector2D a_pos, float a_angle,
                                     Collider2D b_collider, Vector2D b_pos, float b_angle){
    CollisionResult res;
    if(a_collider.type == CIRCLE_COLLIDER && b_collider.type == CIRCLE_COLLIDER){
        res = collision_check_circles(a_collider.circle, a_pos, b_collider.circle, b_pos);
    } 
    else if(a_collider.type == RECT_COLLIDER && b_collider.type == RECT_COLLIDER){
        res = collision_check_rects(a_collider.rect, a_pos, a_angle, b_collider.rect, b_pos, b_angle);
    }
    else if(a_collider.type == RECT_COLLIDER && b_collider.type == CIRCLE_COLLIDER){
        res = collision_check_rect_circle(a_collider.rect, a_pos, a_angle, b_collider.circle, b_pos);
    }
    else if(a_collider.type == CIRCLE_COLLIDER && b_collider.type == RECT_COLLIDER){
        res = collision_check_rect_circle(b_collider.rect, b_pos, b_angle, a_collider.circle, a_pos);
        res.normal_vec = vec_2d_scale(res.normal_vec, -1.0f);
    }
    return res;
}
