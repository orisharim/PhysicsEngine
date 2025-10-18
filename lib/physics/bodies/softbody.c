#include "softbody.h"
#include "rigidbody.h"
#include "spring.h"
#include "geometry_utils.h"
#include <stdlib.h>
#include <float.h>
#include <math.h>

void soft_body_init(Softbody2D* body, float mass, Vector2D* points, int points_amount, float point_radius, float spring_stiffness, float spring_damping) {
    body->point_count = points_amount;
    body->points = malloc(points_amount * sizeof(Rigidbody2D*));

    for (int i = 0; i < points_amount; i++) {
        body->points[i] = malloc(sizeof(Rigidbody2D));
        Collider2D col = circle_collider_create(point_radius);
        rigid_body_2d_init(body->points[i], mass, points[i], 0, col);
    }

    int grid_size = (int)roundf(sqrtf(points_amount));

    int max_springs = grid_size * grid_size * 4;
    body->springs = malloc(max_springs * sizeof(Spring2D*));
    body->spring_count = 0;

    

    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            int i = y * grid_size + x;
            if (x < grid_size - 1) {
                int j = y * grid_size + (x + 1);
                float rest = vec_2d_length(vec_2d_sub(points[i], points[j]));
                body->springs[body->spring_count] = malloc(sizeof(Spring2D));
                spring_2d_init(body->springs[body->spring_count++], body->points[i], body->points[j], spring_stiffness, rest, spring_damping);
            }
            if (y < grid_size - 1) { 
                int j = (y + 1) * grid_size + x;
                float rest = vec_2d_length(vec_2d_sub(points[i], points[j]));
                body->springs[body->spring_count] = malloc(sizeof(Spring2D));
                spring_2d_init(body->springs[body->spring_count++], body->points[i], body->points[j], spring_stiffness, rest, spring_damping);
            }
            if (x < grid_size - 1 && y < grid_size - 1) {
                int j = (y + 1) * grid_size + (x + 1);
                float rest = vec_2d_length(vec_2d_sub(points[i], points[j]));
                body->springs[body->spring_count] = malloc(sizeof(Spring2D));
                spring_2d_init(body->springs[body->spring_count++], body->points[i], body->points[j], spring_stiffness, rest, spring_damping);
            }
            if (x > 0 && y < grid_size - 1) { 
                int j = (y + 1) * grid_size + (x - 1);
                float rest = vec_2d_length(vec_2d_sub(points[i], points[j]));
                body->springs[body->spring_count] = malloc(sizeof(Spring2D));
                spring_2d_init(body->springs[body->spring_count++], body->points[i], body->points[j], spring_stiffness, rest, spring_damping);
            }
        }
    }
}
