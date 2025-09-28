#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "vector2.h"

/** get the normal vector that is perpendicular to an edge of a shape
 */
Vector2D get_edge_normal_vec(Vector2D corner_a, Vector2D corner_b);

// get the 4 corners of a rotated rectangle
// pos -center position of the rect
// angle -in radians
// out[4] - array to store the 4 corners
void get_rect_corners(Vector2D pos, float angle, float width, float height, Vector2D out[4]);

#endif