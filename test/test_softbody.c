#include "raylib.h"
#include "world.h"
#include "bodies.h"
#include <stdlib.h>
#include <stdio.h>

#define MOVE_FORCE 100.0f
#define JUMP_FORCE  70000.0f

int main(void) {
    InitWindow(1200, 1000, "softbody test");

    World2D* world = world_2d_create(10);
    world_2d_set_gravity(world, 300, vec_2d(0, 1));

    Softbody2D* player = (Softbody2D*)malloc(sizeof(Softbody2D));

    Vector2D pts[] = {
        {300, 100}, {320, 100}, {340, 100},
        {300, 120}, {320, 120}, {340, 120},
        {300, 140}, {320, 140}, {340, 140}
    };

    soft_body_init(player, 1.0f, pts, 9, 5.0f, 1000.0f, 50.0f);
    world_2d_add_soft_body(world, player);

    Staticbody2D* ground = (Staticbody2D*)malloc(sizeof(Staticbody2D));
    static_body_2d_init(ground, vec_2d(600, 900), 0, rect_collider_create(1000, 40));
    world_2d_add_static_body(world, ground);

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        int substep_amount = 10;
        for (int i = 0; i < substep_amount; i++) {
            world_2d_step(world, dt / substep_amount);
        }

        BeginDrawing();
        ClearBackground(DARKGRAY);

        DrawText(TextFormat("FPS: %d", GetFPS()), 20, 20, 20, RAYWHITE);

        for (int i = 0; i < player->spring_count; i++) {
            Vector2D a = player->springs[i]->point_a->pos;
            Vector2D b = player->springs[i]->point_b->pos;
            DrawLineV((Vector2){a.x, a.y}, (Vector2){b.x, b.y}, LIGHTGRAY);
        }

        for (int i = 0; i < player->point_count; i++) {
            Vector2D p = player->points[i]->pos;
            DrawCircleV((Vector2){p.x, p.y}, 5.0f, RED);
        }

    

        DrawRectangle(100, 880, 1000, 40, WHITE);

        EndDrawing();
    }

    world_2d_destroy(world);
    CloseWindow();
    return 0;
}
