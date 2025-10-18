#include "raylib.h"
#include "world.h"
#include <stdlib.h>
#include <stdio.h>

#define BALL_RADIUS  15.0f
#define SPAWN_COUNT  100
#define SPAWN_DELAY  0.1f
#define GRAVITY      600.0f

int main(void) {
    InitWindow(1200, 1000, "Ball Pool Test");

    World2D* world = world_2d_create(100);

    Staticbody2D* ground = (Staticbody2D*)malloc(sizeof(Staticbody2D));
    static_body_2d_init(ground, vec_2d(600, 950), 0, rect_collider_create(1200, 40));
    world_2d_add_static_body(world, ground);

    Staticbody2D* left_wall = (Staticbody2D*)malloc(sizeof(Staticbody2D));
    static_body_2d_init(left_wall, vec_2d(-10, 500), 0, rect_collider_create(40, 1000));
    world_2d_add_static_body(world, left_wall);

    Staticbody2D* right_wall = (Staticbody2D*)malloc(sizeof(Staticbody2D));
    static_body_2d_init(right_wall, vec_2d(1210, 500), 0, rect_collider_create(40, 1000));
    world_2d_add_static_body(world, right_wall);

    world_2d_set_gravity(world, GRAVITY, vec_2d(0, 1));

    Rigidbody2D* balls[SPAWN_COUNT] = {0};
    float spawn_timer = 0;
    int spawned = 0;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        spawn_timer += dt;

        if (spawned < SPAWN_COUNT && spawn_timer >= SPAWN_DELAY) {
            spawn_timer = 0;

            Rigidbody2D* ball = (Rigidbody2D*)malloc(sizeof(Rigidbody2D));
            rigid_body_2d_init(
                ball,
                1.0f,
                vec_2d(600 + (rand() % 100 - 50), 50),
                0,
                circle_collider_create(BALL_RADIUS)
            );
            rigid_body_2d_set_bounciness(ball, 1.0f);
            rigid_body_2d_set_friction(ball, 0.5f);
            world_2d_add_rigid_body(world, ball);

            balls[spawned] = ball;
            spawned++;
        }

        world_2d_step_with_substeps(world, dt, 10);

        BeginDrawing();
        ClearBackground(DARKGRAY);

        DrawText(TextFormat("FPS: %d", GetFPS()), 20, 20, 20, WHITE);

        DrawRectangle(ground->pos.x - 600, ground->pos.y - 20, 1200, 40, WHITE);
        DrawRectangle(left_wall->pos.x - 20, left_wall->pos.y - 500, 40, 1000, WHITE);
        DrawRectangle(right_wall->pos.x - 20, right_wall->pos.y - 500, 40, 1000, WHITE);

        for (int i = 0; i < spawned; i++) {
            if (balls[i]) {
                DrawCircle(
                    (int)balls[i]->pos.x,
                    (int)balls[i]->pos.y,
                    BALL_RADIUS,
                    RED
                );
            }
        }

        EndDrawing();
    }

    world_2d_destroy(world);
    CloseWindow();
    return 0;
}
