#include "raylib.h"
#include "world.h"
#include "bodies.h"
#include <stdlib.h>
#include <stdio.h>

#define CUBE_SIZE    30.0f
#define SPAWN_COUNT  50
#define SPAWN_DELAY  0.1f
#define GRAVITY      300.0f

int main(void) {
    InitWindow(1200, 1000, "Cube Pool Test");

    World2D* world = world_2d_create(100);

    // Create ground
    Staticbody2D* ground = (Staticbody2D*)malloc(sizeof(Staticbody2D));
    static_body_2d_init(ground, vec_2d(600, 950), 0, rect_collider_create(1200, 40));
    world_2d_add_static_body(world, ground);

    world_2d_set_gravity(world, GRAVITY, vec_2d(0, 1));

    // Cube array
    Rigidbody2D* cubes[SPAWN_COUNT] = {0};
    float spawn_timer = 0;
    int spawned = 0;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        spawn_timer += dt;

        // Spawn cubes over time
        if (spawned < SPAWN_COUNT && spawn_timer >= SPAWN_DELAY) {
            spawn_timer = 0;

            Rigidbody2D* cube = (Rigidbody2D*)malloc(sizeof(Rigidbody2D));
            rigid_body_2d_init(
                cube,
                1.0f,
                vec_2d(600 + (rand() % 100 - 50), 50),
                0,
                rect_collider_create(CUBE_SIZE, CUBE_SIZE)
            );
            rigid_body_2d_set_material(cube, (Material2D){0.2f, 0.2f}); // bounciness and friction
            world_2d_add_rigid_body(world, cube);

            cubes[spawned] = cube;
            spawned++;
        }

        float substep_amount = 10.0f;
        for (int i = 0; i < substep_amount; i++) {
            world_2d_step(world, dt / substep_amount);
        }

        BeginDrawing();
        ClearBackground(DARKGRAY);

        DrawText(TextFormat("FPS: %d", GetFPS()), 20, 20, 20, WHITE);

        // Draw ground
        DrawRectangle(ground->pos.x - 600, ground->pos.y - 20, 1200, 40, WHITE);

        // Draw cubes
        for (int i = 0; i < spawned; i++) {
            if (cubes[i]) {
                DrawRectangle(
                    cubes[i]->pos.x - CUBE_SIZE/2,
                    cubes[i]->pos.y - CUBE_SIZE/2,
                    CUBE_SIZE,
                    CUBE_SIZE,
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
