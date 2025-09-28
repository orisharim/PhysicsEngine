#include "raylib.h"
#include "world.h"
#include "bodies.h"
#include <stdlib.h>
#include <stdio.h>

#define MOVE_FORCE 100.0f   
#define JUMP_FORCE  70000.0f

int main(void) {
    InitWindow(1200, 1000, "");

    World2D* world = world_2d_create(10);

    Rigidbody2D* player = (Rigidbody2D*)malloc(sizeof(Rigidbody2D));
    rigid_body_2d_init(player, 1.0f, vec_2d(200, 200), 0, rect_collider_create(50, 50));
    rigid_body_2d_set_material(player, (Material2D){1, 0});
    world_2d_add_rigid_body(world, player);

    Staticbody2D* ground = (Staticbody2D*)malloc(sizeof(Staticbody2D));
    static_body_2d_init(ground, vec_2d(600, 400), 0, rect_collider_create(1000, 40));
    world_2d_add_static_body(world, ground);

    world_2d_set_gravity(world, 300, vec_2d(0, 1));

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        
        DrawText(TextFormat("FPS: %d", GetFPS()), 20, 20, 20, RAYWHITE);
        DrawText(TextFormat("Pos: %.2f, %.2f", player->pos.x, player->pos.y), 20, 60, 20, GREEN);
        DrawText(TextFormat("Vel: %.2f, %.2f", player->vel.x, player->vel.y), 20, 90, 20, GREEN);
        

        if (IsKeyDown(KEY_A)) {
            rigid_body_2d_set_velocity(player, vec_2d(-MOVE_FORCE, player->vel.y));
        }
        else if (IsKeyDown(KEY_D)) {
            rigid_body_2d_set_velocity(player, vec_2d(MOVE_FORCE, player->vel.y));
        } else {
            rigid_body_2d_set_velocity(player, vec_2d(0, player->vel.y));
        }

        CollisionResult res = world_2d_check_collision_rigid_body_static_body(player, ground);
        if (IsKeyPressed(KEY_SPACE) && res.normal_vec.y > 0.0) {
            rigid_body_2d_add_force(player, vec_2d(0, -JUMP_FORCE));
        } else {
        }

        int substep_amount = 10;
        for (int i = 0; i < substep_amount; i++) {
            world_2d_step(world, dt / substep_amount);
        }

        BeginDrawing();
        ClearBackground(DARKGRAY);



        // ground
        DrawRectangle(ground->pos.x - 500, ground->pos.y - 20, 1000, 40, WHITE);
         // player
        DrawRectangle(player->pos.x - 25, player->pos.y - 25, 50, 50, RED);

        
        EndDrawing();
    }
    world_2d_destroy(world);
    CloseWindow();

    return 0;
}
