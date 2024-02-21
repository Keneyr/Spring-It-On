#include "common.h"

//--------------------------------------
/*
* it's a simple damper, the goal velocity q is zero, 
* which means the acceleration of the character is zero
*/
void spring_character_update(
    float& x, 
    float& v, 
    float& a, 
    float v_goal, 
    float halflife, 
    float dt)
{
    float y = halflife_to_damping(halflife) / 2.0f;	
    float j0 = v - v_goal;
    float j1 = a + j0*y;
    float eydt = fast_negexp(y*dt);

    //get the character postion: integrate velocity with respect to time
    x = eydt*(((-j1)/(y*y)) + ((-j0 - j1*dt)/y)) + 
        (j1/(y*y)) + j0/y + v_goal * dt + x;
    v = eydt*(j0 + j1*dt) + v_goal;
    a = eydt*(a - j1*y*dt);
}
/*
* If we want to predict the future trajectory we can use simple spring to update arrays of data each with a different dt
*/
void spring_character_predict(
    float px[], 
    float pv[], 
    float pa[], 
    int count,
    float x, 
    float v, 
    float a, 
    float v_goal, 
    float halflife,
    float dt)
{
    for (int i = 0; i < count; i++)
    {
        px[i] = x; 
        pv[i] = v; 
        pa[i] = a;
    }

    for (int i = 0; i < count; i++)
    {
        spring_character_update(px[i], pv[i], pa[i], v_goal, halflife, i * dt);
    }
}

//--------------------------------------

enum
{
    TRAJ_MAX = 32,
    TRAJ_SUB = 4,
    PRED_MAX = 4,
    PRED_SUB = 4,
};

float trajx_prev[TRAJ_MAX];
float trajy_prev[TRAJ_MAX];

float predx[PRED_MAX], predy[PRED_MAX];
float predxv[PRED_MAX], predyv[PRED_MAX];
float predxa[PRED_MAX], predya[PRED_MAX];

int main(void)
{
    // Init Window
    
    const int screenWidth = 2048;
    const int screenHeight = 1024;

    InitWindow(screenWidth, screenHeight, "raylib [springs] example - controller");

    // Init Variables

    float halflife = 0.1f;
    float dt = 1.0 / 60.0f;
    float timescale = 240.0f;
    
    SetTargetFPS(1.0f / dt);

    // Trajectory
    
    float trajx = screenWidth / 2.0f;
    float trajy = screenHeight / 2.0f;
    float trajxv = 0.0, trajyv = 0.0;
    float trajxa = 0.0, trajya = 0.0;
    float traj_xv_goal = 0.0;
    float traj_yv_goal = 0.0;
    
    for (int i = 0; i < TRAJ_MAX; i++)
    {
        trajx_prev[i] = screenWidth / 2.0f;
        trajy_prev[i] = screenHeight / 2.0f;
    }
    
    while (!WindowShouldClose())
    {
        // Shift History, like a sliding window, the front() is always the latest
        
        for (int i = TRAJ_MAX - 1; i > 0; i--)
        {
            trajx_prev[i] = trajx_prev[i - 1];
            trajy_prev[i] = trajy_prev[i - 1];
        }
        
        // Controller
        
        GuiSliderBar((Rectangle){ 100, 20, 120, 20 }, "halflife", TextFormat("%5.3f", halflife), &halflife, 0.0f, 1.0f);

        // Update Spring
        
        /*float gamepadx = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
        float gamepady = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
        float gamepadmag = sqrtf(gamepadx*gamepadx + gamepady*gamepady);
        
        if (gamepadmag > 0.2f)
        {
            float gamepaddirx = gamepadx / gamepadmag;
            float gamepaddiry = gamepady / gamepadmag;
            float gamepadclippedmag = gamepadmag > 1.0f ? 1.0f : gamepadmag*gamepadmag;
            gamepadx = gamepaddirx * gamepadclippedmag;
            gamepady = gamepaddiry * gamepadclippedmag;
        }
        else
        {
            gamepadx = 0.0f;
            gamepady = 0.0f;
        }
        */
        float gamepadx = 0.0f;
        float gamepady = 0.0f;
        //vec3 pad(0, 0, 0); 
        float am = 1.f;
        if (IsKeyDown(KEY_RIGHT_SHIFT) || IsKeyDown(KEY_LEFT_SHIFT)) am *= 0.5f;  // use SHIFT to slow down/walk
        if (IsKeyDown(KEY_RIGHT_CONTROL) || IsKeyDown(KEY_LEFT_CONTROL)) am *= 2.f; // use CTRL to speed up
        // use ARROW keys to move the character
        if (IsKeyDown(KEY_RIGHT)) gamepadx += am;
        if (IsKeyDown(KEY_LEFT)) gamepadx -= am;
        if (IsKeyDown(KEY_UP)) gamepady -= am;
        if (IsKeyDown(KEY_DOWN)) gamepady += am;
        
        //the signal of gamepad or keyboard changes the speed of this character
        traj_xv_goal = 250.0f * gamepadx;
        traj_yv_goal = 250.0f * gamepady;
        
        spring_character_update(trajx, trajxv, trajxa, traj_xv_goal, halflife, dt);
        spring_character_update(trajy, trajyv, trajya, traj_yv_goal, halflife, dt);
        
        spring_character_predict(predx, predxv, predxa, PRED_MAX, trajx, trajxv, trajxa, traj_xv_goal, halflife, dt * PRED_SUB);
        spring_character_predict(predy, predyv, predya, PRED_MAX, trajy, trajyv, trajya, traj_yv_goal, halflife, dt * PRED_SUB);
        
        trajx_prev[0] = trajx;
        trajy_prev[0] = trajy;

        BeginDrawing();
        
            ClearBackground(RAYWHITE);
            
            for (int i = 0; i < TRAJ_MAX - TRAJ_SUB; i += TRAJ_SUB)
            {
                Vector2 start = {trajx_prev[i + 0], trajy_prev[i + 0]};
                Vector2 stop  = {trajx_prev[i + TRAJ_SUB], trajy_prev[i + TRAJ_SUB]};
                
                DrawLineV(start, stop, BLUE);                
                DrawCircleV(start, 3, BLUE);                
            }
            //predicted position of character
            for (int i = 1; i < PRED_MAX; i ++)
            {
                Vector2 start = {predx[i + 0], predy[i + 0]};
                Vector2 stop  = {predx[i - 1], predy[i - 1]};
                
                DrawLineV(start, stop, MAROON);                
                DrawCircleV(start, 3, MAROON);                
            }
            
            //current frame position of character
            DrawCircleV((Vector2){trajx, trajy}, 4, DARKBLUE);                
            
            Vector2 gamepadPosition = {60, 300};
            Vector2 gamepadStickPosition = {gamepadPosition.x + gamepadx * 25, gamepadPosition.y + gamepady * 25};
            DrawCircleLines(gamepadPosition.x, gamepadPosition.y, 25, DARKPURPLE);
            
            DrawCircleV(gamepadPosition, 3, DARKPURPLE);                
            DrawCircleV(gamepadStickPosition, 3, DARKPURPLE);                
            DrawLineV(gamepadPosition, gamepadStickPosition, DARKPURPLE);
            
        EndDrawing();
        
    }

    CloseWindow();

    return 0;
}