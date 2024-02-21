extern "C"
{
#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
}

#include <string.h>

//--------------------------------------

float lerp(float x, float y, float a)
{
    return (1.0f - a) * x + a * y;
}

float clamp(float x, float minimum, float maximum)
{
    return x > maximum ? maximum : x < minimum ? minimum : x;
}

float max(float x, float y)
{
    return x > y ? x : y;
}

float min(float x, float y)
{
    return x < y ? x : y;
}

//--------------------------------------

float damper(float x, float g, float factor)
{
    return lerp(x, g, factor);
}

float damper_bad(float x, float g, float damping, float dt)
{
    return lerp(x, g, damping * dt);
}

/*
float damper_exponential(
    float x, 
    float g, 
    float damping, 
    float dt, 
    float ft = 1.0f / 60.0f)
{
    return lerp(x, g, 1.0f - powf(1.0 - ft * damping, dt / ft));
}
*/

/*
* y = 1.0 - ft * damping
* lerp(x_t, g, 1 - (1/y)^(-n)), n = dt / ft:
* rate of decay, and it dictates what proportion of the distance toward the goal will remain after ft in time.
*/
float damper_exponential(
    float x, 
    float g, 
    float damping, 
    float dt, 
    float ft = 1.0f / 60.0f)
{
    return lerp(x, g, 1.0f - powf(1.0 / (1.0 - ft * damping), -dt / ft));
} 


/*
* To simplify, let the rate of decay to 0.5 and 
* instead scale the timestep such that we can control the exact half-life of the damper
* lerp(x_t, g, 1 - (1/0.5)^(-dt/halflife)
*/
float damper_exact_0(float x, float g, float halflife, float dt)
{
    return lerp(x, g, 1.0f - powf(2, -dt / halflife));
}

//add some small epsilon value like 1e-5f to avoid division by zero when our halflife is very small
float damper_exact_1(float x, float g, float halflife, float dt, float eps=1e-5f)
{
    return lerp(x, g, 1.0f - expf(-(0.69314718056f * dt) / (halflife + eps)));
}

/*
* a fast approximation of the negative exponent function using one over a simple polynomial
*/
float fast_negexp(float x)
{
    return 1.0f / (1.0f + x + 0.48f*x*x + 0.235f*x*x*x);
}

float damper_exact(float x, float g, float halflife, float dt, float eps=1e-5f)
{
    return lerp(x, g, 1.0f - fast_negexp((0.69314718056f * dt) / (halflife + eps)));
}

float damper_decay_exact(float x, float halflife, float dt, float eps=1e-5f)
{
    return x * fast_negexp((0.69314718056f * dt) / (halflife + eps));
}

//--------------------------------------

void spring_damper_bad(
    float& x,
    float& v, 
    float g,
    float q, 
    float stiffness, 
    float damping, 
    float dt)
{
    v += dt * stiffness * (g - x) + dt * damping * (q - v);
    x += dt * v;
}

float fast_atan(float x)
{
    float z = fabs(x);
    float w = z > 1.0f ? 1.0f / z : z;
    float y = (PI / 4.0f)*w - w*(w - 1)*(0.2447f + 0.0663f*w);
    return copysign(z > 1.0f ? PI / 2.0 - y : y, x);
}

float squaref(float x)
{
    return x*x;
}

/*
* the movement of spring is a bit like a cos or sin function, so try to come up with a equation, such as
* x_t = j * e^(-y*t) * cos(w * t + p) + c
*
* since, we simulate a spring damper in a PD Control way (semi-implicit euler integration)before, such as
* v_(t+dt) = v_t + dt * stiffness * (g - x_t) + dt * damping * (q - v_t)
* x_(t+dt) = x_t + dt * v_t
*
* thus,
* we get
* 
* c = g + d * q / s
* y = d / 2
* w = sqrt(s - d^2 / 4)
* j = sqrt( (v + y * (x - c))^2 / w^2 + (x-c)^2)
* p = arctan((v + (x - c) * y) / (-(x - c) * w)
*/
void spring_damper_exact_0(
    float& x, 
    float& v, 
    float x_goal, 
    float v_goal, 
    float stiffness, 
    float damping, 
    float dt, 
    float eps = 1e-5f)
{
    float g = x_goal;
    float q = v_goal;
    float s = stiffness;
    float d = damping;
    float c = g + (d*q) / (s + eps);
    float y = d / 2.0f;
    float w = sqrtf(s - (d*d)/4.0f);
    float j = sqrtf(squaref(v + y*(x - c)) / (w*w + eps) + squaref(x - c));
    float p = fast_atan((v + (x - c) * y) / (-(x - c)*w + eps));

    j = (x - c) > 0.0f ? j : -j;

    float eydt = fast_negexp(y*dt);

    x = j*eydt*cosf(w*dt + p) + c;
    v = -y*j*eydt*cosf(w*dt + p) - w*j*eydt*sinf(w*dt + p);
}


void spring_damper_exact_stiffness_damping(
    float& x, 
    float& v, 
    float x_goal, 
    float v_goal, 
    float stiffness, 
    float damping, 
    float dt, 
    float eps = 1e-5f)
{
    float g = x_goal;
    float q = v_goal;
    float s = stiffness;
    float d = damping;
    float c = g + (d*q) / (s + eps);
    float y = d / 2.0f; 
    //it means the spring is critically damped, meaning it returns to the goal as fast as possible without extra oscillation
    if (fabs(s - (d*d) / 4.0f) < eps) // Critically Damped
    {
        float j0 = x - c;
        float j1 = v + j0*y;
        
        float eydt = fast_negexp(y*dt);
        
        x =  j0*eydt + dt*j1*eydt + c;
        v = -y*j0*eydt - y*dt*j1*eydt + j1*eydt;
    }
    //causing oscillations to appear with motions governed by the equations we already derived
    else if (s - (d*d) / 4.0f > 0.0) // Under Damped
    {
        float w = sqrtf(s - (d*d)/4.0f);
        float j = sqrtf(squaref(v + y*(x - c)) / (w*w + eps) + squaref(x - c));
        float p = fast_atan((v + (x - c) * y) / (-(x - c)*w + eps));
        
        j = (x - c) > 0.0f ? j : -j;
        
        float eydt = fast_negexp(y*dt);
        
        x = j*eydt*cosf(w*dt + p) + c;
        v = -y*j*eydt*cosf(w*dt + p) - w*j*eydt*sinf(w*dt + p);
    }
    //it means the spring is over damped, and will return slowly toward the goal.
    else if (s - (d*d) / 4.0f < 0.0) // Over Damped
    {
        float y0 = (d + sqrtf(d*d - 4*s)) / 2.0f;
        float y1 = (d - sqrtf(d*d - 4*s)) / 2.0f;
        float j1 = (c*y0 - x*y0 - v) / (y1 - y0);
        float j0 = x - j1 - c;
        
        float ey0dt = fast_negexp(y0*dt);
        float ey1dt = fast_negexp(y1*dt);

        x =  j0*ey0dt + j1*ey1dt + c;
        v = -y0*j0*ey0dt - y1*j1*ey1dt;
    }
}

float halflife_to_damping(float halflife, float eps = 1e-5f)
{
    return (4.0f * 0.69314718056f) / (halflife + eps);
}
    
float damping_to_halflife(float damping, float eps = 1e-5f)
{
    return (4.0f * 0.69314718056f) / (damping + eps);
}

float frequency_to_stiffness(float frequency)
{
   return squaref(2.0f * PI * frequency);
}

float stiffness_to_frequency(float stiffness)
{
    return sqrtf(stiffness) / (2.0f * PI);
}

float critical_halflife(float frequency)
{
    return damping_to_halflife(sqrtf(frequency_to_stiffness(frequency) * 4.0f));
}

float critical_frequency(float halflife)
{
    return stiffness_to_frequency(squaref(halflife_to_damping(halflife)) / 4.0f);
}
/*
* More intuitive, 
* the velocity continuity and oscillations of the spring means that 
* the position will not be exactly half way toward the goal in halflife time, 
* while the frequency parameter is more like a pseudo-frequency as the rate of oscillation is also affected by the damping value
*/
void spring_damper_exact(
    float& x, 
    float& v, 
    float x_goal, 
    float v_goal, 
    float frequency, 
    float halflife, 
    float dt, 
    float eps = 1e-5f)
{    
    float g = x_goal;
    float q = v_goal;
    float s = frequency_to_stiffness(frequency);
    float d = halflife_to_damping(halflife);
    float c = g + (d*q) / (s + eps);
    float y = d / 2.0f; 
    
    if (fabs(s - (d*d) / 4.0f) < eps) // Critically Damped
    {
        float j0 = x - c;
        float j1 = v + j0*y;
        
        float eydt = fast_negexp(y*dt);
        
        x = j0*eydt + dt*j1*eydt + c;
        v = -y*j0*eydt - y*dt*j1*eydt + j1*eydt;
    }
    else if (s - (d*d) / 4.0f > 0.0) // Under Damped
    {
        float w = sqrtf(s - (d*d)/4.0f);
        float j = sqrtf(squaref(v + y*(x - c)) / (w*w + eps) + squaref(x - c));
        float p = fast_atan((v + (x - c) * y) / (-(x - c)*w + eps));
        
        j = (x - c) > 0.0f ? j : -j;
        
        float eydt = fast_negexp(y*dt);
        
        x = j*eydt*cosf(w*dt + p) + c;
        v = -y*j*eydt*cosf(w*dt + p) - w*j*eydt*sinf(w*dt + p);
    }
    else if (s - (d*d) / 4.0f < 0.0) // Over Damped
    {
        float y0 = (d + sqrtf(d*d - 4*s)) / 2.0f;
        float y1 = (d - sqrtf(d*d - 4*s)) / 2.0f;
        float j1 = (c*y0 - x*y0 - v) / (y1 - y0);
        float j0 = x - j1 - c;
        
        float ey0dt = fast_negexp(y0*dt);
        float ey1dt = fast_negexp(y1*dt);

        x =  j0*ey0dt + j1*ey1dt + c;
        v = -y0*j0*ey0dt - y1*j1*ey1dt;
    }
}

float damping_ratio_to_stiffness(float ratio, float damping)
{
    return squaref(damping / (ratio * 2.0f));
}

float damping_ratio_to_damping(float ratio, float stiffness)
{
    return ratio * 2.0f * sqrtf(stiffness);
}

void spring_damper_exact_ratio(
    float& x, 
    float& v, 
    float x_goal, 
    float v_goal, 
    float damping_ratio, 
    float halflife, 
    float dt, 
    float eps = 1e-5f)
{    
    float g = x_goal;
    float q = v_goal;
    float d = halflife_to_damping(halflife);
    float s = damping_ratio_to_stiffness(damping_ratio, d);
    float c = g + (d*q) / (s + eps);
    float y = d / 2.0f; 
    
    if (fabs(s - (d*d) / 4.0f) < eps) // Critically Damped
    {
        float j0 = x - c;
        float j1 = v + j0*y;
        
        float eydt = fast_negexp(y*dt);
        
        x = j0*eydt + dt*j1*eydt + c;
        v = -y*j0*eydt - y*dt*j1*eydt + j1*eydt;
    }
    else if (s - (d*d) / 4.0f > 0.0) // Under Damped
    {
        float w = sqrtf(s - (d*d)/4.0f);
        float j = sqrtf(squaref(v + y*(x - c)) / (w*w + eps) + squaref(x - c));
        float p = fast_atan((v + (x - c) * y) / (-(x - c)*w + eps));
        
        j = (x - c) > 0.0f ? j : -j;
        
        float eydt = fast_negexp(y*dt);
        
        x = j*eydt*cosf(w*dt + p) + c;
        v = -y*j*eydt*cosf(w*dt + p) - w*j*eydt*sinf(w*dt + p);
    }
    else if (s - (d*d) / 4.0f < 0.0) // Over Damped
    {
        float y0 = (d + sqrtf(d*d - 4*s)) / 2.0f;
        float y1 = (d - sqrtf(d*d - 4*s)) / 2.0f;
        float j1 = (c*y0 - x*y0 - v) / (y1 - y0);
        float j0 = x - j1 - c;
        
        float ey0dt = fast_negexp(y0*dt);
        float ey1dt = fast_negexp(y1*dt);

        x =  j0*ey0dt + j1*ey1dt + c;
        v = -y0*j0*ey0dt - y1*j1*ey1dt;
    }
}


//--------------------------------------
/*
* most likely the case you may have actually used in your games:
* not only because it's the situation where the spring moves toward the goal as fast as possible without additional oscillation, 
* but because it's the easiest to compute and also to use as it has fewer parameters.
*/
void critical_spring_damper_exact(
    float& x, 
    float& v, 
    float x_goal, 
    float v_goal, 
    float halflife, 
    float dt)
{
    float g = x_goal;
    float q = v_goal;
    float d = halflife_to_damping(halflife);
    float c = g + (d*q) / ((d*d) / 4.0f);
    float y = d / 2.0f;	
    float j0 = x - c;
    float j1 = v + j0*y;
    float eydt = fast_negexp(y*dt);

    x = eydt*(j0 + j1*dt) + c;
    v = eydt*(v - j1*y*dt);
}
/*
* when the goal velocity q is zero
*/
void simple_spring_damper_exact(
    float& x, 
    float& v, 
    float x_goal, 
    float halflife, 
    float dt)
{
    float y = halflife_to_damping(halflife) / 2.0f;	
    float j0 = x - x_goal;
    float j1 = v + j0*y;
    float eydt = fast_negexp(y*dt);

    x = eydt*(j0 + j1*dt) + x_goal;
    v = eydt*(v - j1*y*dt);
}
/*
* when the goal position is zero
*/
void decay_spring_damper_exact(
    float& x, 
    float& v, 
    float halflife, 
    float dt)
{
    float y = halflife_to_damping(halflife) / 2.0f;	
    float j1 = v + x*y;
    float eydt = fast_negexp(y*dt);

    x = eydt*(x + j1*dt);
    v = eydt*(v - j1*y*dt);
}

//--------------------------------------

