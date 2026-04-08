#include "transform.h"
#include <math.h>

/* =========================
   vector3 functions
   ========================= */

vector3 vector3_create(float x, float y, float z)
{
    vector3 v = { x, y, z };
    return v;
}

vector3 vector3_zero(void)
{
    vector3 v = { 0.0f, 0.0f, 0.0f };
    return v;
}

vector3 vector3_add(vector3 a, vector3 b)
{
    vector3 v = { a.x + b.x, a.y + b.y, a.z + b.z };
    return v;
}

vector3 vector3_sub(vector3 a, vector3 b)
{
    vector3 v = { a.x - b.x, a.y - b.y, a.z - b.z };
    return v;
}

vector3 vector3_scale(vector3 v, float s)
{
    vector3 out = { v.x * s, v.y * s, v.z * s };
    return out;
}

// float vector3_dot(vector3 a, vector3 b)
// {
//     return a.x * b.x + a.y * b.y + a.z * b.z;
// }

// vector3 vector3_cross(vector3 a, vector3 b)
// {
//     vector3 v = {
//         a.y * b.z - a.z * b.y,
//         a.z * b.x - a.x * b.z,
//         a.x * b.y - a.y * b.x
//     };
//     return v;
// }

// float vector3_length_squared(vector3 v)
// {
//     return vector3_dot(v, v);
// }

// float vector3_length(vector3 v)
// {
//     return sqrtf(vector3_length_squared(v));
// }

// vector3 vector3_normalize(vector3 v)
// {
//     float len = vector3_length(v);

//     if (len > 1e-6f)
//     {
//         return vector3_scale(v, 1.0f / len);
//     }

//     return vector3_zero();
// }

// float vector3_distance(vector3 a, vector3 b)
// {
//     return vector3_length(vector3_sub(a, b));
// }


/* =========================
   euler6D functions
   ========================= */

euler6D euler6D_create(float x, float y, float z,
                       float roll, float pitch, float yaw)
{
    euler6D t = { x, y, z, roll, pitch, yaw };
    return t;
}

euler6D euler6D_zero(void)
{
    euler6D t = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    return t;
}

vector3 euler6D_position(euler6D t)
{
    vector3 v = { t.x, t.y, t.z };
    return v;
}

vector3 euler6D_rotation(euler6D t)
{
    vector3 v = { t.roll, t.pitch, t.yaw };
    return v;
}


/* =========================
   bodyAttitude4D functions
   ========================= */

bodyAttitude4D bodyAttitude4D_create(float x, float y, float z, float r)
{
    bodyAttitude4D b = { x, y, z, r };
    return b;
}

bodyAttitude4D bodyAttitude4D_zero(void)
{
    bodyAttitude4D b = { 0.0f, 0.0f, 0.0f, 0.0f };
    return b;
}

float bodyAttitude4D_yaw(bodyAttitude4D b)
{
    return b.x;
}

float bodyAttitude4D_pitch(bodyAttitude4D b)
{
    return b.y;
}

float bodyAttitude4D_surge(bodyAttitude4D b)
{
    return b.z;
}

float bodyAttitude4D_roll(bodyAttitude4D b)
{
    return b.r;
}

vector3 bodyAttitude4D_as_vector3(bodyAttitude4D b)
{
    vector3 v = { b.x, b.y, b.z };
    return v;
}