#ifndef TRANSFORM_H
#define TRANSFORM_H

typedef struct
{
    float x;
    float y;
    float z;
} vector3;

typedef struct
{
    double lat_deg;
    double lon_deg;
    float alt_m;
} gpsSTR;

typedef struct
{
    float x;
    float y;
    float z;

    float roll;
    float pitch;
    float yaw;
} euler6D;

typedef struct
{
    float x;   /* yaw   */
    float y;   /* pitch */
    float z;   /* surge */
    float r;   /* roll  */
} bodyAttitude4D;


/* vector3 functions */
vector3 vector3_create(float x, float y, float z);
vector3 vector3_zero(void);

vector3 vector3_add(vector3 a, vector3 b);
vector3 vector3_sub(vector3 a, vector3 b);
vector3 vector3_scale(vector3 v, float s);

// float   vector3_dot(vector3 a, vector3 b);
// vector3 vector3_cross(vector3 a, vector3 b);

// float   vector3_length_squared(vector3 v);
//float   vector3_length(vector3 v);
// vector3 vector3_normalize(vector3 v);
//float   vector3_distance(vector3 a, vector3 b);


/* euler6D functions */
euler6D euler6D_create(float x, float y, float z,
                       float roll, float pitch, float yaw);
euler6D euler6D_zero(void);

vector3 euler6D_position(euler6D t);
vector3 euler6D_rotation(euler6D t);


/* bodyAttitude4D functions */
bodyAttitude4D bodyAttitude4D_create(float x, float y, float z, float r);
bodyAttitude4D bodyAttitude4D_zero(void);

float bodyAttitude4D_yaw(bodyAttitude4D b);
float bodyAttitude4D_pitch(bodyAttitude4D b);
float bodyAttitude4D_surge(bodyAttitude4D b);
float bodyAttitude4D_roll(bodyAttitude4D b);

vector3 bodyAttitude4D_as_vector3(bodyAttitude4D b);

#endif