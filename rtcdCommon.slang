// Common geometry types and constants for RTCD exercises
#pragma once

namespace rtcdCommon {

// Numerical constants
static const float kMaxFloat = 3.402823466e+38f;
static const float EPSILON = 1e-6f;
static const float INTERVAL_EPSILON = 1e-3f;

// Plane equation: n·x == d
struct Plane { float3 n; float d; };

// Axis-aligned bounding boxes (AABB)
// region R = { x | min.x <= x <= max.x, ... }
struct AABB_MM { float3 min; float3 max; };
// region R = { x | min.x <= x <= min.x + d[0], ... }
struct AABB_MD { float3 min; float  d[3]; };
// region R = { x | |x-c.x| <= r[0], ... }
struct AABB_CF { float3 c; float  r[3]; };

// Sphere: region R = { x | |x-c|^2 <= r^2 }
struct Sphere { float3 c; float r;
    [mutating]
    __init(float3 center, float radius)
    {
        c = center;
        r = radius;
       }
    [mutating]
    __init() { c = float3(0.0f); r = 0.0f; }
  
    [mutating]
    __init(float3 p0) 
    {
      c = p0;
      r = 0.0f;
    }
  
    [mutating]
    __init(float3 p0, float3 p1)
    {
      c = (p0 + p1) * 0.5f;
      float3 d = p1 - p0;
      r = sqrt(dot(d, d)) * 0.5f;
    }
  
    [mutating]
    __init(float3 p0, float3 p1, float3 p2)
    {
      float3 A = p1 - p0;
      float3 B = p2 - p0;
      float3 C = cross(A, B);
      float C2 = dot(C, C);
      float3 num = cross(C, A) * dot(B, B) + cross(B, C) * dot(A, A);
      c = p0 + num / (2.0f * C2);
      r = sqrt(dot(c - p0, c - p0));
    }
  
    [mutating]
    __init(float3 p0, float3 p1, float3 p2, float3 p3)
    {
       // four-point circumsphere via linear solve
      float3 A = p1 - p0;
      float3 B = p2 - p0;
      float3 C = p3 - p0;
      float3 crossBC = cross(B, C);
      float denom = 2.0f * dot(A, crossBC);
      float3 u = crossBC * dot(A, A);
      float3 v = cross(C, A) * dot(B, B);
      float3 w = cross(A, B) * dot(C, C);
      c = p0 + (u + v + w) / denom;
      r = sqrt(dot(c - p0, c - p0));
    }
};

// Oriented bounding box (OBB)
// region R = { x | x = c + r*u[0] + s*u[1] + t*u[2], |r|<=e[0], ... }
struct OBB { float3 c; float3 u[3]; float3 e; };

// Rectangle in its local plane
struct Rect {
    float3 c;    // center point of rectangle
    float3 u[2]; // unit vectors determining local x and y axes for the rectangle
    float e[2];  // the halfwidth extents of the rectangle along the axes
};

// Assumes convex and planar
// hacky, just stubbed in for the moment...
struct Polygon {
    float3 v[10];
    int numVerts;
};

// Capsule: swept sphere along segment ab
struct Capsule { float3 a; float3 b; float r; };

// Lozenge: swept sphere along rectangle axes u[0], u[1]
struct Lozenge { float3 a; float3 u[2]; float r; };

// Section 4.6: slabs, k-DOP test, and 8-DOP computation
// Section 4.6.1: Region R = { (x, y, z) | dNear <= n·x <= dFar }
struct Slab { float3 n; float dNear; float dFar; };

// Section 4.6.2: 8-directional discrete orientation polytope
struct DOP8 { float min[4]; float max[4]; };

// Alias: treat DOP8 as KDOP (generic k-DOP) for k=8
typedef DOP8 KDOP;

struct Segment
{
    float3 a; // start point
    float3 b; // end   point
    __init(const float3 p0, const float3 p1) {
        a = p0; b = p1;
    };
};

} // namespace rtcdCommon