// Chapter 3 algorithms: barycentric coordinates, plane, convex quad, farthest-from-edge
#pragma once
namespace ch3 {

// Compute barycentric coordinates (u, v, w) for \
// point p with respect to triangle (a, b, c)
void BarycentricReferenceViaCramers(float3 a, float3 b, float3 c, float3 p,
                                    out float u, out float v, out float w)
{
    float3 v0 = b - a;
    float3 v1 = c - a;
    float3 v2 = p - a;
    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0 - v - w;
}

// Triangle area in 2D
inline float TriArea2D(float x1, float y1, float x2, float y2, float x3, float y3)
{
    return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
}

// Compute barycentric coordinates (u, v, w) for \
// point p with respect to triangle (a, b, c)
void Barycentric(float3 a, float3 b, float3 c, float3 p,
                 out float u, out float v, out float w)
{
    // Unnormalized triangle normal
    float3 m = cross(b - a, c - a);
    // Nominators and one-over-denominator for u and v ratios
    float nu, nv, ood;
    // Absolute components for determining projection plane
    float x = abs(m.x), y = abs(m.y), z = abs(m.z);

    // Compute areas in plane of largest projection
    if (x >= y && x >= z) {
        // x is largest, project to the yz plane
        nu = TriArea2D(p.y, p.z, b.y, b.z, c.y, c.z); // Area of PBC in yz plane
        nv = TriArea2D(p.y, p.z, c.y, c.z, a.y, a.z); // Area of PCA in yz plane
        ood = 1.0 / m.x;                             // 1/(2*area of ABC in yz plane)
    } else if (y >= x && y >= z) {
        // y is largest, project to the xz plane
        nu = TriArea2D(p.x, p.z, b.x, b.z, c.x, c.z);
        nv = TriArea2D(p.x, p.z, c.x, c.z, a.x, a.z);
        ood = 1.0 / -m.y;
    } else {
        // z is largest, project to the xy plane
        nu = TriArea2D(p.x, p.y, b.x, b.y, c.x, c.y);
        nv = TriArea2D(p.x, p.y, c.x, c.y, a.x, a.y);
        ood = 1.0 / m.z;
    }
    u = nu * ood;
    v = nv * ood;
    w = 1.0 - u - v;
}

// Test if point p is contained in triangle (a, b, c)
bool TestPointTriangle(float3 p, float3 a, float3 b, float3 c)
{
    float u, v, w;
    Barycentric(a, b, c, p, u, v, w);
    return v >= 0.0 && w >= 0.0 && (v + w) <= 1.0;
}

// Plane equation type
struct Plane { float3 n; float d; };

// Given three noncollinear points (ordered ccw), compute the plane equation
Plane ComputePlane(float3 a, float3 b, float3 c)
{
    Plane result;
    result.n = normalize(cross(b - a, c - a));
    result.d = dot(result.n, a);
    return result;
}

// Test if quadrilateral (a, b, c, d) is convex
bool IsConvexQuad(float3 a, float3 b, float3 c, float3 d)
{
    // Quad is nonconvex if Dot(Cross(bd, ba), Cross(bd, bc)) >= 0
    float3 bda = cross(d - b, a - b);
    float3 bdc = cross(d - b, c - b);
    if (dot(bda, bdc) >= 0.0f) return false;
    // Quad is now convex iff Dot(Cross(ac, ad), Cross(ac, ab)) < 0
    float3 acd = cross(c - a, d - a);
    float3 acb = cross(c - a, b - a);
    return dot(acd, acb) < 0.0f;
}

// Return index i of point p[i] farthest from the edge ab, to the left of the edge
int PointFarthestFromEdge(float2 a, float2 b, float2 p[], int n)
{
    // Create edge vector and vector (counterclockwise) perpendicular to it
    float2 e = b - a;
    float2 eperp = float2(-e.y, e.x);

    // Track index, "distance" and "rightmostness" of currently best point
    int bestIndex = -1;
    const float kFloatMax = 3.402823466e+38f;
    float maxVal = -kFloatMax, rightMostVal = -kFloatMax;

    // Test all points to find the one farthest from edge ab on the left side
    for (int i = 1; i < n; ++i)
    {
        float d = dot(p[i] - a, eperp); // d is proportional to distance along eperp
        float r = dot(p[i] - a, e);     // r is proportional to distance along e
        if (d > maxVal || (d == maxVal && r > rightMostVal))
        {
            bestIndex    = i;
            maxVal       = d;
            rightMostVal = r;
        }
    }
    return bestIndex;
}

};