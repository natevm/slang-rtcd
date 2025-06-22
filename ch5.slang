import rtcdCommon;
import ch4;

// Chapter 5 algorithms: closest-point queries and distance tests
#pragma once

namespace ch5 {
using namespace rtcdCommon;

// Section 5.1.1: Closest point on a plane
// Compute the projection of point q onto plane p
float3 ClosestPtPointPlane(float3 q, Plane p)
{
    float t = (dot(p.n, q) - p.d) / dot(p.n, p.n);
    return q - t * p.n;
}

// Section 5.1.1: Distance from point to plane
// Return signed distance; if p.n is unit-length, numerator is sufficient
float DistPointPlane(float3 q, Plane p)
{
    return (dot(p.n, q) - p.d) / dot(p.n, p.n);
}

// Section 5.1.2: Closest point on a segment
// Given segment ab and point c, computes closest point d on ab.
// Also returns t for the parametric position d = a + t*(b - a)
void ClosestPtPointSegment(float3 c, float3 a, float3 b,
                           out float t, out float3 d)
{
    float3 ab = b - a;
    float proj = dot(c - a, ab);
    if (proj <= 0.0f) {
        t = 0.0f; d = a;
    } else {
        float denom = dot(ab, ab);
        if (proj >= denom) {
            t = 1.0f; d = b;
        } else {
            t = proj / denom;
            d = a + t * ab;
        }
    }
}

// Section 5.1.2.1: Squared distance from point to segment
// Returns the squared distance between point c and segment ab
float SqDistPointSegment(float3 a, float3 b, float3 c)
{
    float3 ab = b - a, ac = c - a, bc = c - b;
    float e = dot(ac, ab);
    // Handle cases where c projects outside ab
    if (e <= 0.0f) return dot(ac, ac);
    float f = dot(ab, ab);
    if (e >= f) return dot(bc, bc);
    // Handle case where c projects onto ab
    return dot(ac, ac) - e * e / f;
}

// Section 5.1.3: Closest point on an AABB (min/max form)
// Region R = { x | min.x<=x<=max.x, ... }
// Given point p, return point q on (or in) AABB b, that is closest to p
void ClosestPtPointAABB(float3 p, AABB_MM b, out float3 q)
{
    // For each coordinate axis, if the point coordinate value is
    // outside box, clamp it to the box, else keep it as is
    for (int i = 0; i < 3; ++i) {
        float v = p[i];
        if (v < b.min[i]) v = b.min[i]; // v = max(v, b.min[i])
        if (v > b.max[i]) v = b.max[i]; // v = min(v, b.max[i])
        q[i] = v;
    }
}

// Section 5.1.3.1: Squared distance from point to AABB
// Computes the square distance between a point p and an AABB b
float SqDistPointAABB(float3 p, AABB_MM b)
{
    float sqDist = 0.0f;
    for (int i = 0; i < 3; ++i) {
        // For each axis count any excess distance outside box extents
        float v = p[i];
        if (v < b.min[i]) sqDist += (b.min[i] - v) * (b.min[i] - v);
        if (v > b.max[i]) sqDist += (v - b.max[i]) * (v - b.max[i]);
    }
    return sqDist;
}

// Section 5.1.4: Closest point on an OBB
// Region R = { x | x = c + r*u[0] + s*u[1] + t*u[2], |r|<=e[0], ... }
void ClosestPtPointOBB(float3 p, OBB b, out float3 q)
{
    float3 d = p - b.c;
    // Start result at center of box; make steps from there
    q = b.c;
    // For each OBB axis...
    for (int i = 0; i < 3; ++i) {
        // ...project d onto that axis to get the distance
        // along the axis of d from the box center
        float dist = dot(d, b.u[i]);
        // If distance farther than the box extents, clamp to the box
        if (dist > b.e[i]) dist = b.e[i];
        if (dist < -b.e[i]) dist = -b.e[i];
        // Step that distance along the axis to get world coordinate
        q += dist * b.u[i];
    }
}

// Section 5.1.4.1: Squared distance from point to OBB (via closest-point)
// Computes the square distance between point p and OBB b
float SqDistPointOBB(float3 p, OBB b)
{
    float3 closest;
    ClosestPtPointOBB(p, b, closest);
    float sqDist = dot(closest - p, closest - p);
    return sqDist;
}

// Given point p, return point q on (or in) Rect r, closest to p
void ClosestPtPointRect(float3 p, Rect r, out float3 q)
{
    float3 d = p - r.c;
    // Start result at center of rect; make steps from there
    q = r.c;
    // For each rect axis...
    for (int i = 0; i < 2; ++i) {
        // ...project d onto that axis to get the distance
        // along the axis of d from the rect center
        float dist = dot(d, r.u[i]);
        // If distance farther than the rect extents, clamp to the rect
        if (dist > r.e[i]) dist = r.e[i];
        if (dist < -r.e[i]) dist = -r.e[i];
        // Step that distance along the axis to get world coordinate
        q += dist * r.u[i];
    }
}

// Given point p and rectangle corners a, b, c, return point q on (or in) rect
void ClosestPtPointRect(float3 p, float3 a, float3 b, float3 c, out float3 q)
{
    float3 ab = b - a; // vector across rect
    float3 ac = c - a; // vector up rect
    float3 d = p - a;
    // Start result at top-left corner of rect; make steps from there
    q = a;
    // Clamp p' (projection of p to plane of r) to rectangle in the across direction
    float dist = dot(d, ab);
    float maxdist = dot(ab, ab);
    if (dist >= maxdist)
        q += ab;
    else if (dist > 0.0f)
        q += (dist / maxdist) * ab;
    // Clamp p' (projection of p to plane of r) to rectangle in the down direction
    dist = dot(d, ac);
    maxdist = dot(ac, ac);
    if (dist >= maxdist)
        q += ac;
    else if (dist > 0.0f)
        q += (dist / maxdist) * ac;
}

// Section 5.1.5: Closest point on triangle
// Returns the point on (or in) triangle abc that is closest to p
float3 ClosestPtPointTriangle(float3 p, float3 a, float3 b, float3 c)
{
    float3 ab = b - a;
    float3 ac = c - a;
    float3 bc = c - b;

    // Compute parametric position s for projection P' of P on AB,
    // P' = A + s*AB, s = snom/(snom+sdenom)
    float snom = dot(p - a, ab), sdenom = dot(p - b, a - b);

    // Compute parametric position t for projection P' of P on AC,
    // P' = A + t*AC, t = tnom/(tnom+tdenom)
    float tnom = dot(p - a, ac), tdenom = dot(p - c, a - c);

    if (snom <= 0.0f && tnom <= 0.0f)
        return a; // Vertex region early out

    // Compute parametric position u for projection P' of P on BC,
    // P' = B + u*BC, u = unom/(unom+udenom)
    float unom = dot(p - b, bc), udenom = dot(p - c, b - c);

    if (sdenom <= 0.0f && unom <= 0.0f)
        return b; // Vertex region early out
    if (tdenom <= 0.0f && udenom <= 0.0f)
        return c; // Vertex region early out

    // P is outside (or on) AB if the triple scalar product [N, PA, PB] <= 0
    float3 n = cross(b - a, c - a);
    float vc = dot(n, cross(a - p, b - p));
    // If P outside AB and within feature region of AB,
    // return projection of P onto AB
    if (vc <= 0.0f && snom >= 0.0f && sdenom >= 0.0f)
        return a + snom / (snom + sdenom) * ab;

    // P is outside (or on) BC if the triple scalar product [N, PB, PC] <= 0
    float va = dot(n, cross(b - p, c - p));
    // If P outside BC and within feature region of BC,
    // return projection of P onto BC
    if (va <= 0.0f && unom >= 0.0f && udenom >= 0.0f)
        return b + unom / (unom + udenom) * bc;

    // P is outside (or on) CA if the triple scalar product [N, PC, PA] <= 0
    float vb = dot(n, cross(c - p, a - p));
    // If P outside CA and within feature region of CA,
    // return projection of P onto CA
    if (vb <= 0.0f && tnom >= 0.0f && tdenom >= 0.0f)
        return a + tnom / (tnom + tdenom) * ac;

    // P must project inside face region. Compute Q using barycentric coordinates
    float u = va / (va + vb + vc);
    float v = vb / (va + vb + vc);
    float w = 1.0f - u - v; // = vc / (va + vb + vc)
    return u * a + v * b + w * c;
}

// Section 5.1.6: Closest point on a tetrahedron
// Returns the point on (or in) tetrahedron abcd that is closest to p
float3 ClosestPtPointTetrahedron(float3 p, float3 a, float3 b, float3 c, float3 d)
{
    // Start out assuming point inside all halfspaces, so closest to itself
    float3 closestPt = p;
    float bestSqDist = kMaxFloat;
    // If point outside face abc then compute closest point on abc
    if (PointOutsideOfPlane(p, a, b, c, d)) {
        float3 q = ClosestPtPointTriangle(p, a, b, c);
        float sqDist = dot(q - p, q - p);
        // Update best closest point if (squared) distance is less than current best
        if (sqDist < bestSqDist) { bestSqDist = sqDist; closestPt = q; }
    }
    // Repeat test for faces acd, adb, bdc
    if (PointOutsideOfPlane(p, a, c, d, b)) {
        float3 q = ClosestPtPointTriangle(p, a, c, d);
        float sqDist = dot(q - p, q - p);
        if (sqDist < bestSqDist) { bestSqDist = sqDist; closestPt = q; }
    }
    if (PointOutsideOfPlane(p, a, d, b, c)) {
        float3 q = ClosestPtPointTriangle(p, a, d, b);
        float sqDist = dot(q - p, q - p);
        if (sqDist < bestSqDist) { bestSqDist = sqDist; closestPt = q; }
    }
    if (PointOutsideOfPlane(p, b, d, c, a)) {
        float3 q = ClosestPtPointTriangle(p, b, d, c);
        float sqDist = dot(q - p, q - p);
        if (sqDist < bestSqDist) { bestSqDist = sqDist; closestPt = q; }
    }
    return closestPt;
}

// Section 5.1.6: Test if point p lies outside plane through abc
// Returns true if [PA, AB, AC] >= 0
bool PointOutsideOfPlane(float3 p, float3 a, float3 b, float3 c)
{
    return dot(p - a, cross(b - a, c - a)) >= 0.0f;
}

// Section 5.1.6: Test if p and d lie on opposite sides of plane through abc
// Returns true if sign(p) * sign(d) < 0
bool PointOutsideOfPlane(float3 p, float3 a, float3 b, float3 c,
                        float3 d)
{
    float signp = dot(p - a, cross(b - a, c - a));
    float signd = dot(d - a, cross(b - a, c - a));
    return signp * signd < 0.0f;
}

// Section 5.1.9: Closest points of two segments
// Computes closest points c1, c2 of S1(s)=P1+s*(Q1-P1) and
// S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
// distance between S1(s) and S2(t)
float ClosestPtSegmentSegment(float3 p1, float3 q1, float3 p2, float3 q2,
                              out float s, out float t,
                              out float3 c1, out float3 c2)
{
    float3 d1 = q1 - p1; // Direction vector of segment S1
    float3 d2 = q2 - p2; // Direction vector of segment S2
    float3 r  = p1 - p2;

    float a = dot(d1, d1); // Squared length of segment S1, always nonnegative
    float e = dot(d2, d2); // Squared length of segment S2, always nonnegative
    float f = dot(d2, r);

    // Check if either or both segments degenerate into points
    if (a <= EPSILON && e <= EPSILON)
    {
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return dot(c1 - c2, c1 - c2);
    }
    if (a <= EPSILON)
    {
        // First segment degenerates into a point
        s = 0.0f;
        t = clamp(f / e, 0.0f, 1.0f);
    }
    else
    {
        float c = dot(d1, r);
        if (e <= EPSILON)
        {
            // Second segment degenerates into a point
            t = 0.0f;
            s = clamp(-c / a, 0.0f, 1.0f);
        }
        else
        {
            // The general nondegenerate case starts here
            float b = dot(d1, d2);
            float denom = a * e - b * b; // Always nonnegative

            // If segments not parallel, compute closest point on L1 to L2, and
            // clamp to segment S1. Else pick arbitrary s (here 0)
            s = (denom != 0.0f)
                ? clamp((b * f - c * e) / denom, 0.0f, 1.0f)
                : 0.0f;

            // Compute point on L2 closest to S1(s)
            t = (b * s + f) / e;

            // If t in [0,1] done. Else clamp t, recompute s for the new value
            // of t using s = dot((P2+D2*t)-P1,D1) / dot(D1,D1)= (t*b - c) / a
            // and clamp s to [0, 1]
            if (t < 0.0f)
            {
                t = 0.0f;
                s = clamp(-c / a, 0.0f, 1.0f);
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
                s = clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return dot(c1 - c2, c1 - c2);
}

// Section 5.1.9.1: Returns 2 times the signed triangle area
// Positive if abc is ccw, negative if cw, zero if degenerate.
float Signed2DTriArea(float3 a, float3 b, float3 c)
{
    return (a.x - c.x) * (b.y - c.y)
         - (a.y - c.y) * (b.x - c.x);
}

// Section 5.1.9.2: Test if segments ab and cd overlap in 2D
// If they do, compute intersection t value along ab and intersection point p
// Returns 1 if segments intersect, 0 otherwise
int Test2DSegmentSegment(float3 a, float3 b, float3 c, float3 d,
                         out float t, out float3 p)
{
    // Sign of areas correspond to which side of ab points d and c are
    float a1 = Signed2DTriArea(a, b, d); // Compute winding of abd (+ or -)
    float a2 = Signed2DTriArea(a, b, c); // To intersect, must have sign opposite of a1

    // If c and d are on different sides of ab, areas have different signs
    if (a1 * a2 < 0.0f)
    {
        // Compute signs for a and b with respect to segment cd
        float a3 = Signed2DTriArea(c, d, a); // Compute winding of cda (+ or -)
        // Since area is constant, a4 = a3 + a2 - a1
        float a4 = a3 + a2 - a1;
        // Points a and b on different sides of cd if areas have different signs
        if (a3 * a4 < 0.0f)
        {
            // Segments intersect. Find intersection along L(t)=a+t*(b-a).
            // t = a3 / (a3 - a4)
            t = a3 / (a3 - a4);
            p = a + t * (b - a);
            return 1;
        }
    }

    // Segments not intersecting (or collinear)
    t = 0.0f;
    p = float3(0.0f, 0.0f, 0.0f);
    return 0;
}

// Section 5.2.2: Sphere-plane and halfspace tests
// Determine whether plane p intersects sphere s
int TestSpherePlane(Sphere s, Plane p)
{
    // For a normalized plane (||p.n|| = 1), signed distance = dot(s.c,p.n)-p.d
    float dist = dot(s.c, p.n) - p.d;
    // If sphere center within +/-radius from plane, plane intersects sphere
    return abs(dist) <= s.r;
}

// Determine whether sphere s fully behind (inside negative halfspace of) plane p
int InsideSpherePlane(Sphere s, Plane p)
{
    float dist = dot(s.c, p.n) - p.d;
    return dist < -s.r;
}

// Determine whether sphere s intersects negative halfspace of plane p
int TestSphereHalfspace(Sphere s, Plane p)
{
    float dist = dot(s.c, p.n) - p.d;
    return dist <= s.r;
}

// Section 5.2.3: Test if OBB b intersects plane p
// Intersection occurs when signed distance of box center to plane lies within
// the projection interval radius of the OBB
int TestOBBPlane(OBB b, Plane p)
{
    // Projection interval radius of b onto L(t) = b.c + t*p.n
    float r = b.e[0]*abs(dot(p.n, b.u[0]))
            + b.e[1]*abs(dot(p.n, b.u[1]))
            + b.e[2]*abs(dot(p.n, b.u[2]));
    // Signed distance of box center from plane
    float s = dot(p.n, b.c) - p.d;
    return abs(s) <= r;
}

// Section 5.2.5: Sphere vs. AABB test
// Returns true if sphere s intersects AABB b, false otherwise
int TestSphereAABB(Sphere s, AABB_MM b)
{
    // Compute squared distance between sphere center and AABB
    float sqDist = SqDistPointAABB(s.c, b);

    // Sphere and AABB intersect if (squared) distance <= r^2
    return sqDist <= s.r * s.r;
}

// Section 5.2.5.1: Sphere vs. AABB intersection with closest-point
// Returns true if sphere s intersects AABB b, false otherwise.
// The point p on the AABB closest to the sphere center is also returned
int TestSphereAABB(Sphere s, AABB_MM b, out float3 p)
{
    // Find point p on AABB closest to sphere center
    ClosestPtPointAABB(s.c, b, p);

    // Sphere and AABB intersect if the (squared) distance from sphere
    // center to point p is less than the (squared) sphere radius
    float3 v = p - s.c;
    return dot(v, v) <= s.r * s.r;
}

// Section 5.2.7: Sphere vs. triangle intersection with closest-point
// Returns true if sphere s intersects triangle abc, false otherwise.
// The point p on abc closest to the sphere center is also returned
int TestSphereTriangle(Sphere s, float3 a, float3 b, float3 c, out float3 p)
{
    // Find point P on triangle ABC closest to sphere center
    p = ClosestPtPointTriangle(s.c, a, b, c);

    // Sphere and triangle intersect if the (squared) distance from sphere
    // center to point p is less than the (squared) sphere radius
    float3 v = p - s.c;
    return dot(v, v) <= s.r * s.r;
}

// Section 5.2.6: Sphere vs. OBB intersection with closest-point
// Returns true if sphere s intersects OBB b, false otherwise.
// The point p on the OBB closest to the sphere center is also returned
int TestSphereOBB(Sphere s, OBB b, out float3 p)
{
    // Find point p on OBB closest to sphere center
    ClosestPtPointOBB(s.c, b, p);

    // Sphere and OBB intersect if the (squared) distance from sphere
    // center to point p is less than the (squared) sphere radius
    float3 v = p - s.c;
    return dot(v, v) <= s.r * s.r;
}

// Section 5.2.8: Sphere vs. polygon intersection (stub)
// Returns true if sphere s intersects polygon p, false otherwise.

// Test whether sphere s intersects polygon p
int TestSphereConvexPolygon(Sphere s, Polygon p)
{
    // Compute normal for the plane of the polygon
    float3 n = normalize(cross(p.v[1] - p.v[0], p.v[2] - p.v[0]));
    // Compute the plane equation for p
    Plane m; m.n = n; m.d = -dot(n, p.v[0]);
    // No intersection if sphere not intersecting plane of polygon
    if (!bool(TestSpherePlane(s, m))) return 0;
    // Test to see if any one of the polygon edges pierces the sphere
    for (int k = p.numVerts, i = 0, j = k - 1; i < k; j = i, i++) {
        float t;
        float3 q;
        // Test if edge (p.v[j], p.v[i]) intersects s
        if (bool(IntersectRaySphere(p.v[j], p.v[i] - p.v[j], s, t, q)) && t <= 1.0f)
            return 1;
    }
    // Test if the orthogonal projection q of the sphere center onto m is inside p
    float3 q = ClosestPtPointPlane(s.c, m);
    return PointInConvexPolygon(q, p.numVerts, p.v);
}

// Section 5.2.9: Triangle vs AABB intersection
// Returns true if triangle v0v1v2 intersects AABB b, false otherwise
int TestTriangleAABB(float3 v0, float3 v1, float3 v2, AABB_MM b)
{
    float p0, p2, r;

    // Compute box center and extents
    float3 c = (b.max + b.min) * 0.5f;
    float e0 = (b.max.x - b.min.x) * 0.5f;
    float e1 = (b.max.y - b.min.y) * 0.5f;
    float e2 = (b.max.z - b.min.z) * 0.5f;

    // Translate triangle to AABB origin
    v0 -= c;
    v1 -= c;
    v2 -= c;

    // Compute triangle edges
    float3 f0 = v1 - v0;
    float3 f1 = v2 - v1;
    float3 f2 = v0 - v2;

    // Test axes a00..a22 (category 3)
    // axis a00: cross(f0, X)
    p0 = v0.z * f0.y - v0.y * f0.z;
    p2 = v2.z * f0.y - v2.y * f0.z;
    r = e1 * abs(f0.z) + e2 * abs(f0.y);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // axis a01: cross(f1, X)
    p0 = v0.z * f1.y - v0.y * f1.z;
    p2 = v2.z * f1.y - v2.y * f1.z;
    r = e1 * abs(f1.z) + e2 * abs(f1.y);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // axis a02: cross(f2, X)
    p0 = v0.z * f2.y - v0.y * f2.z;
    p2 = v2.z * f2.y - v2.y * f2.z;
    r = e1 * abs(f2.z) + e2 * abs(f2.y);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // axis a10: cross(f0, Y)
    p0 = v0.x * f0.z - v0.z * f0.x;
    p2 = v2.x * f0.z - v2.z * f0.x;
    r = e0 * abs(f0.z) + e2 * abs(f0.x);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // axis a11: cross(f1, Y)
    p0 = v0.x * f1.z - v0.z * f1.x;
    p2 = v2.x * f1.z - v2.z * f1.x;
    r = e0 * abs(f1.z) + e2 * abs(f1.x);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // axis a12: cross(f2, Y)
    p0 = v0.x * f2.z - v0.z * f2.x;
    p2 = v2.x * f2.z - v2.z * f2.x;
    r = e0 * abs(f2.z) + e2 * abs(f2.x);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // axis a20: cross(f0, Z)
    p0 = v0.y * f0.x - v0.x * f0.y;
    p2 = v2.y * f0.x - v2.x * f0.y;
    r = e0 * abs(f0.y) + e1 * abs(f0.x);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // axis a21: cross(f1, Z)
    p0 = v0.y * f1.x - v0.x * f1.y;
    p2 = v2.y * f1.x - v2.x * f1.y;
    r = e0 * abs(f1.y) + e1 * abs(f1.x);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // axis a22: cross(f2, Z)
    p0 = v0.y * f2.x - v0.x * f2.y;
    p2 = v2.y * f2.x - v2.x * f2.y;
    r = e0 * abs(f2.y) + e1 * abs(f2.x);
    if (min(p0, p2) > r || max(p0, p2) < -r) return 0;

    // Test the three axes corresponding to the face normals of b (category 1)
    float minX = min(min(v0.x, v1.x), v2.x);
    float maxX = max(max(v0.x, v1.x), v2.x);
    if (maxX < -e0 || minX > e0) return 0;
    float minY = min(min(v0.y, v1.y), v2.y);
    float maxY = max(max(v0.y, v1.y), v2.y);
    if (maxY < -e1 || minY > e1) return 0;
    float minZ = min(min(v0.z, v1.z), v2.z);
    float maxZ = max(max(v0.z, v1.z), v2.z);
    if (maxZ < -e2 || minZ > e2) return 0;

    // Test separating axis corresponding to triangle face normal (category 2)
    {
        Plane pPlane;
        pPlane.n = cross(f0, f1);
        pPlane.d = dot(pPlane.n, v0);
        // Errata: shift the box by c before testing
        AABB_MM b2 = { b.min - c, b.max - c };
        return TestAABBPlane(b2, pPlane);
    }
}

// Section 5.3.1: Intersect segment vs. plane
// Compute intersection of segment ab with plane p (||p.n|| need not be normalized)
// Returns 1 and sets t,q if intersection occurs with t in [0,1], otherwise 0.
int IntersectSegmentPlane(float3 a, float3 b, Plane p, out float t, out float3 q)
{
    float3 ab = b - a;
    t = (p.d - dot(p.n, a)) / dot(p.n, ab);
    if (t >= 0.0f && t <= 1.0f) {
        q = a + t * ab;
        return 1;
    }
    q = float3(0.0f);
    return 0;
}

// Section 5.3.1: Intersect segment ab against plane of triangle def.
// If intersecting, return t value and position q of intersection
int IntersectSegmentPlane(float3 a, float3 b, float3 d, float3 e, float3 f,
                          out float t, out float3 q)
{
    Plane p;
    p.n = cross(e - d, f - d);
    p.d = dot(p.n, d);
    return IntersectSegmentPlane(a, b, p, t, q);
}

// Section 5.3.2: Intersect ray r = p + t*d, |d| = 1, with sphere s
// returns t value of intersection and intersection point q
int IntersectRaySphere(float3 p, float3 d, Sphere s, out float t, out float3 q)
{
    float3 m = p - s.c;
    float b = dot(m, d);
    float c = dot(m, m) - s.r * s.r;
    // Exit if ray's origin outside s (c > 0) and ray pointing away from s (b > 0)
    if (c > 0.0f && b > 0.0f) { t = 0.0f; q = float3(0.0f); return 0; }
    float discr = b*b - c;
    // A negative discriminant corresponds to ray missing sphere
    if (discr < 0.0f) { t = 0.0f; q = float3(0.0f); return 0; }
    // Ray now found to intersect sphere, compute smallest t value of intersection
    t = -b - sqrt(discr);
    // If t is negative, ray started inside sphere so clamp t to zero
    if (t < 0.0f) t = 0.0f;
    q = p + t * d;
    return 1;
}

// Section 5.3.2: Test if ray r = p + t*d intersects sphere s
int TestRaySphere(float3 p, float3 d, Sphere s)
{
    float3 m = p - s.c;
    float c = dot(m, m) - s.r * s.r;
    // If there is definitely at least one real root, there must be an intersection
    if (c <= 0.0f) return 1;
    float b = dot(m, d);
    // Early exit if ray origin outside sphere and ray pointing away from sphere
    if (b > 0.0f) return 0;
    float disc = b*b - c;
    // A negative discriminant corresponds to ray missing sphere
    if (disc < 0.0f) return 0;
    // Now ray must hit sphere
    return 1;
}
// Section 5.3.3: Intersect ray R(t) = p + t*d against AABB a.
// When intersecting, return intersection distance tmin and point q of intersection
int IntersectRayAABB(float3 p, float3 d, AABB_MM a, out float tmin, out float3 q)
{
    tmin = 0.0f;
    float tmax = kMaxFloat;
    for (int i = 0; i < 3; ++i) {
        if (abs(d[i]) < EPSILON) {
            // Ray is parallel to slab. No hit if origin not within slab
            if (p[i] < a.min[i] || p[i] > a.max[i]) {
                q = float3(0.0f);
                return 0;
            }
        } else {
            float ood = 1.0f / d[i];
            float t1 = (a.min[i] - p[i]) * ood;
            float t2 = (a.max[i] - p[i]) * ood;
            if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
            tmin = max(tmin, t1);
            tmax = min(tmax, t2);
            if (tmin > tmax) {
                q = float3(0.0f);
                return 0;
            }
        }
    }
    q = p + tmin * d;
    return 1;
}
// Section 5.3.3: Test if segment ab intersects AABB b
// Returns true if segment from p0 to p1 intersects box a (AABB_MM), false otherwise
int TestSegmentAABB(float3 p0, float3 p1, AABB_MM a)
{
    float3 c = (a.min + a.max) * 0.5f; // Box center-point
    float3 e = a.max - c;             // Box halflength extents
    float3 m = (p0 + p1) * 0.5f;      // Segment midpoint
    float3 d = p1 - m;                // Segment halflength vector
    m = m - c;                        // Translate box and segment to origin

    // Try world coordinate axes as separating axes
    float adx = abs(d.x);
    if (abs(m.x) > e.x + adx) return 0;
    float ady = abs(d.y);
    if (abs(m.y) > e.y + ady) return 0;
    float adz = abs(d.z);
    if (abs(m.z) > e.z + adz) return 0;

    // Add in an epsilon term to counteract arithmetic errors when segment is
    // (near) parallel to a coordinate axis
    adx += EPSILON; ady += EPSILON; adz += EPSILON;

    // Try cross products of segment direction vector with coordinate axes
    if (abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) return 0;
    if (abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) return 0;
    if (abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) return 0;

    // No separating axis found; segment must be overlapping AABB
    return 1;
}

// Helper for double-sided intersection testing
static bool SameSign(float a, float b)
{
    return (a >= 0.0f && b >= 0.0f) || (a <= 0.0f && b <= 0.0f);
}

// Section 5.3.4: Intersect line pq against CCW triangle abc
// Returns true and barycentric coords u,v,w if line pierces triangle
// If doubleSided==true, intersection on either side of the triangle plane is accepted
int IntersectLineTriangle(float3 p, float3 q,
                          float3 a, float3 b, float3 c,
                          out float u, out float v, out float w,
                          bool doubleSided)
{
    float3 pq = q - p;
    float3 pa = a - p;
    float3 pb = b - p;
    float3 pc = c - p;
    // Optimized test: share cross(pq, pc)
    float3 m = cross(pq, pc);
    u = dot(pb, m);
    v = -dot(pa, m);
    w = dot(pq, cross(pb, pa));

    if (doubleSided) {
        if (!SameSign(u, v) || !SameSign(u, w)) return 0;
    } else {
        if (u < 0.0f || v < 0.0f || w < 0.0f) return 0;
    }

    float denom = 1.0f / (u + v + w);
    u *= denom;
    v *= denom;
    w *= denom;
    return 1;
}

// Overload: single-sided test (default)
int IntersectLineTriangle(float3 p, float3 q,
                          float3 a, float3 b, float3 c,
                          out float u, out float v, out float w)
{
    return IntersectLineTriangle(p, q, a, b, c, u, v, w, false);
}

// Section 5.6.3: Intersect segment pq against CCW triangle abc
// Returns true and barycentric coords u,v,w and t if segment pierces triangle
int IntersectSegmentTriangle(float3 p, float3 q,
                             float3 a, float3 b, float3 c,
                             out float u, out float v, out float w,
                             out float t)
{
    float3 ab = b - a;
    float3 ac = c - a;
    float3 qp = p - q;

    // Compute triangle normal
    float3 n = cross(ab, ac);

    // Compute denominator d. If d <= 0, parallel or pointing away
    float d = dot(qp, n);
    if (d <= 0.0f) { u = v = w = t = 0.0f; return 0; }

    // Compute intersection t value on segment
    float3 ap = p - a;
    t = dot(ap, n);
    if (t < 0.0f || t > d) { u = v = w = 0.0f; return 0; }

    // Barycentric coordinates test
    float3 e = cross(qp, ap);
    v = dot(ac, e);
    if (v < 0.0f || v > d) { u = w = t = 0.0f; return 0; }
    w = -dot(ab, e);
    if (w < 0.0f || v + w > d) { u = t = 0.0f; return 0; }

    // Compute final barycentrics
    float ood = 1.0f / d;
    t *= ood;
    v *= ood;
    w *= ood;
    u = 1.0f - v - w;
    return 1;
}

// Section 5.2.3: Test if AABB b intersects plane p
// Using AABB min/max form (AABB from rtcdCommon)
int TestAABBPlane(AABB_MM b, Plane p)
{
    // Compute AABB center and positive extents
    float3 c = (b.max + b.min) * 0.5f;
    float3 e = b.max - c;
    // Projection interval radius onto plane normal
    float r = e.x*abs(p.n.x) + e.y*abs(p.n.y) + e.z*abs(p.n.z);
    // Signed distance of center from plane
    float s = dot(p.n, c) - p.d;
    return abs(s) <= r;
}

inline float ScalarTriple(const float3 a,
                          const float3 b,
                          const float3 c)
{
    return dot(a, cross(b, c));
}

// Given line pq and ccw quadrilateral abcd, return whether the line
// pierces the triangle. If so, also return the point r of intersection
int IntersectLineQuad(float3 p, float3 q,
                      float3 a, float3 b, float3 c, float3 d,
                      out float3 r)
{
    float3 pq = q - p;
    float3 pa = a - p;
    float3 pb = b - p;
    float3 pc = c - p;

    float3 m = cross(pc, pq);
    float v = dot(pa, m); // choose the triangle

    // Determine which triangle to test against by testing against diagonal first
    if (v >= 0.0f) { // ScalarTriple(pq, pa, pc);
        // Test intersection against triangle abc
        float u = -dot(pb, m); // ScalarTriple(pq, pc, pb);
        if (u < 0.0f) { r = float3(0.0f); return 0; }
        float w = dot(pq, cross(pb, pa)); // pb × pa
        if (w < 0.0f) { r = float3(0.0f); return 0; }
        // Compute r, r = u*a + v*b + w*c, from barycentric coordinates (u, v, w)
        float denom = 1.0f / (u + v + w);
        u *= denom; v *= denom; w *= denom;
        r = u * a + v * b + w * c;
    }
    else {
        // Test intersection against triangle dac
        float3 pd = d - p;
        float u = dot(pd, m); // ScalarTriple(pq, pd, pc);
        if (u < 0.0f) { r = float3(0.0f); return 0; }
        float w = dot(pq, cross(pa, pd));  // pa × pd
        if (w < 0.0f) { r = float3(0.0f); return 0; }
        v = -v; // make v positive
        float denom = 1.0f / (u + v + w);
        u *= denom; v *= denom; w *= denom;
        r = u * a + v * d + w * c; // (a,d,c) is the vertex order here
    }
    return 1;
}

// 5.3.7: Intersect segment S(t)=sa+t(sb-sa), 0<=t<=1 against cylinder specified by p, q and r
int IntersectSegmentCylinder(float3 sa, float3 sb, float3 p, float3 q, float r, out float t)
{
    float3 d = q - p, m = sa - p, n = sb - sa;
    float md = dot(m, d);
    float nd = dot(n, d);
    float dd = dot(d, d);
    // Test if segment fully outside either endcap of cylinder
    if (md < 0.0f && md + nd < 0.0f) { t = 0.0f; return 0; } // Segment outside 'p' side of cylinder
    if (md > dd && md + nd > dd) { t = 0.0f; return 0; } // Segment outside 'q' side of cylinder
    float nn = dot(n, n);
    float mn = dot(m, n);
    float a = dd * nn - nd * nd;
    float k = dot(m, m) - r * r;
    float c = dd * k - md * md;
    if (abs(a) < EPSILON) {
        // Segment runs parallel to cylinder axis
        if (c > 0.0f) { t = 0.0f; return 0; } // 'a' and thus the segment lie outside cylinder
        // Now known that segment intersects cylinder; figure out how it intersects
        if (md < 0.0f) t = -mn / nn; // Intersect segment against 'p' endcap
        else if (md > dd) t = (nd - mn) / nn; // Intersect segment against 'q' endcap
        else t = 0.0f; // 'a' lies inside cylinder
        return 1;
    }
    float b = dd * mn - nd * md;
    float discr = b * b - a * c;
    if (discr < 0.0f) { t = 0.0f; return 0; } // No real roots; no intersection
    // Eratta 3/1/07 changes added.
    float t0 = t = (-b - sqrt(discr)) / a;
    if (md + t * nd < 0.0f) {
        // Intersection outside cylinder on ‘p’ side
        if (nd <= 0.0f) return 0; // Segment pointing away from endcap
        t = -md / nd;
        // Keep intersection if dot(S(t) - p, S(t) - p) <= r^2
        if (k + t * (2.0f * mn + t * nn) <= 0.0f) return 1;
    } else if (md + t * nd > dd) {
        // Intersection outside cylinder on ‘q’ side
        if (nd >= 0.0f) return 0; // Segment pointing away from endcap
        t = (dd - md) / nd;
        // Keep intersection if dot(S(t) - q, S(t) - q) <= r^2
        if (k + dd - 2.0f * md + t * (2.0f * (mn - nd) + t * nn) <= 0.0f) return 1;
    }
    t = t0;
    // Intersection if segment intersects cylinder between the end-caps
    return t >= 0.0f && t <= 1.0f;
}

// 5.3.8: Intersect segment S(t)=A+t(B-A), 0<=t<=1 against convex polyhedron specified
// by the n halfspaces defined by the planes p[]. On exit tfirst and tlast
// define the intersection, if any
int IntersectSegmentPolyhedron(float3 a, float3 b, Plane p[], int n,
                               out float tfirst, out float tlast)
{
    // Compute direction vector for the segment
    float3 d = b - a;
    // Set initial interval to being the whole segment. For a ray, tlast should be
    // set to +kMaxFloat. For a line, additionally tfirst should be set to -kMaxFloat
    tfirst = 0.0f;
    tlast = 1.0f;
    // Intersect segment against each plane
    for (int i = 0; i < n; i++) {
        float denom = dot(p[i].n, d);
        float dist = p[i].d - dot(p[i].n, a);
        // Test if segment runs parallel to the plane
        if (denom == 0.0f) {
            // If so, return ìno intersectionî if segment lies outside plane
            if (dist > 0.0f) return 0;
        } else {
            // Compute parameterized t value for intersection with current plane
            float t = dist / denom;
            if (denom < 0.0f) {
                // When entering halfspace, update tfirst if t is larger
                if (t > tfirst) tfirst = t;
            } else {
                // When exiting halfspace, update tlast if t is smaller
                if (t < tlast) tlast = t;
            }
            // Exit with ìno intersectionî if intersection becomes empty
            if (tfirst > tlast) return 0;
        }
    }
    // A nonzero logical intersection, so the segment intersects the polyhedron
    return 1;
}

inline bool TriangleIsCCW(const float2 a,
                          const float2 b,
                          const float2 c)
{
    // Twice the signed area of the 2-D triangle
    float area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    return area2 > 0.0f; // CCW if positive in a right-handed system
}

// Returns true if the vertex order (a,b,c) is counter-clockwise
// under the right–hand rule in a right-handed coordinate system.
//
// The test projects the triangle onto the axis-aligned plane in which its
// area is largest (“dominant axis” projection).  In that projection the
// signed area reduces to a single component of the 3-D normal:
//
//   n  = (b-a) × (c-a)
//   n.x  is the signed area in the   YZ–plane
//   n.y  is the signed area in the   ZX–plane
//   n.z  is the signed area in the   XY–plane
//
// Whichever component is largest in magnitude gives the most numerically
// stable orientation test.
//
inline bool TriangleIsCCW(const float3 a,
                          const float3 b,
                          const float3 c)
{
    float3 n = cross(b - a, c - a); // un-normalised triangle normal

    float ax = fabs(n.x), ay = fabs(n.y), az = fabs(n.z);

    // Pick the dominant component and use its sign.
    // (No branch on the sign itself – only on which component is dominant.)
    return (ax >= ay && ax >= az) ? (n.x > 0.0f) : // project to YZ
               (ay >= az) ? (n.y > 0.0f) :  // project to ZX
               (n.z > 0.0f); // project to XY
}

// 5.4.1 Test if point p lies inside ccw-specified convex n-gon given by vertices v[]
int PointInConvexPolygon(float3 p, int n, float3 v[])
{
    // Do binary search over polygon vertices to find the fan triangle
    // (v[0], v[low], v[high]) the point p lies within the near sides of
    int low = 0, high = n;
    do {
        int mid = (low + high) / 2;
        if (TriangleIsCCW(v[0], v[mid], p))
            low = mid;
        else
            high = mid;
    } while (low + 1 < high);

    // If point outside last (or first) edge, then it is not inside the n-gon
    if (low == 0 || high == n) return 0;

    // p is inside the polygon if it is left of
    // the directed edge from v[low] to v[high]
    return TriangleIsCCW(v[low], v[high], p);
}

// Test if point P lies inside the counterclockwise 3D triangle ABC
int PointInTriangle(float3 p, float3 a, float3 b, float3 c)
{
    // Translate point and triangle so that point lies at origin
    a -= p; b -= p; c -= p;

    float ab = dot(a, b);
    float ac = dot(a, c);
    float bc = dot(b, c);
    float cc = dot(c, c);
    // Make sure plane normals for pab and pbc point in the same direction
    if (bc * ac - cc * ab < 0.0f) return 0;
    // Make sure plane normals for pab and pca point in the same direction
    float bb = dot(b, b);
    if (ab * bc - ac * bb < 0.0f) return 0;
    // Otherwise P must be in (or on) the triangle
    return 1;
}

// Compute the 2D pseudo cross product dot(Perp(u), v)
float cross2D(float2 u, float2 v)
{
    return u.y * v.x - u.x * v.y;
}

// Test if 2D point P lies inside the counterclockwise 2D triangle ABC
int PointInTriangle2D(float2 p, float2 a, float2 b, float2 c)
{
    // If P to the right of AB then outside triangle
    if (cross2D(p - a, b - a) < 0.0f) return 0;
    // If P to the right of BC then outside triangle
    if (cross2D(p - b, c - b) < 0.0f) return 0;
    // If P to the right of CA then outside triangle
    if (cross2D(p - c, a - c) < 0.0f) return 0;
    // Otherwise P must be in (or on) the triangle
    return 1;
}

// Test if 2D point P lies inside 2D triangle ABC
int PointInTriangle2DDoubleSided(float2 p, float2 a, float2 b, float2 c)
{
    float pab = cross2D(p - a, b - a);
    float pbc = cross2D(p - b, c - b);
    // If P left of one of AB and BC and right of the other, not inside triangle
    if (!SameSign(pab, pbc)) return 0;
    float pca = cross2D(p - c, a - c);
    // If P left of one of AB and CA and right of the other, not inside triangle
    if (!SameSign(pab, pca)) return 0;
    // P left or right of all edges, so must be in (or on) the triangle
    return 1;
}

//Section 5.4.3 :
// Test if point p inside polyhedron given as the intersection volume of n halfspaces
int TestPointPolyhedron(float3 p, Plane h[], int n)
{
    for (int i = 0; i < n; i++) {
        // Exit with ëno containmentí if p ever found outside a halfspace
        if (DistPointPlane(p, h[i]) > 0.0f) return 0;
    }
    // p inside all halfspaces, so p must be inside intersection volume
    return 1;
}

//Section 5.4.4 :

// // Given planes p1 and p2, compute line L = p+t*d of their intersection.
// // Return 0 if no such line exists
// int IntersectPlanes(Plane p1, Plane p2, out float3 p, out float3 d)
// {
//     // Compute direction of intersection line
//     d = cross(p1.n, p2.n);

//     // If d is zero, the planes are parallel (and separated)
//     // or coincident, so theyíre not considered intersecting
//     if (dot(d, d) < EPSILON) return 0;

//     float d11 = dot(p1.n, p1.n);
//     float d12 = dot(p1.n, p2.n);
//     float d22 = dot(p2.n, p2.n);

//     float denom = d11 * d22 - d12 * d12;
//     float k1 = (p1.d * d22 - p2.d * d12) / denom;
//     float k2 = (p2.d * d11 - p1.d * d12) / denom;
//     p = k1 * p1.n + k2 * p2.n;
//     return 1;
// }


// Given planes p1 and p2, compute line L = p+t*d of their intersection.
// Return 0 if no such line exists
int IntersectPlanes(Plane p1, Plane p2, out float3 p, out float3 d)
{
    // Compute direction of intersection line
    d = cross(p1.n, p2.n);

    // If d is (near) zero, the planes are parallel (and separated)
    // or coincident, so theyíre not considered intersecting
    float denom = dot(d, d);
    if (denom < EPSILON) {
        p = float3(0.0f);
        d = float3(0.0f);
        return 0;
    }

    // Compute point on intersection line
    p = cross(p1.d * p2.n - p2.d * p1.n, d) / denom;
    return 1;
}

// Section 5.4.5

// // Compute the point p at which the three planes p1, p2 and p3 intersect (if at all)
// int IntersectPlanes(Plane p1, Plane p2, Plane p3, out float3 p)
// {
//     float3 m1 = float3(p1.n.x, p2.n.x, p3.n.x);
//     float3 m2 = float3(p1.n.y, p2.n.y, p3.n.y);
//     float3 m3 = float3(p1.n.z, p2.n.z, p3.n.z);

//     float3 u = cross(m2, m3);
//     float denom = dot(m1, u);
//     if (abs(denom) < EPSILON) return 0; // Planes do not intersect in a point
//     float3 d = float3(p1.d, p2.d, p3.d);
//     float3 v = cross(m1, d);
//     float ood = 1.0f / denom;
//     p.x = dot(d, u) * ood;
//     p.y = dot(m3, v) * ood;
//     p.z = -dot(m2, v) * ood;
//     return 1;
// }

// Compute the point p at which the three planes p1, p2, and p3 intersect (if at all)
int IntersectPlanes(Plane p1, Plane p2, Plane p3, out float3 p)
{
    float3 u = cross(p2.n, p3.n);
    float denom = dot(p1.n, u);
    if (abs(denom) < EPSILON) {
        p = float3(0.0f);
        return 0; // Planes do not intersect in a point
    }
    p = (p1.d * u + cross(p1.n, p3.d * p2.n - p2.d * p3.n)) / denom;
    return 1;
}

// Section 5.5.1 :

// Intersect sphere s0 moving in direction d over time interval t0 <= t <= t1, against
// a stationary sphere s1. If found intersecting, return time t of collision
int TestMovingSphereSphere(Sphere s0, float3 d, float t0, float t1, Sphere s1, out float t)
{
    // Compute sphere bounding motion of s0 during time interval from t0 to t1
    Sphere b;
    float mid = (t0 + t1) * 0.5f;
    b.c = s0.c + d * mid;
    b.r = (mid - t0) * length(d) + s0.r;
    // If bounding sphere not overlapping s1, then no collision in this interval
    if (!bool(ch4::TestSphereSphere(b, s1))) {
        t = 0.0f;
        return 0;
    }

    // Cannot rule collision out: recurse for more accurate testing. To terminate the
    // recursion, collision is assumed when time interval becomes sufficiently small
    if (t1 - t0 < INTERVAL_EPSILON) {
        t = t0;
        return 1;
    }

    // Recursively test first half of interval; return collision if detected
    if (bool(TestMovingSphereSphere(s0, d, t0, mid, s1, t))) return 1;

    // Recursively test second half of interval
    return TestMovingSphereSphere(s0, d, mid, t1, s1, t);
}

// // Test collision between objects a and b moving over the time interval
// // [startTime, endTime]. When colliding, time of collision is returned in hitTime
// int IntervalCollision(Object a, Object b, float startTime, float endTime, out float hitTime)
// {
//     // Compute the maximum distance objects a and b move over the time interval
//     float maxMoveA = MaximumObjectMovementOverTime(a, startTime, endTime);
//     float maxMoveB = MaximumObjectMovementOverTime(b, startTime, endTime);
//     float maxMoveDistSum = maxMoveA + maxMoveB;
//     // Exit if distance between a and b at start larger than sum of max movements
//     float minDistStart = MinimumObjectDistanceAtTime(a, b, startTime);
//     if (minDistStart > maxMoveDistSum) return 0;
//     // Exit if distance between a and b at end larger than sum of max movements
//     float minDistEnd = MinimumObjectDistanceAtTime(a, b, endTime);
//     if (minDistEnd > maxMoveDistSum) return 0;

//     // Cannot rule collision out: recurse for more accurate testing. To terminate the
//     // recursion, collision is assumed when time interval becomes sufficiently small
//     if (endTime - startTime < INTERVAL_EPSILON) {
//         hitTime = startTime;
//         return 1;
//     }
//     // Recursively test first half of interval; return collision if detected
//     float midTime = (startTime + endTime) * 0.5f;
//     if (IntervalCollision(a, b, startTime, midTime, hitTime)) return 1;
//     // Recursively test second half of interval
//     return IntervalCollision(a, b, midTime, endTime, hitTime);
// }

// Section 5.5.3 :
// Intersect sphere s with movement vector v with plane p. If intersecting
// return time t of collision and point q at which sphere hits plane
int IntersectMovingSpherePlane(Sphere s, float3 v, Plane p, out float t, out float3 q)
{
    // Compute distance of sphere center to plane
    float dist = dot(p.n, s.c) - p.d;
    if (abs(dist) <= s.r) {
        // The sphere is already overlapping the plane. Set time of
        // intersection to zero and q to sphere center
        t = 0.0f;
        q = s.c;
        return 1;
    } else {
        float denom = dot(p.n, v);
        if (denom * dist >= 0.0f) {
            // No intersection as sphere moving parallel to or away from plane
            t = 0.0f;
            q = float3(0.0f);
            return 0;
        } else {
            // Sphere is moving towards the plane

            // Use +r in computations if sphere in front of plane, else -r
            float r = dist > 0.0f ? s.r : -s.r;
            t = (r - dist) / denom;
            q = s.c + t * v - r * p.n;
            return 1;
        }
    }
}

// Test if sphere with radius r moving from a to b intersects with plane p
int TestMovingSpherePlane(float3 a, float3 b, float r, Plane p)
{
    // Get the distance for both a and b from plane p
    float adist = dot(a, p.n) - p.d;
    float bdist = dot(b, p.n) - p.d;
    // Intersects if on different sides of plane (distances have different signs)
    if (adist * bdist < 0.0f) return 1;
    // Intersects if start or end position within radius from plane
    if (abs(adist) <= r || abs(bdist) <= r) return 1;
    // No intersection
    return 0;
}

// Section 5.5.5 :

// int TestMovingSphereSphere(Sphere s0, Sphere s1, float3 v0, float3 v1, out float t)
// {
//     float3 s = s1.c - s0.c; // float3 between sphere centers
//     float3 v = v1 - v0;     // Relative motion of s1 with respect to stationary s0
//     float r = s1.r + s0.r;  // Sum of sphere radii
//     float c = dot(s, s) - r * r;
//     if (c < 0.0f) {
//         // Spheres initially overlapping so exit directly
//         t = 0.0f;
//         return 1;
//     }
//     float a = dot(v, v);
//     if (a < EPSILON) return 0; // Spheres not moving relative each other
//     float b = dot(v, s);
//     if (b >= 0.0f) return 0; // Spheres not moving towards each other
//     float d = b * b - a * c;
//     if (d < 0.0f) return 0; // No real-valued root, spheres do not intersect

//     t = (-b - sqrt(d)) / a;
//     return 1;
// }

int TestMovingSphereSphere(Sphere s0, Sphere s1, float3 v0, float3 v1, out float t)
{
    // Expand sphere s1 by the radius of s0
    s1.r += s0.r;
    // Subtract movement of s1 from both s0 and s1, making s1 stationary
    float3 v = v0 - v1;
    // Can now test directed segment s = s0.c + tv, v = (v0-v1)/||v0-v1|| against
    // the expanded sphere for intersection
    float3 q;
    float vlen = length(v);
    if (bool(IntersectRaySphere(s0.c, v / vlen, s1, t, q))) {
        return t <= vlen;
    }
    return 0;
}

// Support function that returns the AABB vertex with index n
float3 Corner(AABB_MM b, int n)
{
    float3 p;
    p.x = (bool(n & 1) ? b.max.x : b.min.x);
    p.y = (bool(n & 2) ? b.max.y : b.min.y);
    p.z = (bool(n & 4) ? b.max.z : b.min.z);
    return p;
}

int IntersectSegmentSphere(float3 sa,
                           float3 sb,
                           float3 center,
                           float radius,
                           out float tOut)
{
    float3 m = sa - center;
    float3 d = sb - sa; // segment direction
    float b = dot(m, d);
    float c = dot(m, m) - radius * radius;

    // Early out when the start point is inside the sphere.
    if (c <= 0.0f) { tOut = 0.0f; return 1; }

    // Exit if segment is pointing away from sphere or parallel.
    if (b > 0.0f) { tOut = 0.0f; return 0; }

    float discr = b * b - dot(d, d) * c;
    if (discr < 0.0f) { tOut = 0.0f; return 0; }

    tOut = (-b - sqrt(discr)) / dot(d, d); // first root in [0,1]
    return (tOut >= 0.0f && tOut <= 1.0f) ? 1 : 0;
}

// -----------------------------------------------------------------------------
// Segment × capsule  (cylinder  +  two hemispherical caps)
// -----------------------------------------------------------------------------
int IntersectSegmentCapsule(const Segment seg,
                            const float3 capA, // capsule axis end‑point A
                            const float3 capB, // capsule axis end‑point B
                            float r,
                            out float tOut)
{
    float bestT = 1.0f + 1.0f; // >1 → “no hit yet”
    bool hitAny = false;

    // 1. Finite cylinder (body)
    float t = 0.0f;
    if (bool(IntersectSegmentCylinder(seg.a, seg.b, capA, capB, r, t)) &&
        t >= 0.0f && t <= 1.0f)
    {
        bestT = t;
        hitAny = true;
    }

    // 2. Hemispherical end‑cap at capA
    if (bool(IntersectSegmentSphere(seg.a, seg.b, capA, r, t)) &&
        t >= 0.0f && t <= bestT)
    {
        bestT = t;
        hitAny = true;
    }

    // 3. Hemispherical end‑cap at capB
    if (bool(IntersectSegmentSphere(seg.a, seg.b, capB, r, t)) &&
        t >= 0.0f && t <= bestT)
    {
        bestT = t;
        hitAny = true;
    }

    if (!hitAny) { tOut = 0.0f; return 0; }
    tOut = bestT;
    return 1;
}

// Section 5.5.7 : 

int IntersectMovingSphereAABB(Sphere s, float3 d, AABB_MM b, out float t)
{
    // Compute the AABB resulting from expanding b by sphere radius r
    AABB_MM e = b;
    e.min.x -= s.r; e.min.y -= s.r; e.min.z -= s.r;
    e.max.x += s.r; e.max.y += s.r; e.max.z += s.r;

    // Intersect ray against expanded AABB e. Exit with no intersection if ray
    // misses e, else get intersection point p and time t as result
    float3 p;
    if (!bool(IntersectRayAABB(s.c, d, e, t, p)) || t > 1.0f)
        return 0;

    // Compute which min and max faces of b the intersection point p lies
    // outside of. Note, u and v cannot have the same bits set and
    // they must have at least one bit set amongst them
    int u = 0, v = 0;
    if (p.x < b.min.x) u |= 1;
    if (p.x > b.max.x) v |= 1;
    if (p.y < b.min.y) u |= 2;
    if (p.y > b.max.y) v |= 2;
    if (p.z < b.min.z) u |= 4;
    if (p.z > b.max.z) v |= 4;

    // ëOrí all set bits together into a bit mask (note: here u + v == u | v)
    int m = u + v;

    // Define line segment [c, c+d] specified by the sphere movement
    Segment seg = Segment(s.c, s.c + d);

    // If all 3 bits set (m == 7) then p is in a vertex region
    if (m == 7) {
        // Must now intersect segment [c, c+d] against the capsules of the three
        // edges meeting at the vertex and return the best time, if one or more hit
        float tmin = kMaxFloat;
        if (bool(IntersectSegmentCapsule(seg, Corner(b, v), Corner(b, v ^ 1), s.r, t)))
            tmin = min(t, tmin);
        if (bool(IntersectSegmentCapsule(seg, Corner(b, v), Corner(b, v ^ 2), s.r, t)))
            tmin = min(t, tmin);
        if (bool(IntersectSegmentCapsule(seg, Corner(b, v), Corner(b, v ^ 4), s.r, t)))
            tmin = min(t, tmin);
        if (tmin == kMaxFloat) return 0; // No intersection
        t = tmin;
        return 1; // Intersection at time t == tmin
    }
    // If only one bit set in m, then p is in a face region
    if ((m & (m - 1)) == 0) {
        // Do nothing. Time t from intersection with
        // expanded box is correct intersection time
        return 1;
    }
    // p is in an edge region. Intersect against the capsule at the edge
    return IntersectSegmentCapsule(seg, Corner(b, u ^ 7), Corner(b, v), s.r, t);
}



// Section 5.5.8 :

// Intersect AABBs ëaí and ëbí moving with constant velocities va and vb.
// On intersection, return time of first and last contact in tfirst and tlast
int IntersectMovingAABBAABB(AABB_MM a, AABB_MM b, float3 va, float3 vb, out float tfirst, out float tlast)
{
    // Exit early if ëaí and ëbí initially overlapping
    if (bool(ch4::TestAABB_MM(a, b))) {
        tfirst = tlast = 0.0f;
        return 1;
    }

    // Use relative velocity; effectively treating 'a' as stationary
    float3 v = vb - va;

    // Initialize times of first and last contact
    tfirst = 0.0f;
    tlast = 1.0f;

    // For each axis, determine times of first and last contact, if any
    for (int i = 0; i < 3; i++) {
        if (v[i] < 0.0f) {
            if (b.max[i] < a.min[i]) return 0; // Nonintersecting and moving apart
            if (a.max[i] < b.min[i]) tfirst = max((a.max[i] - b.min[i]) / v[i], tfirst);
            if (b.max[i] > a.min[i]) tlast = min((a.min[i] - b.max[i]) / v[i], tlast);
        }
        if (v[i] > 0.0f) {
            if (b.min[i] > a.max[i]) return 0; // Nonintersecting and moving apart
            if (b.max[i] < a.min[i]) tfirst = max((a.min[i] - b.max[i]) / v[i], tfirst);
            if (a.max[i] > b.min[i]) tlast = min((a.max[i] - b.min[i]) / v[i], tlast);
        }

        // No overlap possible if time of first contact occurs after time of last contact
        if (tfirst > tlast) return 0;
    }

    return 1;
}

} // namespace ch5