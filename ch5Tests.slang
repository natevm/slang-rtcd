// Chapter 5 test harness
import rtcdCommon;
import ch5;

bool feq(float a, float b) { return abs(a - b) < 1e-5f; }
bool feq3(float3 a, float3 b) { return feq(a.x, b.x) && feq(a.y, b.y) && feq(a.z, b.z); }

#define EXPECT(cond, name)                                                   \
    if (!(cond)) { printf("FAIL: " name "\n"); failures++; }               \
    else         { printf("PASS: " name "\n"); }

export __extern_cpp int main(int argc, NativeString *argv)
{
    int failures = 0;
    using namespace rtcdCommon;
    using namespace ch5;

    // Section 5.1.1: Closest point on plane & distance
    {
        Plane plane = { float3(0, 0, 1), 0.0f }; // z=0 plane
        float3 q = float3(0, 0, 5);
        float3 cp = ClosestPtPointPlane(q, plane);
        EXPECT(feq3(cp, float3(0, 0, 0)), "ClosestPtPointPlane basic");
        EXPECT(feq(DistPointPlane(q, plane), 5.0f), "DistPointPlane basic");
    }

    // Section 5.1.2: Closest point on segment
    {
        float3 a = float3(0,0,0), b = float3(2,0,0);
        float3 c = float3(1,1,0);
        float t; float3 d;
        ClosestPtPointSegment(c, a, b, t, d);
        EXPECT(feq(t, 0.5f) && feq3(d, float3(1,0,0)), "ClosestPtPointSegment midpoint");
        ClosestPtPointSegment(float3(-1,1,0), a, b, t, d);
        EXPECT(feq(t, 0.0f) && feq3(d, a), "ClosestPtPointSegment clamp a");
        ClosestPtPointSegment(float3(3,2,0), a, b, t, d);
        EXPECT(feq(t, 1.0f) && feq3(d, b), "ClosestPtPointSegment clamp b");
    }

    // Section 5.1.2.1: Squared distance point-segment
    {
        float3 a = float3(0,0,0), b = float3(2,0,0), c = float3(1,1,0);
        EXPECT(feq(SqDistPointSegment(a,b,c), 1.0f), "SqDistPointSegment perpendicular");
        EXPECT(feq(SqDistPointSegment(a,b,a), 0.0f), "SqDistPointSegment at endpoint");
    }

    // Section 5.1.3: Closest point and sqdist for AABB
    {
        float3 p = float3(2, -1, 3);
        AABB_MM b = { float3(0,0,0), float3(1,1,1) };
        float3 q;
        ClosestPtPointAABB(p, b, q);
        EXPECT(feq3(q, float3(1,0,1)), "ClosestPtPointAABB basic");
        EXPECT(feq(SqDistPointAABB(p, b), 6.0f), "SqDistPointAABB basic");
    }

    // Section 5.1.4: Closest point and sqdist for OBB
    {
        float3 p = float3(2,2,2);
        OBB b = { float3(0,0,0), { float3(1,0,0), float3(0,1,0), float3(0,0,1) }, float3(1,1,1) };
        float3 q;
        ClosestPtPointOBB(p, b, q);
        EXPECT(feq3(q, float3(1,1,1)), "ClosestPtPointOBB basic");
        EXPECT(feq(SqDistPointOBB(p, b), 3.0f), "SqDistPointOBB basic");
    }

    // Section 5.1.4.2: Closest point on Rect
    {
        float3 p = float3(2, 2, 0);
        // Non-axis-aligned local axes for rectangle
        float3 u0 = normalize(float3(1, 1, 0));
        float3 u1 = normalize(float3(-1, 1, 0));
        Rect r = { float3(0,0,0), { u0, u1 }, {1,1} };
        float3 q;
        ClosestPtPointRect(p, r, q);
        EXPECT(feq3(q, float3(0.70710678f, 0.70710678f, 0.0f)),
               "ClosestPtPointRect struct basic");
        // Test using explicit corner-based overload
        ClosestPtPointRect(p, float3(0,0,0), float3(2,0,0), float3(0,2,0), q);
        EXPECT(feq3(q, float3(2,2,0)), "ClosestPtPointRect corners basic");
    }

    // Section 5.1.5: Closest point on triangle
    {
        float3 a = float3(0,0,0), b = float3(1,0,0), c = float3(0,1,0);
        float3 p = float3(0.25f, 0.25f, 0);
        float3 q = ClosestPtPointTriangle(p, a, b, c);
        EXPECT(feq3(q, p), "ClosestPtPointTriangle inside face");
        p = float3(2,0.1f, 0);
        q = ClosestPtPointTriangle(p, a, b, c);
        EXPECT(feq3(q, float3(1,0,0)), "ClosestPtPointTriangle project AB");
        p = float3(-1,-1, 0);
        q = ClosestPtPointTriangle(p, a, b, c);
        EXPECT(feq3(q, a), "ClosestPtPointTriangle vertex A");
    }

    // Section 5.1.6: Closest point on tetrahedron
    {
        float3 a = float3(0,0,0);
        float3 b = float3(1,0,0);
        float3 c = float3(0,1,0);
        float3 d = float3(0,0,1);
        // Inside tetrahedron
        float3 p = float3(0.2f, 0.2f, 0.2f);
        float3 q = ClosestPtPointTetrahedron(p, a, b, c, d);
        EXPECT(feq3(q, p), "ClosestPtPointTetrahedron inside");
        // Outside tetrahedron near face abc
        p = float3(0.3f, 0.3f, -1.0f);
        q = ClosestPtPointTetrahedron(p, a, b, c, d);
        EXPECT(feq3(q, ClosestPtPointTriangle(p, a, b, c)), "ClosestPtPointTetrahedron face abc");
    }

    // Section 5.1.7: Closest points on two segments
    {
        float3 p1 = float3(0,0,0), q1 = float3(1,0,0);
        float3 p2 = float3(0,1,0), q2 = float3(1,1,0);
        float s, t; float3 c1, c2;
        float d2 = ClosestPtSegmentSegment(p1, q1, p2, q2, s, t, c1, c2);
        EXPECT(feq(d2, 1.0f), "ClosestPtSegmentSegment parallel");
        EXPECT(feq(s, 0.0f) && feq(t, 0.0f),      "ClosestPtSegmentSegment s/t");
        EXPECT(feq3(c1, p1) && feq3(c2, p2),      "ClosestPtSegmentSegment c1/c2");
    }

    // Section 5.1.9.1: Signed2DTriArea
    {
        float3 a = float3(0,0,0), b = float3(1,0,0), c = float3(0,1,0);
        EXPECT(feq(Signed2DTriArea(a, b, c),  1.0f), "Signed2DTriArea ccw");
        EXPECT(feq(Signed2DTriArea(a, c, b), -1.0f), "Signed2DTriArea cw");
        EXPECT(feq(Signed2DTriArea(a, a, b),  0.0f), "Signed2DTriArea degenerate");
    }

    // Section 5.1.9.2: Test intersection of two segments in 2D
    {
        float3 a = float3(0,0,0), b = float3(2,2,0);
        float3 c = float3(0,2,0), d = float3(2,0,0);
        float t; float3 p;
        EXPECT(Test2DSegmentSegment(a, b, c, d, t, p) == 1,
               "Test2DSegmentSegment intersect");
        EXPECT(feq(t, 0.5f) && feq3(p, float3(1,1,0)),
               "Test2DSegmentSegment t/p");
        // Non-intersecting
        c = float3(0,2,0); d = float3(2,2,0);
        EXPECT(Test2DSegmentSegment(a, b, c, d, t, p) == 0,
               "Test2DSegmentSegment disjoint");
    }

    // Section 5.2.2: Sphere-plane and halfspace tests
    {
        Sphere s = Sphere( float3(0, 0, 0), 1.0f );
        Plane p = Plane( float3(0,0,1), 0.5f ); // z=0.5 plane
        EXPECT(TestSpherePlane(s, p) == 1,       "TestSpherePlane intersect");
        p.d = 2.0f;
        EXPECT(TestSpherePlane(s, p) == 0,       "TestSpherePlane disjoint");

        // Sphere not fully behind z=0 plane (half overlaps)
        p = Plane( float3(0,0,1), 0.0f );
        EXPECT(InsideSpherePlane(s, p) == 0,     "InsideSpherePlane not fully behind");
        EXPECT(TestSphereHalfspace(s, p) == 1,   "TestSphereHalfspace basic");

        // Sphere fully behind z=2 plane
        p = Plane( float3(0,0,1), 2.0f );
        EXPECT(InsideSpherePlane(s, p) == 1,     "InsideSpherePlane fully behind");
    }

    // Section 5.2.3: OBB and AABB vs. plane
    {
        float3 cMin = float3(0,0,0), cMax = float3(1,1,1);
        AABB_MM b = { cMin, cMax };
        // Plane at z=0 intersects box [0..1]
        Plane p = { float3(0,0,1), 0.0f };
        EXPECT(TestAABBPlane(b, p) == 1,  "TestAABBPlane intersect");
        // Plane at z=3 above the box
        p.d = 3.0f;
        EXPECT(TestAABBPlane(b, p) == 0,  "TestAABBPlane disjoint");

        // Rotated OBB to avoid axis-aligned trivial case
        float3 u0 = normalize(float3(1, 1, 0));
        float3 u1 = normalize(float3(-1, 1, 0));
        float3 u2 = float3(0, 0, 1);
        OBB obb = { float3(0,0,0), { u0, u1, u2 }, float3(1,1,1) };
        // Plane at z=0 intersects OBB centered at origin
        p = { float3(0,0,1), 0.0f };
        EXPECT(TestOBBPlane(obb, p) == 1,  "TestOBBPlane intersect");
        // Plane at z=3 above the box
        p.d = 3.0f;
        EXPECT(TestOBBPlane(obb, p) == 0,  "TestOBBPlane disjoint");
    }

    // Section 5.2.5: Sphere vs AABB
    {
        Sphere s = { float3(0,0,0), 1.0f };
        AABB_MM b = { float3(-2,-2,-2), float3(-1,-1,-1) };
        EXPECT(TestSphereAABB(s, b) == 0, "TestSphereAABB disjoint");
        b = { float3(-0.5f,-0.5f,-0.5f), float3(0.5f,0.5f,0.5f) };
        EXPECT(TestSphereAABB(s, b) == 1, "TestSphereAABB intersect");
        float3 p;
        b = { float3(-2,-2,-2), float3(-1,-1,-1) };
        EXPECT(TestSphereAABB(s, b, p) == 0 && feq3(p, float3(-1,-1,-1)),
               "TestSphereAABB wpt disjoint");
        b = { float3(-0.5f,-0.5f,-0.5f), float3(0.5f,0.5f,0.5f) };
        EXPECT(TestSphereAABB(s, b, p) == 1 && feq3(p, float3(0.0f,0.0f,0.0f)),
               "TestSphereAABB wpt intersect (center inside)");
    }

    // Section 5.2.6: Sphere vs. OBB with closest-point
    {
        Sphere s = { float3(0,0,0), 1.0f };
        // Rotated OBB to test non-axis-aligned case
        float3 u0 = normalize(float3(1, 1, 0));
        float3 u1 = normalize(float3(-1, 1, 0));
        float3 u2 = float3(0, 0, 1);
        OBB b = { float3(0,0,0), { u0, u1, u2 }, float3(1,1,1) };
        float3 p;
        EXPECT(TestSphereOBB(s, b, p) == 1 && feq3(p, float3(0,0,0)),
               "TestSphereOBB intersect (inside)");
        s.c = float3(0,0,3);
        EXPECT(TestSphereOBB(s, b, p) == 0 && feq3(p, float3(0,0,1)),
               "TestSphereOBB disjoint");
    }

    // Section 5.2.7: Sphere vs triangle with closest-point
    {
        Sphere s = { float3(0,0,0), 1.0f };
        float3 a = float3(0,0,2), b = float3(1,0,2), c = float3(0,1,2);
        float3 p;
        EXPECT(TestSphereTriangle(s, a, b, c, p) == 0 && feq3(p, float3(0,0,2)),
               "TestSphereTriangle disjoint");
        s.c = float3(0,0,1.9f);
        EXPECT(TestSphereTriangle(s, a, b, c, p) == 1 && feq3(p, float3(0,0,2)),
               "TestSphereTriangle intersect");
    }

    // Section 5.2.8: Sphere vs polygon tests (to be implemented)
    printf("SKIP: Section 5.2.8 TestSpherePolygon (to be implemented)\n");

    // Section 5.2.9: Triangle vs AABB
    {
        float3 v0 = float3(2, 2, 2);
        float3 v1 = float3(3, 2, 2);
        float3 v2 = float3(2, 3, 2);
        AABB_MM b = { float3(0,0,0), float3(1,1,1) };
        EXPECT(TestTriangleAABB(v0, v1, v2, b) == 0,
               "TestTriangleAABB disjoint");

        v0 = float3(0.5f, 0.5f, 0.5f);
        v1 = float3(1.5f, 0.5f, 0.5f);
        v2 = float3(0.5f, 1.5f, 0.5f);
        EXPECT(TestTriangleAABB(v0, v1, v2, b) == 1,
               "TestTriangleAABB intersect");
    }

    // Section 5.2.9 errata check: verify proper box-shift for triangle face test
    {
        float3 v0 = float3(2.5f, 3.5f, 4.75f);
        float3 v1 = float3(2.6f, 3.5f, 4.75f);
        float3 v2 = float3(2.5f, 3.6f, 4.75f);
        AABB_MM b = { float3(2.0f, 3.0f, 4.0f), float3(3.0f, 4.0f, 5.0f) };
        EXPECT(TestTriangleAABB(v0, v1, v2, b) == 1,
               "TestTriangleAABB errata shift");
    }

    // Section 5.3.1: Intersect segment vs. plane
    {
        float3 a = float3(0, 0, 0);
        float3 b = float3(0, 0, 2);
        Plane p = { float3(0, 0, 1), 1.0f }; // z=1 plane
        float t; float3 q;
        EXPECT(IntersectSegmentPlane(a, b, p, t, q) == 1
               && feq(t, 0.5f) && feq3(q, float3(0,0,1)),
               "IntersectSegmentPlane plane-basic");
        EXPECT(IntersectSegmentPlane(float3(0,0,2), float3(0,0,3), p, t, q) == 0,
               "IntersectSegmentPlane plane-miss");

        // Overload: segment vs. triangle-plane
        float3 d = float3(0,0,1), e = float3(1,0,1), f = float3(0,1,1);
        EXPECT(IntersectSegmentPlane(a, b, d, e, f, t, q) == 1
               && feq(t, 0.5f) && feq3(q, float3(0,0,1)),
               "IntersectSegmentPlane tri-basic");
        EXPECT(IntersectSegmentPlane(float3(0,0,2), float3(0,0,3), d, e, f, t, q) == 0,
               "IntersectSegmentPlane tri-miss");
    }

    // Section 5.3.2: Intersect ray vs. sphere
    {
        Sphere s = Sphere(float3(0,0,0), 1.0f);
        float t; float3 q;
        float3 p0 = float3(0,0,-2);
        float3 d0 = normalize(float3(0,0,1));
        EXPECT(IntersectRaySphere(p0, d0, s, t, q) == 1
               && feq(t, 1.0f) && feq3(q, float3(0,0,-1)),
               "IntersectRaySphere basic-hit");
        EXPECT(IntersectRaySphere(float3(0,0,2), d0, s, t, q) == 0,
               "IntersectRaySphere basic-miss");
    }
    {
        Sphere s = Sphere(float3(0,0,0), 1.0f);
        float3 p0 = float3(0,0,-2);
        float3 d0 = normalize(float3(0,0,1));
        EXPECT(TestRaySphere(p0, d0, s) == 1,
               "TestRaySphere basic-hit");
        EXPECT(TestRaySphere(float3(0,0,2), d0, s) == 0,
               "TestRaySphere basic-miss");
    }
    // Section 5.3.3: Intersect ray vs. AABB
    {
        AABB_MM b = { float3(0,0,0), float3(1,1,1) };
        float3 p0 = float3(-1.0f, 0.5f, 0.5f);
        float3 d0 = normalize(float3(1,0,0));
        float tmin; float3 q;
        EXPECT(IntersectRayAABB(p0, d0, b, tmin, q) == 1
               && feq(tmin, 1.0f) && feq3(q, float3(0.0f,0.5f,0.5f)),
               "IntersectRayAABB basic-hit");
        EXPECT(IntersectRayAABB(float3(2.0f,0.5f,0.5f), d0, b, tmin, q) == 0,
               "IntersectRayAABB basic-miss");
    }

    // Section 5.3.3: Test segment vs. AABB
    {
        AABB_MM b2 = { float3(0,0,0), float3(1,1,1) };
        float3 p0_2 = float3(-0.5f, 0.5f, 0.5f);
        float3 p1_2 = float3(1.5f,  0.5f, 0.5f);
        EXPECT(TestSegmentAABB(p0_2, p1_2, b2) == 1,
               "TestSegmentAABB hit-through");
        EXPECT(TestSegmentAABB(float3(-1.0f,2.0f,0.5f), float3(-0.5f,2.0f,0.5f), b2) == 0,
               "TestSegmentAABB miss-outside");
        EXPECT(TestSegmentAABB(float3(0.2f,0.2f,0.2f), float3(0.8f,0.8f,0.8f), b2) == 1,
               "TestSegmentAABB inside");
    }

    // Section 5.3.4: Intersect line vs. triangle
    {
        float3 a = float3(0,0,0), b = float3(1,0,0), c = float3(0,1,0);
        float3 p0 = float3(0.2f,0.3f,-1.0f), p1 = float3(0.2f,0.3f,1.0f);
        float u, v, w;
        EXPECT(IntersectLineTriangle(p0, p1, a, b, c, u, v, w, true) == 1
               && feq(u + v + w, 1.0f),
               "IntersectLineTriangle basic-hit");
        EXPECT(IntersectLineTriangle(float3(2,2,-1), float3(2,2,1), a, b, c,
                                     u, v, w, true) == 0,
               "IntersectLineTriangle basic-miss");
    }

    // Section 5.3.4: double-sided vs single-sided line-triangle
    {
        float3 a = float3(0,0,0), b = float3(1,0,0), c = float3(0,1,0);
        float3 p0 = float3(0.2f, 0.3f, -1.0f), p1 = float3(0.2f, 0.3f,  1.0f);
        float u, v, w;
        EXPECT(IntersectLineTriangle(p0, p1, a, b, c, u, v, w) == 0,
               "IntersectLineTriangle single-sided back-face");
        EXPECT(IntersectLineTriangle(p0, p1, a, b, c, u, v, w, true) == 1
               && feq(u + v + w, 1.0f),
               "IntersectLineTriangle double-sided back-face");
    }

    // Section 5.3.5: Intersect line vs. quad
    {
        printf("SKIP: Section 5.3.5 tests (IntersectLineQuad not working yet)\n");
        float3 a = float3(0,0,0), b = float3(1,0,0), c = float3(1,1,0), d = float3(0,1,0);
        float3 p0 = float3(0.2f,0.3f,1.0f), p1 = float3(0.2f,0.3f,-1.0f);
        float3 r;
        EXPECT(IntersectLineQuad(p0, p1, a, b, c, d, r) == 1
               && feq3(r, float3(0.2f, 0.3f, 0.0f)),
               "IntersectLineQuad basic-hit");
        EXPECT(IntersectLineQuad(float3(2, 2, -1), float3(2, 2, 1), a, b, c, d, r) == 0,
               "IntersectLineQuad basic-miss");
    }

    // Section 5.4: Plane and polyhedron intersection tests
    {
        Plane p1 = Plane(float3(1,0,0), 1.0f);
        Plane p2 = Plane(float3(0,1,0), 2.0f);
        float3 p, d;
        EXPECT(IntersectPlanes(p1, p2, p, d) == 1 &&
               feq3(p, float3(1,2,0)) &&
               feq3(normalize(d), float3(0,0,1)),
               "IntersectPlanes two-plane basic");
        EXPECT(IntersectPlanes(p1, p1, p, d) == 0 &&
               feq3(p, float3(0,0,0)) &&
               feq3(d, float3(0,0,0)),
               "IntersectPlanes two-plane parallel");

        Plane p3 = Plane(float3(0,0,1), 3.0f);
        EXPECT(IntersectPlanes(p1, p2, p3, p) == 1 &&
               feq3(p, float3(1,2,3)),
               "IntersectPlanes three-plane basic");
        EXPECT(IntersectPlanes(p1, p2, p2, p) == 0 &&
               feq3(p, float3(0,0,0)),
               "IntersectPlanes three-plane degenerate");

        Plane h[6];
        h[0] = Plane(float3(1,0,0), 1.0f);
        h[1] = Plane(float3(0,1,0), 1.0f);
        h[2] = Plane(float3(0,0,1), 1.0f);
        h[3] = Plane(float3(-1,0,0), 0.0f);
        h[4] = Plane(float3(0,-1,0), 0.0f);
        h[5] = Plane(float3(0,0,-1), 0.0f);
        EXPECT(TestPointPolyhedron(float3(0.5f,0.5f,0.5f), h, 6) == 1,
               "TestPointPolyhedron inside cube");
        EXPECT(TestPointPolyhedron(float3(2.0f,0.5f,0.5f), h, 6) == 0,
               "TestPointPolyhedron outside cube");
    }

    // Section 5.5: Moving-sphere and swept intersection tests
    {
        Sphere s;
        float t; float3 q;
        // IntersectMovingSpherePlane
        s = Sphere(float3(0,0,0), 1.0f);
        float3 v = float3(0,0,1);
        Plane pl = Plane(float3(0,0,1), 5.0f);
        EXPECT(IntersectMovingSpherePlane(s, v, pl, t, q) == 1 &&
               feq(t, 4.0f) && feq3(q, float3(0,0,5)),
               "IntersectMovingSpherePlane basic-hit");
        EXPECT(IntersectMovingSpherePlane(s, v, Plane(float3(0,1,0), 5.0f), t, q) == 0 &&
               feq(t, 0.0f) && feq3(q, float3(0.0f)),
               "IntersectMovingSpherePlane miss");

        // TestMovingSpherePlane
        EXPECT(TestMovingSpherePlane(float3(0,0,0), float3(0,0,5), 1.0f, Plane(float3(0,0,1), 3.0f)) == 1,
               "TestMovingSpherePlane basic-hit");
        EXPECT(TestMovingSpherePlane(float3(0,0,0), float3(0,0,5), 1.0f, Plane(float3(0,0,1), -5.0f)) == 0,
               "TestMovingSpherePlane miss");

        // TestMovingSphereSphere (relative motion)
        Sphere s0 = Sphere(float3(0,0,0), 0.0f);
        Sphere s1 = Sphere(float3(0,0,5), 1.0f);
        EXPECT(TestMovingSphereSphere(s0, s1, float3(0,0,10), float3(0,0,0), t) == 1 &&
               feq(t, 4.0f),
               "TestMovingSphereSphere relative-hit");
        EXPECT(TestMovingSphereSphere(s0, s1, float3(0,0,1), float3(0,0,0), t) == 0,
               "TestMovingSphereSphere relative-miss");

        // Section 5.5.7: IntersectMovingSphereAABB
        s = Sphere(float3(0,0,0), 1.0f);
        float3 d2 = float3(1,0,0);
        AABB_MM bb = { float3(2,-1,-1), float3(3,1,1) };
        EXPECT(IntersectMovingSphereAABB(s, d2, bb, t) == 1 && feq(t, 1.0f),
               "IntersectMovingSphereAABB hit");
        bb = { float3(3,-1,-1), float3(4,1,1) };
        EXPECT(IntersectMovingSphereAABB(s, d2, bb, t) == 0,
               "IntersectMovingSphereAABB miss");
    }

    // Section 5.5.8: Intersect moving AABB vs. AABB
    {
        AABB_MM a = { float3(0,0,0), float3(1,1,1) };
        AABB_MM b = { float3(3,0,0), float3(4,1,1) };
        float3 va = float3(0,0,0), vb = float3(-2,0,0);
        float tfirst, tlast;
        EXPECT(IntersectMovingAABBAABB(a, b, va, vb, tfirst, tlast) == 1 &&
               feq(tfirst, 1.0f) && feq(tlast, 1.0f),
               "IntersectMovingAABBAABB basic-collide");
        vb = float3(2,0,0);
        EXPECT(IntersectMovingAABBAABB(a, b, va, vb, tfirst, tlast) == 0,
               "IntersectMovingAABBAABB miss");
    }

    // Section 5.6.2: Intersect segment vs. cylinder
    {
        float3 p = float3(0,0,0), q = float3(0,0,10);
        float radius = 1.0f;
        float t;

        // Miss below bottom cap
        EXPECT(IntersectSegmentCylinder(float3(0,0,-2), float3(0,0,-1), p, q, radius, t) == 0,
               "IntersectSegmentCylinder miss-below");
        // Miss above top cap
        EXPECT(IntersectSegmentCylinder(float3(0,0,11), float3(0,0,12), p, q, radius, t) == 0,
               "IntersectSegmentCylinder miss-above");
        // Parallel to axis but outside radius
        EXPECT(IntersectSegmentCylinder(float3(2, 0, 1), float3(2, 0, 5), p, q, radius, t) == 0,
               "IntersectSegmentCylinder parallel-miss");
        // Parallel to axis, intersect bottom cap  ➜ expect t ≈ 1/6
        EXPECT(IntersectSegmentCylinder(float3(0.5f, 0, -1), float3(0.5f, 0, 5),
                                        p, q, radius, t) == 1 && feq(t, 1.0f / 6.0f),
               "IntersectSegmentCylinder parallel-bottom-hit");
        // Barrel *hit* with narrower end‑points (rename or change expectation)
        EXPECT(IntersectSegmentCylinder(float3(1.5f, 0, 5), float3(-1.5f, 0, 5),
                                        p, q, radius, t) == 1 &&
                   feq(t, 1.0f / 6.0f), "IntersectSegmentCylinder barrel-hit-narrow");
        // Barrel hit
        EXPECT(IntersectSegmentCylinder(float3(2,0,5), float3(-2,0,5), p, q, radius, t) == 1
               && feq(t, 0.25f),
               "IntersectSegmentCylinder barrel-hit");
        // Barrel miss
        EXPECT(IntersectSegmentCylinder(float3(1.5f, 0, 5), float3(3.0f, 0, 5), p, q, radius, t) == 0,
               "IntersectSegmentCylinder barrel-miss");
    }

    // Section 5.6.3: Intersect segment vs. triangle
    {
        float3 a = float3(0,0,0), b = float3(1,0,0), c = float3(0,1,0);
        // segment passes through triangle interior
        float3 p0 = float3(0.2f,0.2f,1.0f), p1 = float3(0.2f,0.2f, -1.0f);
        float u, v, w, t;
        EXPECT(IntersectSegmentTriangle(p0, p1, a, b, c, u, v, w, t) == 1
               && feq(u + v + w, 1.0f) && feq(t, 0.5f),
               "IntersectSegmentTriangle basic-hit");
        // miss: segment parallel above triangle
        EXPECT(IntersectSegmentTriangle(float3(0.2f,0.2f,1.1f), float3(0.2f,0.2f,2.0f),
                                        a, b, c, u, v, w, t) == 0,
               "IntersectSegmentTriangle basic-miss");
    }

    if (failures == 0)
    printf("ALL TESTS PASSED\n");
    else
        printf("%d TESTS FAILED\n", failures);

    return failures;
}