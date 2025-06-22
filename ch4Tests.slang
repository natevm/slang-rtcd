#include "ch4.slang"

bool feq(float a, float b) { return abs(a - b) < 1e-5f; }
bool feq3(float3 a, float3 b) { return feq(a.x, b.x) && feq(a.y, b.y) && feq(a.z, b.z); }

#define EXPECT(cond, name)                                                   \
    if (!(cond)) { printf("FAIL: " name "\n"); failures++; }               \
    else         { printf("PASS: " name "\n"); }

export __extern_cpp int main(int argc, NativeString *argv)
{
    int failures = 0;
    using namespace ch4;

    // Test AABB_MM
    AABB_MM a_mm1 = { float3(0.0f, 0.0f, 0.0f), float3(1.0f, 1.0f, 1.0f) };
    AABB_MM b_mm1 = { float3(0.5f, 0.5f, 0.5f), float3(2.0f, 2.0f, 2.0f) };
    EXPECT(TestAABB_MM(a_mm1, b_mm1) == 1, "TestAABB_MM overlapping");
    AABB_MM b_mm2 = { float3(2.0f, 2.0f, 2.0f), float3(3.0f, 3.0f, 3.0f) };
    EXPECT(TestAABB_MM(a_mm1, b_mm2) == 0, "TestAABB_MM disjoint");

    // Test AABB_MD
    AABB_MD a_md1 = { float3(0.0f, 0.0f, 0.0f), {1.0f, 1.0f, 1.0f} };
    AABB_MD b_md1 = { float3(0.5f, 0.5f, 0.5f), {2.0f, 2.0f, 2.0f} };
    EXPECT(TestAABB_MD(a_md1, b_md1) == 1, "TestAABB_MD overlapping");
    AABB_MD b_md2 = { float3(2.0f, 2.0f, 2.0f), {1.0f, 1.0f, 1.0f} };
    EXPECT(TestAABB_MD(a_md1, b_md2) == 0, "TestAABB_MD disjoint");

    // Test AABB_CF
    AABB_CF a_cf1 = { float3(0.5f, 0.5f, 0.5f), {0.5f, 0.5f, 0.5f} };
    AABB_CF b_cf1 = { float3(1.0f, 1.0f, 1.0f), {1.0f, 1.0f, 1.0f} };
    EXPECT(TestAABB_CF(a_cf1, b_cf1) == 1, "TestAABB_CF overlapping");
    AABB_CF b_cf2 = { float3(2.0f, 2.0f, 2.0f), {0.25f, 0.25f, 0.25f} };
    EXPECT(TestAABB_CF(a_cf1, b_cf2) == 0, "TestAABB_CF disjoint");

    // Test AABBUtils: extreme points along a direction
    {
        float3 pts3[4] = { float3(-1,0,0), float3(1,0,0), float3(0,-1,0), float3(0,1,0) };
        int imin, imax;
        ExtremePointsAlongDirection(float3(1,0,0), pts3, 4, imin, imax);
        EXPECT(imin == 0 && imax == 1, "ExtremePointsAlongDirection X-axis");
        ExtremePointsAlongDirection(float3(0,1,0), pts3, 4, imin, imax);
        EXPECT(imin == 2 && imax == 3, "ExtremePointsAlongDirection Y-axis");
    }

    // Test AABBUtils: UpdateAABB transforms
    {
        AABB_MM a_mm = { float3(1,2,3), float3(2,3,4) };
        float3x3 m = float3x3(1,0,0, 0,1,0, 0,0,1);
        float3 t = float3(1,1,1);
        AABB_MM b_mm;
        UpdateAABB(a_mm, m, t, b_mm);
        EXPECT(feq3(b_mm.min, float3(2,3,4)) && feq3(b_mm.max, float3(3,4,5)),
               "UpdateAABB min/max identity+translate");

        AABB_CF a_cf = { float3(1,2,3), {1,2,3} };
        AABB_CF b_cf;
        UpdateAABB(a_cf, m, t, b_cf);
        EXPECT(feq3(b_cf.c, float3(2,3,4)) &&
               feq(b_cf.r[0], 1.0f) && feq(b_cf.r[1], 2.0f) && feq(b_cf.r[2], 3.0f),
               "UpdateAABB center/radius identity+translate");
    }

    // Section 4.3.1: TestSphereSphere
    {
        Sphere s1 = { float3(0,0,0), 1.0f };
        Sphere s2 = { float3(1,0,0), 1.0f };
        EXPECT(TestSphereSphere(s1, s2) == 1, "TestSphereSphere overlap");
        Sphere s3 = { float3(3,0,0), 1.0f };
        EXPECT(TestSphereSphere(s1, s3) == 0, "TestSphereSphere disjoint");
    }

    // Section 4.3.2 & 4.3.3: SphereFromDistantPoints, SphereOfSphereAndPt, RitterSphere
    {
        float3 pts2[2] = { float3(0,0,0), float3(2,0,0) };
        Sphere sfd;
        SphereFromDistantPoints(sfd, &pts2[0], 2);
        EXPECT(feq3(sfd.c, float3(1,0,0)) && feq(sfd.r, 1.0f), "SphereFromDistantPoints simple");

        Sphere ssp = { float3(0,0,0), 1.0f };
        SphereOfSphereAndPt(ssp, float3(3,0,0));
        EXPECT(feq3(ssp.c, float3(1.0f,0,0)) && feq(ssp.r, 2.0f), "SphereOfSphereAndPt simple");

        Sphere sr = { float3(0,0,0), 0.0f };
        RitterSphere(sr, &pts2[0], 2);
        EXPECT(feq3(sr.c, float3(1,0,0)) && feq(sr.r, 1.0f), "RitterSphere simple");
    }

    // Section 4.3.4: RitterIterative
    {
        float3 pts2[2] = { float3(0,0,0), float3(2,0,0) };
        Sphere si = { float3(0,0,0), 0.0f };
        RitterIterative(si, &pts2[0], 2);
        EXPECT(feq3(si.c, float3(1,0,0)) && feq(si.r, 1.0f), "RitterIterative simple");
    }

    // Section 4.3.3: Variance
    {
        float arr[3] = { 1.0f, 2.0f, 3.0f };
        EXPECT(feq(Variance(&arr[0], 3), 2.0f/3.0f), "Variance simple");
    }

    // Section 4.3.3: CovarianceMatrix
    {
        float3 pts3[2] = { float3(0,0,0), float3(2,0,0) };
        float3x3 cov;
        CovarianceMatrix(cov, pts3, 2);
        EXPECT(feq(cov[0][0], 1.0f) && feq(cov[1][1], 0.0f) && feq(cov[2][2], 0.0f),
               "CovarianceMatrix diag simple");
        EXPECT(feq(cov[0][1], 0.0f) && feq(cov[0][2], 0.0f) && feq(cov[1][2], 0.0f),
               "CovarianceMatrix off-diagonals");
    }

    // Section 4.3.5: WelzlSphere
    {
        float3 pts2[2] = { float3(0, 0, 0), float3(2, 0, 0) };
        Sphere sw = WelzlSphere(pts2, 2);
        EXPECT(feq3(sw.c, float3(1,0,0)) && feq(sw.r, 1.0f), "WelzlSphere simple");
    }
    {   // colinear three-point case
        float3 pts3[3] = { float3(0,0,0), float3(2,0,0), float3(4,0,0) };
        Sphere sw3 = WelzlSphere(pts3, 3);
        EXPECT(feq3(sw3.c, float3(2,0,0)) && feq(sw3.r, 2.0f), "WelzlSphere colinear");
    }

    { // triangle circumsphere case
        float3 pts4[3] = { float3(0, 0, 0), float3(0, 2, 0), float3(2, 0, 0) };
        Sphere sw4 = WelzlSphere(pts4, 3);
        EXPECT(feq3(sw4.c, float3(1, 1, 0)) && feq(sw4.r, sqrt(2.0f)), "WelzlSphere triangle");
    }

    // Section 4.4.1: TestOBBOBB
    {
        OBB a = { float3(0,0,0), { float3(1,0,0), float3(0,1,0), float3(0,0,1) }, float3(1,2,3) };
        OBB b = { float3(1,1,1), { float3(1,0,0), float3(0,1,0), float3(0,0,1) }, float3(1,2,3) };
        EXPECT(TestOBBOBB(a, b) == 1, "TestOBBOBB overlapping");
        b.c = float3(5,5,5);
        EXPECT(TestOBBOBB(a, b) == 0, "TestOBBOBB disjoint");
    }

    // Section 4.4.4: MinAreaRect
    {
        float2 pts2D[4] = { float2(0.0f, 0.0f), float2(2.0f, 0.0f),
                            float2(2.0f, 1.0f), float2(0.0f, 1.0f) };
        float2 center, u0, u1;
        float area = MinAreaRect(pts2D, 4, center, u0, u1);
        EXPECT(feq(area, 2.0f), "MinAreaRect area axis-aligned");
        EXPECT(feq(center.x, 1.0f) && feq(center.y, 0.5f),
               "MinAreaRect center axis-aligned");
        EXPECT((feq(abs(u0.x), 1.0f) && feq(u0.y, 0.0f)) ||
               (feq(abs(u0.y), 1.0f) && feq(u0.x, 0.0f)),
               "MinAreaRect u0 axis-aligned");
        EXPECT((feq(abs(u1.x), 1.0f) && feq(u1.y, 0.0f)) ||
               (feq(abs(u1.y), 1.0f) && feq(u1.x, 0.0f)),
               "MinAreaRect u1 axis-aligned");

    }

    // Section 4.5: capsule tests (pending chapter 5 routines)
    printf("SKIP: Section 4.5 tests (requires chapter 5 routines)\n");

    // Section 4.6.1: Slab struct only (no test function)
    printf("SKIP: Section 4.6.1 slab tests (no test function)\n");

    // Section 4.6.3: k-DOP test (requires chapter 6 KDOP struct)
    printf("SKIP: Section 4.6.3 KDOP tests (requires chapter 6 KDOP struct)\n");

    // Section 4.6.4: ComputeDOP8 tests
    {
        float3 pts3[2] = { float3(0,0,0), float3(1,1,1) };
        DOP8 d;
        ComputeDOP8(pts3, 2, d);
        EXPECT(feq(d.min[0], 0.0f) && feq(d.max[0], 3.0f), "ComputeDOP8 axis0");
        EXPECT(feq(d.min[1], 0.0f) && feq(d.max[1], 1.0f), "ComputeDOP8 axis1");
        EXPECT(feq(d.min[2], 0.0f) && feq(d.max[2], 1.0f), "ComputeDOP8 axis2");
        EXPECT(feq(d.min[3], 0.0f) && feq(d.max[3], 1.0f), "ComputeDOP8 axis3");
    }

    if (failures == 0)
        printf("ALL TESTS PASSED\n");
    else
        printf("%d TESTS FAILED\n", failures);

    return failures;
}
