#include "ch3.slang"

bool feq(float a, float b) { return abs(a - b) < 1e-5f; }
bool feq3(float3 a, float3 b) { return feq(a.x, b.x) && feq(a.y, b.y) && feq(a.z, b.z); }

#define EXPECT(cond, name)                                                  \
    if (!(cond)) { printf("FAIL: " name "\n"); failures++; }               \
    else         { printf("PASS: " name "\n"); }

export __extern_cpp int main(int argc, NativeString *argv)
{
    int failures = 0;

    float area = ch3::TriArea2D(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f);
    EXPECT(feq(area, 1.0f), "TriArea2D right triangle");

    float u, v, w;
    ch3::BarycentricReferenceViaCramers(
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(0.0f, 0.0f, 0.0f),
        u, v, w);
    EXPECT(feq3(float3(u, v, w), float3(1, 0, 0)), "BarycentricRef p=a");
    ch3::BarycentricReferenceViaCramers(
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(1.0f, 0.0f, 0.0f),
        u, v, w);
    EXPECT(feq3(float3(u, v, w), float3(0, 1, 0)), "BarycentricRef p=b");
    ch3::BarycentricReferenceViaCramers(
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(0.0f, 1.0f, 0.0f),
        u, v, w);
    EXPECT(feq3(float3(u, v, w), float3(0, 0, 1)), "BarycentricRef p=c");
    ch3::BarycentricReferenceViaCramers(
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(1.0f/3.0f, 1.0f/3.0f, 0.0f),
        u, v, w);
    EXPECT(feq3(float3(u, v, w), float3(1.0f/3.0f, 1.0f/3.0f, 1.0f/3.0f)), "BarycentricRef centroid");

    ch3::Barycentric(
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(0.0f, 0.0f, 0.0f),
        u, v, w);
    EXPECT(feq3(float3(u, v, w), float3(1, 0, 0)), "Barycentric p=a");
    ch3::Barycentric(
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(1.0f, 0.0f, 0.0f),
        u, v, w);
    EXPECT(feq3(float3(u, v, w), float3(0, 1, 0)), "Barycentric p=b");
    ch3::Barycentric(
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(0.0f, 1.0f, 0.0f),
        u, v, w);
    EXPECT(feq3(float3(u, v, w), float3(0, 0, 1)), "Barycentric p=c");

    EXPECT(ch3::TestPointTriangle(
               float3(0.2f, 0.2f, 0.0f), float3(0.0f,0.0f,0.0f), float3(1.0f,0.0f,0.0f), float3(0.0f,1.0f,0.0f)),
           "TestPointTriangle inside");
    EXPECT(!ch3::TestPointTriangle(
               float3(1.1f, 0.1f, 0.0f), float3(0.0f,0.0f,0.0f), float3(1.0f,0.0f,0.0f), float3(0.0f,1.0f,0.0f)),
           "TestPointTriangle outside");

    ch3::Plane p = ch3::ComputePlane(
        float3(0.0f,0.0f,0.0f), float3(1.0f,0.0f,0.0f), float3(0.0f,1.0f,0.0f));
    EXPECT(feq3(p.n, float3(0,0,1)), "ComputePlane normal");
    EXPECT(feq(p.d, 0.0f), "ComputePlane d");

    EXPECT(ch3::IsConvexQuad(float3(0,0,0), float3(1,0,0), float3(1,1,0), float3(0,1,0)), "IsConvexQuad convex");
    EXPECT(!ch3::IsConvexQuad(float3(0,0,0), float3(1,0,0), float3(0.5,0.5,0), float3(0,1,0)), "IsConvexQuad concave");

    float2 pts2[3] = { float2(0.0f,0.0f), float2(0.2f,1.0f), float2(0.2f,0.5f) };
    int idx = ch3::PointFarthestFromEdge(
        float2(0.0f,0.0f), float2(1.0f,0.0f), pts2, 3);
    EXPECT(idx == 1, "PointFarthestFromEdge simple");

    if (failures == 0)
        printf("ALL TESTS PASSED\n");
    else
        printf("%d TESTS FAILED\n", failures);

    return failures;
}