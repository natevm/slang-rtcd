import rtcdCommon;
#pragma once
namespace ch4 {
using namespace rtcdCommon;

// Section 4.2: Axis-Aligned Bounding Boxes (AABB)
// region R = { (x, y, z) | min.x<=x<=max.x, min.y<=y<=max.y, min.z<=z<=max.z }

// Test intersection of two AABBs (min/max form)
// Exit with no intersection if separated along an axis
int TestAABB_MM(AABB_MM a, AABB_MM b)
{
    if (a.max.x < b.min.x || a.min.x > b.max.x) return 0;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return 0;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return 0;
    // Overlapping on all axes means AABBs are intersecting
    return 1;
}

// Test intersection of two AABBs (min + diameter form)
// Exit with no intersection if a.min - b.min lies outside [0, b.d] or [-a.d,0]
int TestAABB_MD(AABB_MD a, AABB_MD b)
{
    float t;
    if ((t = a.min.x - b.min.x) > b.d[0] || -t > a.d[0]) return 0;
    if ((t = a.min.y - b.min.y) > b.d[1] || -t > a.d[1]) return 0;
    if ((t = a.min.z - b.min.z) > b.d[2] || -t > a.d[2]) return 0;
    return 1;
}

// Test intersection of two AABBs (center + radius form)
// Exit with no intersection if center separation exceeds radius sum on any axis
int TestAABB_CF(AABB_CF a, AABB_CF b)
{
    if (abs(a.c.x - b.c.x) > (a.r[0] + b.r[0])) return 0;
    if (abs(a.c.y - b.c.y) > (a.r[1] + b.r[1])) return 0;
    if (abs(a.c.z - b.c.z) > (a.r[2] + b.r[2])) return 0;
    return 1;
}

// Section 4.2.4 & 4.2.6: Utility functions for AABBs

// Returns indices imin and imax into pts[] array of the least and
// most, respectively, distant points along the direction dir
// Returns indices of least and most distant points along dir
void ExtremePointsAlongDirection(float3 dir, float3 pts[], int n,
                                out int imin, out int imax)
{
    float minproj =  kMaxFloat;
    float maxproj = -kMaxFloat;
    imin = imax = 0;
    for (int i = 0; i < n; ++i)
    {
        // Project vector from origin to point onto direction vector
        float proj = dot(pts[i], dir);
        // Keep track of least distant point along direction vector
        if (proj < minproj) { minproj = proj; imin = i; }
        // Keep track of most distant point along direction vector
        if (proj > maxproj) { maxproj = proj; imax = i; }
    }
}

// Transform AABB a by the matrix m and translation t,
// find maximum extents, and store result into AABB b (min/max form)
void UpdateAABB(AABB_MM a, float3x3 m, float3 t, out AABB_MM b)
{
    // For all three axes
    for (int i = 0; i < 3; ++i)
    {
        // Start by adding in translation
        b.min[i] = t[i];
        b.max[i] = t[i];
        // Form extent by summing smaller and larger terms respectively
        for (int j = 0; j < 3; ++j)
        {
            float e = m[i][j] * a.min[j];
            float f = m[i][j] * a.max[j];
            if (e < f) { b.min[i] += e; b.max[i] += f; }
            else      { b.min[i] += f; b.max[i] += e; }
        }
    }
}

// Transform AABB a by the matrix m and translation t,
// find maximum extents, and store result into AABB b (center/radius form)
void UpdateAABB(AABB_CF a, float3x3 m, float3 t, out AABB_CF b)
{
    for (int i = 0; i < 3; ++i)
    {
        b.c[i] = t[i];
        b.r[i] = 0.0f;
        for (int j = 0; j < 3; ++j)
        {
            b.c[i] += m[i][j] * a.c[j];
            b.r[i] += abs(m[i][j]) * a.r[j];
        }
    }
}

// Section 4.3: Sphere routines and utilities (Sphere from rtcdCommon)

// Test if two spheres intersect
int TestSphereSphere(Sphere a, Sphere b)
{
    // Calculate squared distance between centers
    float3 d = a.c - b.c;
    float dist2 = dot(d, d);
    // Spheres intersect if squared distance is less than squared sum of radii
    float radiusSum = a.r + b.r;
    return dist2 <= radiusSum * radiusSum;
}

// Compute indices min and max into pt[] array of the most separated points
// defining the AABB encompassing the point set.
void MostSeparatedPointsOnAABB(out int minIdx, out int maxIdx, float3* pt, int numPts)
{
    // First find most extreme points along principal axes
    int minx = 0, maxx = 0, miny = 0, maxy = 0, minz = 0, maxz = 0;
    for (int i = 1; i < numPts; ++i) {
        if (pt[i].x < pt[minx].x) minx = i;
        if (pt[i].x > pt[maxx].x) maxx = i;
        if (pt[i].y < pt[miny].y) miny = i;
        if (pt[i].y > pt[maxy].y) maxy = i;
        if (pt[i].z < pt[minz].z) minz = i;
        if (pt[i].z > pt[maxz].z) maxz = i;
    }

    // Compute the squared distances for the three pairs of points
    float3 dx = pt[maxx] - pt[minx]; float dist2x = dot(dx, dx);
    float3 dy = pt[maxy] - pt[miny]; float dist2y = dot(dy, dy);
    float3 dz = pt[maxz] - pt[minz]; float dist2z = dot(dz, dz);

    // Pick the pair (minIdx, maxIdx) of points most distant
    minIdx = minx; maxIdx = maxx;
    if (dist2y > dist2x && dist2y > dist2z) { minIdx = miny; maxIdx = maxy; }
    if (dist2z > dist2x && dist2z > dist2y) { minIdx = minz; maxIdx = maxz; }
}

// Given Sphere s and Point p, update s (if needed) to just encompass p
void SphereOfSphereAndPt(inout Sphere s, float3 p)
{
    // Compute squared distance between point and sphere center
    float3 d = p - s.c;
    float dist2 = dot(d, d);
    // Only update s if point p is outside it
    if (dist2 > s.r * s.r) {
        float dist = sqrt(dist2);
        float newRadius = (s.r + dist) * 0.5f;
        float k = (newRadius - s.r) / dist;
        s.r = newRadius;
        s.c += d * k;
    }
}

// Get sphere encompassing two approximately most distant points
void RitterSphere(inout Sphere s, float3* pt, int numPts)
{
    SphereFromDistantPoints(s, pt, numPts);
    // Grow sphere to include all points
    for (int i = 0; i < numPts; ++i)
        SphereOfSphereAndPt(s, pt[i]);
}

// A simple linear congruential generator for random numbers
static int g_randSeed = 123456789;
int NextRand()
{
    g_randSeed = g_randSeed * 1664525 + 1013904223;
    return g_randSeed;
}

// Iterative refinement of bounding sphere (Section 4.3.4)
void RitterIterative(inout Sphere s, float3* pt, int numPts)
{
    const int NUM_ITER = 8;
    RitterSphere(s, pt, numPts);
    Sphere s2 = s;
    for (int k = 0; k < NUM_ITER; ++k)
    {
        // Shrink sphere somewhat to make it an underestimate (not bound)
        s2.r = s2.r * 0.95f;

        // Make sphere bound data again
        for (int i = 0; i < numPts; ++i)
        {
            // Swap pt[i] with pt[j], where j randomly from interval [i+1,numPts-1]
            {
                int j = i + NextRand() % (numPts - i);
                float3 tmp = pt[i];
                pt[i] = pt[j];
                pt[j] = tmp;
            }
            SphereOfSphereAndPt(s2, pt[i]);
        }

        // Update s whenever a tighter sphere is found
        if (s2.r < s.r)
            s = s2;
    }
}

// Section 4.3.5: Welzl's algorithm for smallest enclosing sphere (iterative)
bool PointInsideSphere(float3 p, Sphere s)
{
    float3 d = p - s.c;
    float dist2 = dot(d, d);
    return dist2 <= s.r * s.r;
}

static const int WF_START = 0;
static const int WF_AFTER_FIRST = 1;
static const int WL_MAX_STACK = 64;

struct WLFrame { int numPts; int numSos; int state; };

// Convenience ------------------------------------------------------------
inline Sphere sphereFromSupport(float3 sos[4], uint sosCount)
{
    switch (sosCount)
    {
    case 0: return Sphere();
    case 1: return Sphere(sos[0]);
    case 2: return Sphere(sos[0], sos[1]);
    case 3: return Sphere(sos[0], sos[1], sos[2]);
    default: return Sphere(sos[0], sos[1], sos[2], sos[3]);
    }
}

// -----------------------------------------------------------------------------
// Non‑recursive Welzl ----------------------------------------------------------
// -----------------------------------------------------------------------------
static const uint kMaxPts = 256;       // adjust to your worst‑case
static const uint kMaxDepth = kMaxPts; // recursion depth ≤ numPts

struct Frame
{
    uint nPts;  // how many pts of the original set are still “active”
    uint nSOS;  // current size of support set
    uint state; // 0 = entry, 1 = after 1st recurse, 2 = after 2nd recurse
};

Sphere WelzlSphere<let kMaxPts : uint>(in float3 pts[kMaxPts], uint numPts)
{
    //----------------------------------------------------------------------
    // Runtime working storage (all fixed‑size, therefore Slang‑legal)
    //----------------------------------------------------------------------
    Frame callStack[kMaxDepth];
    Sphere sphereStack[kMaxDepth]; // return values per frame
    float3 sos[4];                  // support set (≤4 by theorem)

    //----------------------------------------------------------------------
    // Push root frame -----------------------------------------------------
    //----------------------------------------------------------------------
    uint sp = 0;
    callStack[sp++] = Frame(numPts, 0, 0);

    Sphere result = Sphere(); // will hold the final answer

    //----------------------------------------------------------------------
    // Manual depth‑first traversal ---------------------------------------
    //----------------------------------------------------------------------
    while (bool(sp))
    {
        const Frame f = callStack[sp - 1]; // top‑of‑stack alias

        if (f.nPts == 0)
        {
            // Base‑case – build the exact sphere of the support set
            result = sphereFromSupport(sos, f.nSOS);
            sphereStack[sp - 1] = result; // pass value upward
            --sp;                         // “return”
            continue;
        }

        //--------------------------------------------------------------
        // STATE 0: mimic first recursive call
        //--------------------------------------------------------------
        if (f.state == 0)
        {
            callStack[sp - 1].state = 1;                                    // next time resume at 1
            callStack[sp++] = Frame(f.nPts - 1, f.nSOS, 0); // first “recursive” call
            continue;
        }

        //--------------------------------------------------------------
        // STATE 1: we are back from the first call, decision time
        //--------------------------------------------------------------
        if (f.state == 1)
        {
            Sphere smallest = sphereStack[sp]; // result of child frame
            float3 candidate = pts[f.nPts - 1];

            if (PointInsideSphere(candidate, smallest))
            {
                // Point is inside – propagate sphere upward
                result = smallest;
                sphereStack[sp - 1] = result;
                --sp; // “return”
                continue;
            }

            // Otherwise enlarge support and perform second “recursive” call
            sos[f.nSOS] = candidate;
            callStack[sp - 1].state = 2; // resume at state 2
            callStack[sp++] = Frame(f.nPts - 1, f.nSOS + 1, 0);
            continue;
        }

        //--------------------------------------------------------------
        // STATE 2: back from second call, bubble result up
        //--------------------------------------------------------------
        if (f.state == 2)
        {
            result = sphereStack[sp]; // child’s answer
            sphereStack[sp - 1] = result;
            --sp; // “return”
        }
    }

    return result; // final sphere
}

// Compute sphere from two most separated points on AABB
void SphereFromDistantPoints(inout Sphere s, float3* pt, int numPts)
{
    int imin, imax;
    MostSeparatedPointsOnAABB(imin, imax, pt, numPts);

    // Set up sphere to just encompass these two points
    s.c = (pt[imin] + pt[imax]) * 0.5f;
    s.r = dot(pt[imax] - s.c, pt[imax] - s.c);
    s.r = sqrt(s.r);
}

// Section 4.3.3: Compute variance of a set of 1D values
float Variance(float* x, int n)
{
    float u = 0.0f;
    for (int i = 0; i < n; ++i)
        u += x[i];
    u /= n;
    float s2 = 0.0f;
    for (int i = 0; i < n; ++i)
        s2 += (x[i] - u) * (x[i] - u);
    return s2 / n;
}

// Compute covariance matrix of a set of points
// Section 4.3.3: Compute covariance matrix of a set of points
// cov = 1/n * sum_i (pt[i] - centroid) * (pt[i] - centroid)^T
void CovarianceMatrix(out float3x3 cov, float3 pt[], int numPts)
{
    float oon = 1.0f / numPts;
    float3 c = float3(0.0f, 0.0f, 0.0f);
    // Compute the center of mass (centroid) of the points
    for (int i = 0; i < numPts; ++i)
        c += pt[i];
    c *= oon;

    float e00 = 0.0f, e11 = 0.0f, e22 = 0.0f, e01 = 0.0f, e02 = 0.0f, e12 = 0.0f;
    // Compute covariance of translated points
    for (int i = 0; i < numPts; ++i)
    {
        float3 p = pt[i] - c;
        e00 += p.x * p.x;
        e11 += p.y * p.y;
        e22 += p.z * p.z;
        e01 += p.x * p.y;
        e02 += p.x * p.z;
        e12 += p.y * p.z;
    }
    cov[0][0] = e00 * oon;
    cov[1][1] = e11 * oon;
    cov[2][2] = e22 * oon;
    cov[0][1] = cov[1][0] = e01 * oon;
    cov[0][2] = cov[2][0] = e02 * oon;
    cov[1][2] = cov[2][1] = e12 * oon;
}

// Section 4.3.6: Symmetric Schur helper for Jacobi rotations
// Compute the c = cos(theta) and s = sin(theta) for Jacobi rotation to annihilate a[p][q]
void SymSchur2(inout float3x3 a, int p, int q, out float c, out float s)
{
    if (abs(a[p][q]) > 0.0001f)
    {
        float r = (a[q][q] - a[p][p]) / (2.0f * a[p][q]);
        float t;
        if (r >= 0.0f)
            t = 1.0f / (r + sqrt(1.0f + r*r));
        else
            t = -1.0f / (-r + sqrt(1.0f + r*r));
        c = 1.0f / sqrt(1.0f + t*t);
        s = t * c;
    }
    else
    {
        c = 1.0f;
        s = 0.0f;
    }
}

// Computes the eigenvectors and eigenvalues of a symmetric 3×3 matrix 'a'
// using the classic Jacobi method (Golub–Van Loan, Matrix Computations).
// On exit, 'v' holds eigenvectors and diagonal of 'a' holds eigenvalues.
void Jacobi(inout float3x3 a, inout float3x3 v)
{
    // initialize v to identity
    for (int i = 0; i < 3; ++i)
    {
        v[i][0] = v[i][1] = v[i][2] = 0.0f;
        v[i][i] = 1.0f;
    }

    const int MAX_ITER = 50;
    float prevoff = 0.0f;
    for (int n = 0; n < MAX_ITER; ++n)
    {
        // find largest off-diagonal element a[p][q]
        int p = 0, q = 1;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (i != j && abs(a[i][j]) > abs(a[p][q]))
                    { p = i; q = j; }

        float c, s;
        SymSchur2(a, p, q, c, s);

        // build Jacobi rotation matrix J
        float3x3 J;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                J[i][j] = (i == j) ? 1.0f : 0.0f;
        J[p][p] = c; J[p][q] = s;
        J[q][p] = -s; J[q][q] = c;

        // accumulate eigenvectors and diagonalize 'a'
        v = v * J;
        a = (transpose(J) * a) * J;

        // compute off-diagonal norm
        float off = 0.0f;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (i != j) off += a[i][j] * a[i][j];

        if (n > 2 && off >= prevoff)
            break;
        prevoff = off;
    }
}

// Section 4.3.7: EigenSphere – sphere from maximum spread eigendirection (Alg. 4.4)
void EigenSphere(inout Sphere eigSphere, float3 pt[], int numPts)
{
    float3x3 m, v;
    // Compute covariance of points
    CovarianceMatrix(m, pt, numPts);
    // Find eigenvectors (in v) and eigenvalues (on diag of m)
    Jacobi(m, v);

    // Select principal axis (largest eigenvalue)
    int maxc = 0;
    float maxe = abs(m[0][0]);
    float f;
    if ((f = abs(m[1][1])) > maxe) { maxc = 1; maxe = f; }
    if ((f = abs(m[2][2])) > maxe) { maxc = 2; maxe = f; }
    float3 axis = float3(v[0][maxc], v[1][maxc], v[2][maxc]);

    // Project points onto axis to find extreme pair
    int imin, imax;
    ExtremePointsAlongDirection(axis, pt, numPts, imin, imax);
    float3 pMin = pt[imin];
    float3 pMax = pt[imax];
    float span = sqrt(dot(pMax - pMin, pMax - pMin));
    eigSphere.r = span * 0.5f;
    eigSphere.c = (pMin + pMax) * 0.5f;
}

// Section 4.3.7: RitterEigenSphere – grow eigen-based sphere (Alg. 4.5)
void RitterEigenSphere(inout Sphere s, float3 pt[], int numPts)
{
    // Start from eigen-based initial guess
    EigenSphere(s, pt, numPts);
    // Grow sphere to bound all points
    for (int i = 0; i < numPts; ++i)
        SphereOfSphereAndPt(s, pt[i]);
}

// Section 4.4: Oriented Bounding Boxes (OBB)
// Region R = { x | x = c + r*u[0] + s*u[1] + t*u[2], |r|<=e[0], |s|<=e[1], |t|<=e[2] }
// Section 4.4: OBB routines (OBB from rtcdCommon)

// Section 4.4.1: Separating-axis test for two OBBs
int TestOBBOBB(OBB a, OBB b)
{
    float ra, rb;
    float3x3 R, AbsR;

    // Compute rotation matrix expressing b in a's coordinate frame
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = dot(a.u[i], b.u[j]);

    // Compute translation vector t
    float3 t = b.c - a.c;
    // Bring translation into a's frame
    t = float3(dot(t, a.u[0]), dot(t, a.u[1]), dot(t, a.u[2]));

    // Add in epsilon term to counteract arithmetic errors when edges are parallel
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            AbsR[i][j] = abs(R[i][j]) + EPSILON;

    // Test axes L = A0, A1, A2
    for (int i = 0; i < 3; ++i)
    {
        ra = a.e[i];
        rb = b.e[0] * AbsR[i][0] + b.e[1] * AbsR[i][1] + b.e[2] * AbsR[i][2];
        if (abs(t[i]) > ra + rb) return 0;
    }

    // Test axes L = B0, B1, B2
    for (int i = 0; i < 3; ++i)
    {
        ra = a.e[0] * AbsR[0][i] + a.e[1] * AbsR[1][i] + a.e[2] * AbsR[2][i];
        rb = b.e[i];
        if (abs(t.x * R[0][i] + t.y * R[1][i] + t.z * R[2][i]) > ra + rb) return 0;
    }

    // Test axis L = Ai x Bj
    ra = a.e[1] * AbsR[2][0] + a.e[2] * AbsR[1][0];
    rb = b.e[1] * AbsR[0][2] + b.e[2] * AbsR[0][1];
    if (abs(t.z * R[1][0] - t.y * R[2][0]) > ra + rb) return 0;

    ra = a.e[1] * AbsR[2][1] + a.e[2] * AbsR[1][1];
    rb = b.e[0] * AbsR[0][2] + b.e[2] * AbsR[0][0];
    if (abs(t.z * R[1][1] - t.y * R[2][1]) > ra + rb) return 0;

    ra = a.e[1] * AbsR[2][2] + a.e[2] * AbsR[1][2];
    rb = b.e[0] * AbsR[0][1] + b.e[1] * AbsR[0][0];
    if (abs(t.z * R[1][2] - t.y * R[2][2]) > ra + rb) return 0;

    ra = a.e[0] * AbsR[2][0] + a.e[2] * AbsR[0][0];
    rb = b.e[1] * AbsR[1][2] + b.e[2] * AbsR[1][1];
    if (abs(t.x * R[2][0] - t.z * R[0][0]) > ra + rb) return 0;

    ra = a.e[0] * AbsR[2][1] + a.e[2] * AbsR[0][1];
    rb = b.e[0] * AbsR[1][2] + b.e[2] * AbsR[1][0];
    if (abs(t.x * R[2][1] - t.z * R[0][1]) > ra + rb) return 0;

    ra = a.e[0] * AbsR[2][2] + a.e[2] * AbsR[0][2];
    rb = b.e[0] * AbsR[1][1] + b.e[1] * AbsR[1][0];
    if (abs(t.x * R[2][2] - t.z * R[0][2]) > ra + rb) return 0;

    ra = a.e[0] * AbsR[1][0] + a.e[1] * AbsR[0][0];
    rb = b.e[1] * AbsR[2][2] + b.e[2] * AbsR[2][1];
    if (abs(t.y * R[0][0] - t.x * R[1][0]) > ra + rb) return 0;

    ra = a.e[0] * AbsR[1][1] + a.e[1] * AbsR[0][1];
    rb = b.e[0] * AbsR[2][2] + b.e[2] * AbsR[2][0];
    if (abs(t.y * R[0][1] - t.x * R[1][1]) > ra + rb) return 0;

    ra = a.e[0] * AbsR[1][2] + a.e[1] * AbsR[0][2];
    rb = b.e[0] * AbsR[2][1] + b.e[1] * AbsR[2][0];
    if (abs(t.y * R[0][2] - t.x * R[1][2]) > ra + rb) return 0;

    // No separating axis found
    return 1;
}

// Section 4.4.4: Minimum area rectangle in the xy plane
// Compute the center point, 'c', and axis orientation, u0 and u1, of
// the minimum area rectangle in the xy plane containing the points pt[].
float MinAreaRect(float2 pt[], int numPts, out float2 c, out float2 u0, out float2 u1)
{
    float minArea = kMaxFloat;

    // Loop through all edges; j trails i by 1, modulo numPts
    for (int i = 0, j = numPts - 1; i < numPts; j = i, ++i)
    {
        // Get current edge e0 (e0x,e0y), normalized
        float2 e0 = pt[i] - pt[j];
        e0 = normalize(e0);

        // Get an axis e1 orthogonal to edge e0
        float2 e1 = float2(-e0.y, e0.x);

        // Loop through all points to get maximum extents
        float min0 = 0.0f, min1 = 0.0f, max0 = 0.0f, max1 = 0.0f;
        for (int k = 0; k < numPts; ++k)
        {
            // Project points onto axes e0 and e1 and keep track
            // of minimum and maximum values along both axes
            float2 d = pt[k] - pt[j];
            float dot0 = dot(d, e0);
            if (dot0 < min0) min0 = dot0;
            if (dot0 > max0) max0 = dot0;
            float dot1 = dot(d, e1);
            if (dot1 < min1) min1 = dot1;
            if (dot1 > max1) max1 = dot1;
        }
        float area = (max0 - min0) * (max1 - min1);

        // If best so far, remember area, center, and axes
        if (area < minArea)
        {
            minArea = area;
            c = pt[j] + 0.5f * ((min0 + max0) * e0 + (min1 + max1) * e1);
            u0 = e0;
            u1 = e1;
        }
    }
    return minArea;
}

// Forward declarations for Chapter 5 routines (to be implemented in ch5)
float SqDistPointSegment(float3 a, float3 b, float3 c);
float ClosestPtSegmentSegment(float3 p1, float3 q1, float3 p2, float3 q2,
                                out float s, out float t,
                                out float3 c1, out float3 c2);

// Section 4.5.1: Sphere-Capsule and Capsule-Capsule intersection tests
int TestSphereCapsule(Sphere s, Capsule capsule)
{
    // Compute (squared) distance between sphere center and capsule line segment
    float dist2 = SqDistPointSegment(capsule.a, capsule.b, s.c);

    // If (squared) distance smaller than (squared) sum of radii, they collide
    float radius = s.r + capsule.r;
    return dist2 <= radius * radius;
}

int TestCapsuleCapsule(Capsule capsule1, Capsule capsule2)
{
    // Compute (squared) distance between the inner structures of the capsules
    float s, t;
    float3 c1, c2;
    float dist2 = ClosestPtSegmentSegment(capsule1.a, capsule1.b,
                                            capsule2.a, capsule2.b,
                                            s, t, c1, c2);

    // If (squared) distance smaller than (squared) sum of radii, they collide
    float radius = capsule1.r + capsule2.r;
    return dist2 <= radius * radius;
}

// Section 4.6.3: Test overlap of two k-DOPs
int TestKDOPKDOP(KDOP a, KDOP b, int k)
{
    // Check if any intervals are non-overlapping, return if so
    for (int i = 0; i < k / 2; ++i)
        if (a.min[i] > b.max[i] || a.max[i] < b.min[i])
            return 0;
    // All intervals are overlapping, so k-DOPs intersect
    return 1;
}

// Section 4.6.4: Compute 8-DOP for object vertices v[] in world space
// using the axes (1,1,1), (1,1,-1), (1,-1,1) and (-1,1,1)
void ComputeDOP8(float3 v[], int numPts, out DOP8 dop8)
{
    // Initialize 8-DOP to an empty volume
    dop8.min[0] = dop8.min[1] = dop8.min[2] = dop8.min[3] = kMaxFloat;
    dop8.max[0] = dop8.max[1] = dop8.max[2] = dop8.max[3] = -kMaxFloat;

    // For each point, update 8-DOP bounds if necessary
    for (int i = 0; i < numPts; ++i)
    {
        float3 p = v[i];
        float value;
        value = dot(p, float3(1, 1, 1));
        dop8.min[0] = min(dop8.min[0], value);
        dop8.max[0] = max(dop8.max[0], value);
        value = dot(p, float3(1, 1, -1));
        dop8.min[1] = min(dop8.min[1], value);
        dop8.max[1] = max(dop8.max[1], value);
        value = dot(p, float3(1, -1, 1));
        dop8.min[2] = min(dop8.min[2], value);
        dop8.max[2] = max(dop8.max[2], value);
        value = dot(p, float3(-1, 1, 1));
        dop8.min[3] = min(dop8.min[3], value);
        dop8.max[3] = max(dop8.max[3], value);
    }
}

} // namespace ch4
