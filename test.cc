#include <math.h>

#include "cdt.hh"

using namespace ImplicitTriangulation;
using namespace autodiff;

int main()
{
    auto mesh = CurvatureDependentTriangulation::Tessellate(
        [](const dual &x, const dual &y, const dual &z) -> dual
        { return x * x + y * y + z * z; },
        {Vector3D(-10, -10, -10), Vector3D(10, 10, 10)},
        2,
        1,
        Vector3D(0, 0, 0),
        Vector3D(0, 0, 3));

    /*
    auto mesh = CurvatureDependentTriangulation::Tessellate(
        [](const dual &x, const dual &y, const dual &z) -> dual
        //{ return x * x + y * y + z * z; },
        { return x * x - y * y + z * z; },
        {Vector3D(-2, -2, -2), Vector3D(2, 2, 2)},
        2,
        1,
        Vector3D(0, 0, 0),
        Vector3D(0, 0, 2)); */

    mesh.writeOBJ("testmesh.obj");

    /* 
    Vector3D point = CurvatureDependentTriangulation::ProjectPointToSurface([](const dual &x, const dual &y, const dual &z) -> dual
                                                                            { return x * x + y * y + z * z; },
                                                                            1, Vector3D(3, 5, 20));

    std::cout << point.norm() << std::endl;
    */
}