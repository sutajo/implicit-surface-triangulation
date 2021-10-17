#include <math.h>

#include "cdt.hh"

using namespace Geometry;
using namespace autodiff;

int main()
{
    auto mesh = CurvatureDependentTriangulation::Tessellate(
        [](const dual &x, const dual &y, const dual &z) -> dual
        { return x * x + y * y + z * z; },
        {Vector3D(-1, -1, -1), Vector3D(1, 1, 1)},
        1,
        1,
        Vector3D(0, 0, 0),
        Vector3D(0, 0, 2));

    mesh.writeOBJ("/home/tamas/Documents/diplomamunka/testmesh.obj");

    /* 
    Vector3D point = CurvatureDependentTriangulation::ProjectPointToSurface([](const dual &x, const dual &y, const dual &z) -> dual
                                                                            { return x * x + y * y + z * z; },
                                                                            1, Vector3D(3, 5, 20));

    std::cout << point.norm() << std::endl;
    */
}