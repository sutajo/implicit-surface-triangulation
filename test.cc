#include <math.h>
#include <filesystem>
#include <spdlog/spdlog.h>
#include "cdt.hh"

using namespace ImplicitTriangulation;
using namespace autodiff;

int main()
{
    auto mesh = CurvatureDependentTriangulation::Tessellate(
        [](const dual &x, const dual &y, const dual &z) -> dual
        { return x * x + y * y + z * z; },
        {Vector3D(-10, -10, -10), Vector3D(10, 10, 10)},
        1,
        1,
        Vector3D(0, 0, 0),
        Vector3D(0, 0, 2));

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

    const std::string meshOutPath = std::filesystem::current_path() / "testmesh.obj";
    spdlog::info("Writing mesh to {}", meshOutPath);
    mesh.writeOBJ(meshOutPath);

    /*
    Vector3D point = CurvatureDependentTriangulation::ProjectPointToSurface([](const dual &x, const dual &y, const dual &z) -> dual
                                                                            { return x * x + y * y + z * z; },
                                                                            1, Vector3D(3, 5, 20));

    std::cout << point.norm() << std::endl;
    */
}