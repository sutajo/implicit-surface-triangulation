#pragma once

#include "ImplicitPrimitive.hpp"
#include <autodiff/forward/dual.hpp>
#include <functional>

namespace Implicit
{
	template<class Derived>
	class Differentiated : public Object
	{
	public:
		using EquationT = std::function<autodiff::dual(const autodiff::dual& x, const autodiff::dual& y, const autodiff::dual& z)>;
	protected:
		Differentiated() : Object() { }

		Differentiated(double iso) : Object(iso) { }

		virtual void SetEquation()=0;

		virtual double FieldValue(const glm::dvec3& point) override
		{
			autodiff::dual x = point.x;
			autodiff::dual y = point.y;
			autodiff::dual z = point.z;

			return autodiff::val(static_cast<Derived&>(*this).Equation(x, y, z));
		}

		virtual glm::dvec3 Normal(const glm::dvec3& point) override
		{
			autodiff::dual x = point.x;
			autodiff::dual y = point.y;
			autodiff::dual z = point.z;
			auto u = static_cast<Derived&>(*this).Equation(x, y, z);
			const double ux = derivative(static_cast<Derived&>(*this).Equation, autodiff::wrt(x), autodiff::at(x, y, z));
			const double uy = derivative(static_cast<Derived&>(*this).Equation, autodiff::wrt(y), autodiff::at(x, y, z));
			const double uz = derivative(static_cast<Derived&>(*this).Equation, autodiff::wrt(z), autodiff::at(x, y, z));
			const glm::vec3 derivative = glm::dvec3(ux, uy, uz);
			if (glm::length(derivative) == 0.0) throw std::runtime_error("Failed to compute normal");
			return glm::normalize(derivative);
		}
	};
};

