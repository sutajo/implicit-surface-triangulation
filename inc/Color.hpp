/*
 * Color
 *
 * File: 	Color.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#ifndef COLOR_HPP
#define COLOR_HPP

#include <glm/glm.hpp>

/**
 * \brief Color class
 * Stores color in floating-point form from 0 to 1
 * To translate from 0 - 255, divide your value by 255
 */
class Color3
{
	public:
		Color3();
		explicit Color3(float a);
		Color3(float _a, float _b, float _c);
		Color3(int _a, int _b, int _c);

		// Accessors
		union
		{
			struct {float r; float g; float b;};
			struct {float h; float s; float l;};
		};

		float &		operator [] (int i);
		const float &	operator [] (int i) const;

		// Copy constructors
		Color3(const Color3& c);
		const Color3 & 	operator = (const Color3& c);

		// Component-wise addition
		const Color3& 	operator += (const Color3& c);
		Color3 		operator + (const Color3& c);

		// Component-wise subtraction
		const Color3& 	operator -= (const Color3& c);
		Color3		operator - (const Color3& c);

		// Negation
		Color3		operator -() const;
		const Color3&	negate();

		// Component-wise multiplication
		const Color3&	operator *= (const Color3& c);
		const Color3& 	operator *= (float a);
		Color3		operator * (const Color3& c) const;
		Color3		operator * (float a) const;

		// Component-wise division
		const Color3&	operator /= (const Color3& c);
		const Color3&	operator /= (float a);
		Color3		operator / (const Color3& c) const;
		Color3		operator / (float a) const;

		// Equality
		bool		operator == (const Color3& c) const;
		bool		operator != (const Color3& c) const;

};

class Color4
{
	public:
		Color4();
		explicit Color4(float a);
		Color4(float _a, float _b, float _c, float _d);
		Color4(int _a, int _b, int _c, int _d);

		// Accessors
		union
		{
			struct {float r; float g; float b; float a;};
			struct {float h; float s; float l; float a;};
		};
		float & 	operator [] (int i);
		const float &	operator [] (int i) const;

		// Copy constructor
		Color4(const Color4& c);
		Color4(const Color3& c);
		const Color4& operator = (const Color4& c);
		const Color4& operator = (const Color3& c);

		// Component-wise addition
		const Color4& 	operator += (const Color4& c);
		Color4 		operator + (const Color4& c);

		// Component-wise subtraction
		const Color4& 	operator -= (const Color4& c);
		Color4		operator - (const Color4& c);

		// Negation
		Color4		operator -() const;
		const Color4&	negate();

		// Component-wise multiplication
		const Color4&	operator *= (const Color4& c);
		const Color4& 	operator *= (float a);
		Color4		operator * (const Color4& c) const;
		Color4		operator * (float a) const;

		// Component-wise division
		const Color4&	operator /= (const Color4& c);
		const Color4&	operator /= (float a);
		Color4		operator / (const Color4& c) const;
		Color4		operator / (float a) const;

		// Equality
		bool		operator == (const Color4& c) const;
		bool		operator != (const Color4& c) const;
};
#endif//COLOR_HPP
