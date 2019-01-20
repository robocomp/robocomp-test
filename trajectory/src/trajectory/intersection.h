#include <iostream>

struct Pointinter
{
	Pointinter()
	{
		x = 0;
		y = 0;
	}
	Pointinter(float x_, float y_)
	{
		x = x_;
		y = y_;
	}
	float x;
	float y;
};

struct LineSegment
{
	LineSegment()
	{
		first = Pointinter();
		second = Pointinter();
	}
	LineSegment(Pointinter f, Pointinter s)
	{
		first = f;
		second = s;
	}
	Pointinter first;
	Pointinter second;
};

LineSegment getIntersection(LineSegment a, LineSegment b);


