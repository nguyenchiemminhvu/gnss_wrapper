#ifndef GEOMETRIC_H
#define GEOMETRIC_H

#include <vector>
#include <climits>
#include <cstdint>
#include <cfloat>
#include <cmath>

#define GEOMETRIC_MAX_INPUT_POINTS (1024U)

#ifndef EPSILON
#define EPSILON (1e-9)
#endif

struct Point
{
    double _x;
    double _y;

    Point(double x = 0.0, double y = 0.0)
        : _x(x), _y(y)
    {

    }

    Point(const Point &other)
    {
        _x = other._x;
        _y = other._y;
    }

    static int Orientation(const Point& a, const Point& b, const Point& c)
    {
        double val = (b._y - a._y) * (c._x - b._x) - (c._y - b._y) * (b._x - a._x);

        if (std::abs(val) <= EPSILON)
        {
            return 0;
        }

        return (val > 0.0) ? 1 : -1;
    }
};

struct Line
{
    double _a;
    double _b;
    double _c;

    Line(double a = 0.0, double b = 0.0, double c = 0.0)
        : _a(a), _b(b), _c(c)
    {

    }

    Line(const Point& A, const Point& B)
    {
        this->_a = B._y - A._y;
        this->_b = A._x - B._x;
        this->_c = this->_a * A._x + this->_b * A._y;
    }

    bool FindIntersectingPoint(const Line& L, Point &I) const
    {
        double determinant = _a * L._a - _b * L._b;
        if (std::abs(determinant) <= EPSILON) // parallel case
        {
            return false;
        }

        I._x = (_c * L._b - L._c * _b) / (_a * L._b - L._a * _b);
        I._y = (_c * L._a - _a * L._c) / (_b * L._a - _a * L._b);
        return true;
    }

    static bool IsIntersecting(const Point& p1, const Point& p2, const Point& q1, const Point& q2) 
    {
        int o1, o2, o3, o4;
        o1 = Point::Orientation(p1, p2, q1);
        o2 = Point::Orientation(p1, p2, q2);
        o3 = Point::Orientation(q1, q2, p1);
        o4 = Point::Orientation(q1, q2, p2);

        if (o1 != o2 && o3 != o4)
        {
            return true;
        }

        return (o1 * o2 < 0) && (o3 * o4 < 0);
    }

    static bool OnSegment(const Point& A, const Point& B, const Point& O)
    {
        if (O._x <= std::max(A._x, B._x) && O._x >= std::min(A._x, B._x)
        &&  O._y <= std::max(A._y, B._y) && O._y >= std::min(A._y, B._y))
        {
            return true;
        }

        return false;
    }
};

struct Polygon
{
    std::vector<Point> _pol;

    Polygon(const std::vector<Point>& pol)
        : _pol(pol)
    {

    }

    bool isPointInside_JordanCurveTheorem(const Point& P) const
    {
        if (_pol.size() < 3U)
        {
            return false;
        }

        bool c = false;
        size_t nvert = _pol.size();
        size_t i = 0U;
        size_t j = nvert - 1U;
        while (i < nvert && i < GEOMETRIC_MAX_INPUT_POINTS && j < nvert && j < GEOMETRIC_MAX_INPUT_POINTS)
        {
            if (((_pol[i]._y > P._y) != (_pol[j]._y > P._y)) 
             && (P._x < (_pol[j]._x - _pol[i]._x) * (P._y - _pol[i]._y) / (_pol[j]._y - _pol[i]._y) + _pol[i]._x))
            {
                c = !c;
            }

            j = i;
            i++;
        }
        return c;
    }
};

#endif // GEOMETRIC_H