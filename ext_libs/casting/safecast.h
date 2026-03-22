/*
MIT License

Copyright (c) 2025 nguyenchiemminhvu@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef POD_CAST_H
#define POD_CAST_H

#include <cstdint>
#include <cfloat>
#include <limits>
#include <cmath>

class POD_CAST
{
public:
    // bool conversions
    static int8_t bool_to_int8(bool value);
    static int16_t bool_to_int16(bool value);
    static int32_t bool_to_int32(bool value);
    static int64_t bool_to_int64(bool value);
    static uint8_t bool_to_uint8(bool value);
    static uint16_t bool_to_uint16(bool value);
    static uint32_t bool_to_uint32(bool value);
    static uint64_t bool_to_uint64(bool value);
    static float bool_to_float(bool value);
    static double bool_to_double(bool value);

    // int8_t conversions
    static bool int8_to_bool(int8_t value);
    static int16_t int8_to_int16(int8_t value);
    static int32_t int8_to_int32(int8_t value);
    static int64_t int8_to_int64(int8_t value);
    static uint8_t int8_to_uint8(int8_t value);
    static uint16_t int8_to_uint16(int8_t value);
    static uint32_t int8_to_uint32(int8_t value);
    static uint64_t int8_to_uint64(int8_t value);
    static float int8_to_float(int8_t value);
    static double int8_to_double(int8_t value);

    // int16_t conversions
    static bool int16_to_bool(int16_t value);
    static int8_t int16_to_int8(int16_t value);
    static int32_t int16_to_int32(int16_t value);
    static int64_t int16_to_int64(int16_t value);
    static uint8_t int16_to_uint8(int16_t value);
    static uint16_t int16_to_uint16(int16_t value);
    static uint32_t int16_to_uint32(int16_t value);
    static uint64_t int16_to_uint64(int16_t value);
    static float int16_to_float(int16_t value);
    static double int16_to_double(int16_t value);

    // int32_t conversions
    static bool int32_to_bool(int32_t value);
    static int8_t int32_to_int8(int32_t value);
    static int16_t int32_to_int16(int32_t value);
    static int64_t int32_to_int64(int32_t value);
    static uint8_t int32_to_uint8(int32_t value);
    static uint16_t int32_to_uint16(int32_t value);
    static uint32_t int32_to_uint32(int32_t value);
    static uint64_t int32_to_uint64(int32_t value);
    static float int32_to_float(int32_t value);
    static double int32_to_double(int32_t value);

    // int64_t conversions
    static bool int64_to_bool(int64_t value);
    static int8_t int64_to_int8(int64_t value);
    static int16_t int64_to_int16(int64_t value);
    static int32_t int64_to_int32(int64_t value);
    static uint8_t int64_to_uint8(int64_t value);
    static uint16_t int64_to_uint16(int64_t value);
    static uint32_t int64_to_uint32(int64_t value);
    static uint64_t int64_to_uint64(int64_t value);
    static float int64_to_float(int64_t value);
    static double int64_to_double(int64_t value);

    // uint8_t conversions
    static bool uint8_to_bool(uint8_t value);
    static int8_t uint8_to_int8(uint8_t value);
    static int16_t uint8_to_int16(uint8_t value);
    static int32_t uint8_to_int32(uint8_t value);
    static int64_t uint8_to_int64(uint8_t value);
    static uint16_t uint8_to_uint16(uint8_t value);
    static uint32_t uint8_to_uint32(uint8_t value);
    static uint64_t uint8_to_uint64(uint8_t value);
    static float uint8_to_float(uint8_t value);
    static double uint8_to_double(uint8_t value);

    // uint16_t conversions
    static bool uint16_to_bool(uint16_t value);
    static int8_t uint16_to_int8(uint16_t value);
    static int16_t uint16_to_int16(uint16_t value);
    static int32_t uint16_to_int32(uint16_t value);
    static int64_t uint16_to_int64(uint16_t value);
    static uint8_t uint16_to_uint8(uint16_t value);
    static uint32_t uint16_to_uint32(uint16_t value);
    static uint64_t uint16_to_uint64(uint16_t value);
    static float uint16_to_float(uint16_t value);
    static double uint16_to_double(uint16_t value);

    // uint32_t conversions
    static bool uint32_to_bool(uint32_t value);
    static int8_t uint32_to_int8(uint32_t value);
    static int16_t uint32_to_int16(uint32_t value);
    static int32_t uint32_to_int32(uint32_t value);
    static int64_t uint32_to_int64(uint32_t value);
    static uint8_t uint32_to_uint8(uint32_t value);
    static uint16_t uint32_to_uint16(uint32_t value);
    static uint64_t uint32_to_uint64(uint32_t value);
    static float uint32_to_float(uint32_t value);
    static double uint32_to_double(uint32_t value);

    // uint64_t conversions
    static bool uint64_to_bool(uint64_t value);
    static int8_t uint64_to_int8(uint64_t value);
    static int16_t uint64_to_int16(uint64_t value);
    static int32_t uint64_to_int32(uint64_t value);
    static int64_t uint64_to_int64(uint64_t value);
    static uint8_t uint64_to_uint8(uint64_t value);
    static uint16_t uint64_to_uint16(uint64_t value);
    static uint32_t uint64_to_uint32(uint64_t value);
    static float uint64_to_float(uint64_t value);
    static double uint64_to_double(uint64_t value);

    // float conversions
    static bool float_to_bool(float value);
    static int8_t float_to_int8(float value);
    static int16_t float_to_int16(float value);
    static int32_t float_to_int32(float value);
    static int64_t float_to_int64(float value);
    static uint8_t float_to_uint8(float value);
    static uint16_t float_to_uint16(float value);
    static uint32_t float_to_uint32(float value);
    static uint64_t float_to_uint64(float value);
    static double float_to_double(float value);

    // double conversions
    static bool double_to_bool(double value);
    static int8_t double_to_int8(double value);
    static int16_t double_to_int16(double value);
    static int32_t double_to_int32(double value);
    static int64_t double_to_int64(double value);
    static uint8_t double_to_uint8(double value);
    static uint16_t double_to_uint16(double value);
    static uint32_t double_to_uint32(double value);
    static uint64_t double_to_uint64(double value);
    static float double_to_float(double value);
};

template <typename From, typename To>
To POD_CAST(From value)
{
    return static_cast<To>(value);
}

template <> int8_t POD_CAST<bool, int8_t>(bool value);
template <> int16_t POD_CAST<bool, int16_t>(bool value);
template <> int32_t POD_CAST<bool, int32_t>(bool value);
template <> int64_t POD_CAST<bool, int64_t>(bool value);
template <> uint8_t POD_CAST<bool, uint8_t>(bool value);
template <> uint16_t POD_CAST<bool, uint16_t>(bool value);
template <> uint32_t POD_CAST<bool, uint32_t>(bool value);
template <> uint64_t POD_CAST<bool, uint64_t>(bool value);
template <> float POD_CAST<bool, float>(bool value);
template <> double POD_CAST<bool, double>(bool value);
template <> bool POD_CAST<int8_t, bool>(int8_t value);
template <> int16_t POD_CAST<int8_t, int16_t>(int8_t value);
template <> int32_t POD_CAST<int8_t, int32_t>(int8_t value);
template <> int64_t POD_CAST<int8_t, int64_t>(int8_t value);
template <> uint8_t POD_CAST<int8_t, uint8_t>(int8_t value);
template <> uint16_t POD_CAST<int8_t, uint16_t>(int8_t value);
template <> uint32_t POD_CAST<int8_t, uint32_t>(int8_t value);
template <> uint64_t POD_CAST<int8_t, uint64_t>(int8_t value);
template <> float POD_CAST<int8_t, float>(int8_t value);
template <> double POD_CAST<int8_t, double>(int8_t value);
template <> bool POD_CAST<int16_t, bool>(int16_t value);
template <> int8_t POD_CAST<int16_t, int8_t>(int16_t value);
template <> int32_t POD_CAST<int16_t, int32_t>(int16_t value);
template <> int64_t POD_CAST<int16_t, int64_t>(int16_t value);
template <> uint8_t POD_CAST<int16_t, uint8_t>(int16_t value);
template <> uint16_t POD_CAST<int16_t, uint16_t>(int16_t value);
template <> uint32_t POD_CAST<int16_t, uint32_t>(int16_t value);
template <> uint64_t POD_CAST<int16_t, uint64_t>(int16_t value);
template <> float POD_CAST<int16_t, float>(int16_t value);
template <> double POD_CAST<int16_t, double>(int16_t value);
template <> bool POD_CAST<int32_t, bool>(int32_t value);
template <> int8_t POD_CAST<int32_t, int8_t>(int32_t value);
template <> int16_t POD_CAST<int32_t, int16_t>(int32_t value);
template <> int64_t POD_CAST<int32_t, int64_t>(int32_t value);
template <> uint8_t POD_CAST<int32_t, uint8_t>(int32_t value);
template <> uint16_t POD_CAST<int32_t, uint16_t>(int32_t value);
template <> uint32_t POD_CAST<int32_t, uint32_t>(int32_t value);
template <> uint64_t POD_CAST<int32_t, uint64_t>(int32_t value);
template <> float POD_CAST<int32_t, float>(int32_t value);
template <> double POD_CAST<int32_t, double>(int32_t value);
template <> bool POD_CAST<int64_t, bool>(int64_t value);
template <> int8_t POD_CAST<int64_t, int8_t>(int64_t value);
template <> int16_t POD_CAST<int64_t, int16_t>(int64_t value);
template <> int32_t POD_CAST<int64_t, int32_t>(int64_t value);
template <> uint8_t POD_CAST<int64_t, uint8_t>(int64_t value);
template <> uint16_t POD_CAST<int64_t, uint16_t>(int64_t value);
template <> uint32_t POD_CAST<int64_t, uint32_t>(int64_t value);
template <> uint64_t POD_CAST<int64_t, uint64_t>(int64_t value);
template <> float POD_CAST<int64_t, float>(int64_t value);
template <> double POD_CAST<int64_t, double>(int64_t value);
template <> bool POD_CAST<uint8_t, bool>(uint8_t value);
template <> int8_t POD_CAST<uint8_t, int8_t>(uint8_t value);
template <> int16_t POD_CAST<uint8_t, int16_t>(uint8_t value);
template <> int32_t POD_CAST<uint8_t, int32_t>(uint8_t value);
template <> int64_t POD_CAST<uint8_t, int64_t>(uint8_t value);
template <> uint16_t POD_CAST<uint8_t, uint16_t>(uint8_t value);
template <> uint32_t POD_CAST<uint8_t, uint32_t>(uint8_t value);
template <> uint64_t POD_CAST<uint8_t, uint64_t>(uint8_t value);
template <> float POD_CAST<uint8_t, float>(uint8_t value);
template <> double POD_CAST<uint8_t, double>(uint8_t value);
template <> bool POD_CAST<uint16_t, bool>(uint16_t value);
template <> int8_t POD_CAST<uint16_t, int8_t>(uint16_t value);
template <> int32_t POD_CAST<uint16_t, int32_t>(uint16_t value);
template <> int64_t POD_CAST<uint16_t, int64_t>(uint16_t value);
template <> uint8_t POD_CAST<uint16_t, uint8_t>(uint16_t value);
template <> uint16_t POD_CAST<uint16_t, uint16_t>(uint16_t value);
template <> uint32_t POD_CAST<uint16_t, uint32_t>(uint16_t value);
template <> uint64_t POD_CAST<uint16_t, uint64_t>(uint16_t value);
template <> float POD_CAST<uint16_t, float>(uint16_t value);
template <> double POD_CAST<uint16_t, double>(uint16_t value);
template <> bool POD_CAST<uint32_t, bool>(uint32_t value);
template <> int8_t POD_CAST<uint32_t, int8_t>(uint32_t value);
template <> int16_t POD_CAST<uint32_t, int16_t>(uint32_t value);
template <> int32_t POD_CAST<uint32_t, int32_t>(uint32_t value);
template <> int64_t POD_CAST<uint32_t, int64_t>(uint32_t value);
template <> uint8_t POD_CAST<uint32_t, uint8_t>(uint32_t value);
template <> uint16_t POD_CAST<uint32_t, uint16_t>(uint32_t value);
template <> uint64_t POD_CAST<uint32_t, uint64_t>(uint32_t value);
template <> float POD_CAST<uint32_t, float>(uint32_t value);
template <> double POD_CAST<uint32_t, double>(uint32_t value);
template <> bool POD_CAST<uint64_t, bool>(uint64_t value);
template <> int8_t POD_CAST<uint64_t, int8_t>(uint64_t value);
template <> int16_t POD_CAST<uint64_t, int16_t>(uint64_t value);
template <> int32_t POD_CAST<uint64_t, int32_t>(uint64_t value);
template <> int64_t POD_CAST<uint64_t, int64_t>(uint64_t value);
template <> uint8_t POD_CAST<uint64_t, uint8_t>(uint64_t value);
template <> uint16_t POD_CAST<uint64_t, uint16_t>(uint64_t value);
template <> uint32_t POD_CAST<uint64_t, uint32_t>(uint64_t value);
template <> float POD_CAST<uint64_t, float>(uint64_t value);
template <> double POD_CAST<uint64_t, double>(uint64_t value);
template <> bool POD_CAST<float, bool>(float value);
template <> int8_t POD_CAST<float, int8_t>(float value);
template <> int16_t POD_CAST<float, int16_t>(float value);
template <> int32_t POD_CAST<float, int32_t>(float value);
template <> int64_t POD_CAST<float, int64_t>(float value);
template <> uint8_t POD_CAST<float, uint8_t>(float value);
template <> uint16_t POD_CAST<float, uint16_t>(float value);
template <> uint32_t POD_CAST<float, uint32_t>(float value);
template <> uint64_t POD_CAST<float, uint64_t>(float value);
template <> double POD_CAST<float, double>(float value);
template <> bool POD_CAST<double, bool>(double value);
template <> int8_t POD_CAST<double, int8_t>(double value);
template <> int16_t POD_CAST<double, int16_t>(double value);
template <> int32_t POD_CAST<double, int32_t>(double value);
template <> int64_t POD_CAST<double, int64_t>(double value);
template <> uint8_t POD_CAST<double, uint8_t>(double value);
template <> uint16_t POD_CAST<double, uint16_t>(double value);
template <> uint32_t POD_CAST<double, uint32_t>(double value);
template <> uint64_t POD_CAST<double, uint64_t>(double value);
template <> float POD_CAST<double, float>(double value);

#endif // POD_CAST_H