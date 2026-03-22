#include "safecast.h"

#ifndef EPSILON
#define EPSILON (1e-9)
#endif

// bool conversions
int8_t POD_CAST::bool_to_int8(bool value)
{
    return value ? static_cast<int8_t>(1) : static_cast<int8_t>(0);
}

int16_t POD_CAST::bool_to_int16(bool value)
{
    return value ? static_cast<int16_t>(1) : static_cast<int16_t>(0);
}

int32_t POD_CAST::bool_to_int32(bool value)
{
    return value ? static_cast<int32_t>(1) : static_cast<int32_t>(0);
}

int64_t POD_CAST::bool_to_int64(bool value)
{
    return value ? static_cast<int64_t>(1) : static_cast<int64_t>(0);
}

uint8_t POD_CAST::bool_to_uint8(bool value)
{
    return value ? static_cast<uint8_t>(1U) : static_cast<uint8_t>(0U);
}

uint16_t POD_CAST::bool_to_uint16(bool value)
{
    return value ? static_cast<uint16_t>(1U) : static_cast<uint16_t>(0U);
}

uint32_t POD_CAST::bool_to_uint32(bool value)
{
    return value ? static_cast<uint32_t>(1U) : static_cast<uint32_t>(0U);
}

uint64_t POD_CAST::bool_to_uint64(bool value)
{
    return value ? static_cast<uint64_t>(1U) : static_cast<uint64_t>(0U);
}

float POD_CAST::bool_to_float(bool value)
{
    return value ? 1.0f : 0.0f;
}

double POD_CAST::bool_to_double(bool value)
{
    return value ? 1.0 : 0.0;
}

// int8_t conversions
bool POD_CAST::int8_to_bool(int8_t value)
{
    return value != 0;
}

int16_t POD_CAST::int8_to_int16(int8_t value)
{
    return static_cast<int16_t>(value);
}

int32_t POD_CAST::int8_to_int32(int8_t value)
{
    return static_cast<int32_t>(value);
}

int64_t POD_CAST::int8_to_int64(int8_t value)
{
    return static_cast<int64_t>(value);
}

uint8_t POD_CAST::int8_to_uint8(int8_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint8_t>(value);
}

uint16_t POD_CAST::int8_to_uint16(int8_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint16_t>(value);
}

uint32_t POD_CAST::int8_to_uint32(int8_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint32_t>(value);
}

uint64_t POD_CAST::int8_to_uint64(int8_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint64_t>(value);
}

float POD_CAST::int8_to_float(int8_t value)
{
    return static_cast<float>(value);
}

double POD_CAST::int8_to_double(int8_t value)
{
    return static_cast<double>(value);
}

// int16_t conversions
bool POD_CAST::int16_to_bool(int16_t value)
{
    return value != 0;
}

int8_t POD_CAST::int16_to_int8(int16_t value)
{
    if (value < static_cast<int16_t>(INT8_MIN))
    {
        return static_cast<int8_t>(INT8_MIN);
    }
    if (value > static_cast<int16_t>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int32_t POD_CAST::int16_to_int32(int16_t value)
{
    return static_cast<int32_t>(value);
}

int64_t POD_CAST::int16_to_int64(int16_t value)
{
    return static_cast<int64_t>(value);
}

uint8_t POD_CAST::int16_to_uint8(int16_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    if (value > static_cast<int16_t>(UINT8_MAX))
    {
        return static_cast<uint8_t>(UINT8_MAX);
    }
    return static_cast<uint8_t>(value);
}

uint16_t POD_CAST::int16_to_uint16(int16_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint16_t>(value);
}

uint32_t POD_CAST::int16_to_uint32(int16_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint32_t>(value);
}

uint64_t POD_CAST::int16_to_uint64(int16_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint64_t>(value);
}

float POD_CAST::int16_to_float(int16_t value)
{
    return static_cast<float>(value);
}

double POD_CAST::int16_to_double(int16_t value)
{
    return static_cast<double>(value);
}

// int32_t conversions
bool POD_CAST::int32_to_bool(int32_t value)
{
    return value != 0;
}

int8_t POD_CAST::int32_to_int8(int32_t value)
{
    if (value < static_cast<int32_t>(INT8_MIN))
    {
        return static_cast<int8_t>(INT8_MIN);
    }
    if (value > static_cast<int32_t>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int16_t POD_CAST::int32_to_int16(int32_t value)
{
    if (value < static_cast<int32_t>(INT16_MIN))
    {
        return static_cast<int16_t>(INT16_MIN);
    }
    if (value > static_cast<int32_t>(INT16_MAX))
    {
        return static_cast<int16_t>(INT16_MAX);
    }
    return static_cast<int16_t>(value);
}

int64_t POD_CAST::int32_to_int64(int32_t value)
{
    return static_cast<int64_t>(value);
}

uint8_t POD_CAST::int32_to_uint8(int32_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    if (value > static_cast<int32_t>(UINT8_MAX))
    {
        return static_cast<uint8_t>(UINT8_MAX);
    }
    return static_cast<uint8_t>(value);
}

uint16_t POD_CAST::int32_to_uint16(int32_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    if (value > static_cast<int32_t>(UINT16_MAX))
    {
        return static_cast<uint16_t>(UINT16_MAX);
    }
    return static_cast<uint16_t>(value);
}

uint32_t POD_CAST::int32_to_uint32(int32_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint32_t>(value);
}

uint64_t POD_CAST::int32_to_uint64(int32_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint64_t>(value);
}

float POD_CAST::int32_to_float(int32_t value)
{
    return static_cast<float>(value);
}

double POD_CAST::int32_to_double(int32_t value)
{
    return static_cast<double>(value);
}

// int64_t conversions
bool POD_CAST::int64_to_bool(int64_t value)
{
    return value != 0;
}

int8_t POD_CAST::int64_to_int8(int64_t value)
{
    if (value < static_cast<int64_t>(INT8_MIN))
    {
        return static_cast<int8_t>(INT8_MIN);
    }
    if (value > static_cast<int64_t>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int16_t POD_CAST::int64_to_int16(int64_t value)
{
    if (value < static_cast<int64_t>(INT16_MIN))
    {
        return static_cast<int16_t>(INT16_MIN);
    }
    if (value > static_cast<int64_t>(INT16_MAX))
    {
        return static_cast<int16_t>(INT16_MAX);
    }
    return static_cast<int16_t>(value);
}

int32_t POD_CAST::int64_to_int32(int64_t value)
{
    if (value < static_cast<int64_t>(INT32_MIN))
    {
        return static_cast<int32_t>(INT32_MIN);
    }
    if (value > static_cast<int64_t>(INT32_MAX))
    {
        return static_cast<int32_t>(INT32_MAX);
    }
    return static_cast<int32_t>(value);
}

uint8_t POD_CAST::int64_to_uint8(int64_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    if (value > static_cast<int64_t>(UINT8_MAX))
    {
        return static_cast<uint8_t>(UINT8_MAX);
    }
    return static_cast<uint8_t>(value);
}

uint16_t POD_CAST::int64_to_uint16(int64_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    if (value > static_cast<int64_t>(UINT16_MAX))
    {
        return static_cast<uint16_t>(UINT16_MAX);
    }
    return static_cast<uint16_t>(value);
}

uint32_t POD_CAST::int64_to_uint32(int64_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    if (value > static_cast<int64_t>(UINT32_MAX))
    {
        return static_cast<uint32_t>(UINT32_MAX);
    }
    return static_cast<uint32_t>(value);
}

uint64_t POD_CAST::int64_to_uint64(int64_t value)
{
    if (value < 0)
    {
        return 0U;
    }
    return static_cast<uint64_t>(value);
}

float POD_CAST::int64_to_float(int64_t value)
{
    if (static_cast<double>(value) < static_cast<double>(-FLT_MAX))
    {
        return -FLT_MAX;
    }
    if (static_cast<double>(value) > static_cast<double>(FLT_MAX))
    {
        return FLT_MAX;
    }
    return static_cast<float>(value);
}

double POD_CAST::int64_to_double(int64_t value)
{
    return static_cast<double>(value);
}

// uint8_t conversions
bool POD_CAST::uint8_to_bool(uint8_t value)
{
    return value != 0U;
}

int8_t POD_CAST::uint8_to_int8(uint8_t value)
{
    if (value > static_cast<uint8_t>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int16_t POD_CAST::uint8_to_int16(uint8_t value)
{
    return static_cast<int16_t>(value);
}

int32_t POD_CAST::uint8_to_int32(uint8_t value)
{
    return static_cast<int32_t>(value);
}

int64_t POD_CAST::uint8_to_int64(uint8_t value)
{
    return static_cast<int64_t>(value);
}

uint16_t POD_CAST::uint8_to_uint16(uint8_t value)
{
    return static_cast<uint16_t>(value);
}

uint32_t POD_CAST::uint8_to_uint32(uint8_t value)
{
    return static_cast<uint32_t>(value);
}

uint64_t POD_CAST::uint8_to_uint64(uint8_t value)
{
    return static_cast<uint64_t>(value);
}

float POD_CAST::uint8_to_float(uint8_t value)
{
    return static_cast<float>(value);
}

double POD_CAST::uint8_to_double(uint8_t value)
{
    return static_cast<double>(value);
}

// uint16_t conversions
bool POD_CAST::uint16_to_bool(uint16_t value)
{
    return value != 0U;
}

int8_t POD_CAST::uint16_to_int8(uint16_t value)
{
    if (value > static_cast<uint16_t>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int16_t POD_CAST::uint16_to_int16(uint16_t value)
{
    if (value > static_cast<uint16_t>(INT16_MAX))
    {
        return static_cast<int16_t>(INT16_MAX);
    }
    return static_cast<int16_t>(value);
}

int32_t POD_CAST::uint16_to_int32(uint16_t value)
{
    return static_cast<int32_t>(value);
}

int64_t POD_CAST::uint16_to_int64(uint16_t value)
{
    return static_cast<int64_t>(value);
}

uint8_t POD_CAST::uint16_to_uint8(uint16_t value)
{
    if (value > static_cast<uint16_t>(UINT8_MAX))
    {
        return static_cast<uint8_t>(UINT8_MAX);
    }
    return static_cast<uint8_t>(value);
}

uint32_t POD_CAST::uint16_to_uint32(uint16_t value)
{
    return static_cast<uint32_t>(value);
}

uint64_t POD_CAST::uint16_to_uint64(uint16_t value)
{
    return static_cast<uint64_t>(value);
}

float POD_CAST::uint16_to_float(uint16_t value)
{
    return static_cast<float>(value);
}

double POD_CAST::uint16_to_double(uint16_t value)
{
    return static_cast<double>(value);
}

// uint32_t conversions
bool POD_CAST::uint32_to_bool(uint32_t value)
{
    return value != 0U;
}

int8_t POD_CAST::uint32_to_int8(uint32_t value)
{
    if (value > static_cast<uint32_t>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int16_t POD_CAST::uint32_to_int16(uint32_t value)
{
    if (value > static_cast<uint32_t>(INT16_MAX))
    {
        return static_cast<int16_t>(INT16_MAX);
    }
    return static_cast<int16_t>(value);
}

int32_t POD_CAST::uint32_to_int32(uint32_t value)
{
    if (value > static_cast<uint32_t>(INT32_MAX))
    {
        return static_cast<int32_t>(INT32_MAX);
    }
    return static_cast<int32_t>(value);
}

int64_t POD_CAST::uint32_to_int64(uint32_t value)
{
    return static_cast<int64_t>(value);
}

uint8_t POD_CAST::uint32_to_uint8(uint32_t value)
{
    if (value > static_cast<uint32_t>(UINT8_MAX))
    {
        return static_cast<uint8_t>(UINT8_MAX);
    }
    return static_cast<uint8_t>(value);
}

uint16_t POD_CAST::uint32_to_uint16(uint32_t value)
{
    if (value > static_cast<uint32_t>(UINT16_MAX))
    {
        return static_cast<uint16_t>(UINT16_MAX);
    }
    return static_cast<uint16_t>(value);
}

uint64_t POD_CAST::uint32_to_uint64(uint32_t value)
{
    return static_cast<uint64_t>(value);
}

float POD_CAST::uint32_to_float(uint32_t value)
{
    return static_cast<float>(value);
}

double POD_CAST::uint32_to_double(uint32_t value)
{
    return static_cast<double>(value);
}

// uint64_t conversions
bool POD_CAST::uint64_to_bool(uint64_t value)
{
    return value != 0U;
}

int8_t POD_CAST::uint64_to_int8(uint64_t value)
{
    if (value > static_cast<uint64_t>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int16_t POD_CAST::uint64_to_int16(uint64_t value)
{
    if (value > static_cast<uint64_t>(INT16_MAX))
    {
        return static_cast<int16_t>(INT16_MAX);
    }
    return static_cast<int16_t>(value);
}

int32_t POD_CAST::uint64_to_int32(uint64_t value)
{
    if (value > static_cast<uint64_t>(INT32_MAX))
    {
        return static_cast<int32_t>(INT32_MAX);
    }
    return static_cast<int32_t>(value);
}

int64_t POD_CAST::uint64_to_int64(uint64_t value)
{
    if (value > static_cast<uint64_t>(INT64_MAX))
    {
        return static_cast<int64_t>(INT64_MAX);
    }
    return static_cast<int64_t>(value);
}

uint8_t POD_CAST::uint64_to_uint8(uint64_t value)
{
    if (value > static_cast<uint64_t>(UINT8_MAX))
    {
        return static_cast<uint8_t>(UINT8_MAX);
    }
    return static_cast<uint8_t>(value);
}

uint16_t POD_CAST::uint64_to_uint16(uint64_t value)
{
    if (value > static_cast<uint64_t>(UINT16_MAX))
    {
        return static_cast<uint16_t>(UINT16_MAX);
    }
    return static_cast<uint16_t>(value);
}

uint32_t POD_CAST::uint64_to_uint32(uint64_t value)
{
    if (value > static_cast<uint64_t>(UINT32_MAX))
    {
        return static_cast<uint32_t>(UINT32_MAX);
    }
    return static_cast<uint32_t>(value);
}

float POD_CAST::uint64_to_float(uint64_t value)
{
    return static_cast<float>(value);
}

double POD_CAST::uint64_to_double(uint64_t value)
{
    return static_cast<double>(value);
}

// float conversions
bool POD_CAST::float_to_bool(float value)
{
    return std::abs(static_cast<double>(value)) >= EPSILON; // not zero
}

int8_t POD_CAST::float_to_int8(float value)
{
    if (static_cast<double>(value) <= static_cast<double>(INT8_MIN))
    {
        return static_cast<int8_t>(INT8_MIN);
    }
    if (static_cast<double>(value) >= static_cast<double>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int16_t POD_CAST::float_to_int16(float value)
{
    if (static_cast<double>(value) <= static_cast<double>(INT16_MIN))
    {
        return static_cast<int16_t>(INT16_MIN);
    }
    if (static_cast<double>(value) >= static_cast<double>(INT16_MAX))
    {
        return static_cast<int16_t>(INT16_MAX);
    }
    return static_cast<int16_t>(value);
}

int32_t POD_CAST::float_to_int32(float value)
{
    if (static_cast<double>(value) <= static_cast<double>(INT32_MIN))
    {
        return static_cast<int32_t>(INT32_MIN);
    }
    if (static_cast<double>(value) >= static_cast<double>(INT32_MAX))
    {
        return static_cast<int32_t>(INT32_MAX);
    }
    return static_cast<int32_t>(value);
}

int64_t POD_CAST::float_to_int64(float value)
{
    if ((double)value <= static_cast<double>(INT64_MIN))
    {
        return static_cast<int64_t>(INT64_MIN);
    }
    if ((double)value >= static_cast<double>(INT64_MAX))
    {
        return static_cast<int64_t>(INT64_MAX);
    }
    return static_cast<int64_t>(value);
}

uint8_t POD_CAST::float_to_uint8(float value)
{
    if (static_cast<double>(value) <= 0.0)
    {
        return 0U;
    }
    if (static_cast<double>(value) >= static_cast<double>(UINT8_MAX))
    {
        return static_cast<uint8_t>(UINT8_MAX);
    }
    return static_cast<uint8_t>(value);
}

uint16_t POD_CAST::float_to_uint16(float value)
{
    if (static_cast<double>(value) <= 0.0)
    {
        return 0U;
    }
    if (static_cast<double>(value) >= static_cast<double>(UINT16_MAX))
    {
        return static_cast<uint16_t>(UINT16_MAX);
    }
    return static_cast<uint16_t>(value);
}

uint32_t POD_CAST::float_to_uint32(float value)
{
    if (static_cast<double>(value) <= 0.0)
    {
        return 0U;
    }
    if (static_cast<double>(value) >= static_cast<double>(UINT32_MAX))
    {
        return static_cast<uint32_t>(UINT32_MAX);
    }
    return static_cast<uint32_t>(value);
}

uint64_t POD_CAST::float_to_uint64(float value)
{
    if (static_cast<double>(value) <= 0.0)
    {
        return 0U;
    }
    if (static_cast<double>(value) >= static_cast<double>(UINT64_MAX))
    {
        return static_cast<uint64_t>(UINT64_MAX);
    }
    return static_cast<uint64_t>(value);
}

double POD_CAST::float_to_double(float value)
{
    return static_cast<double>(value);
}

// double conversions
bool POD_CAST::double_to_bool(double value)
{
    return std::abs(value) >= EPSILON; // not zero
}

int8_t POD_CAST::double_to_int8(double value)
{
    if (value <= static_cast<double>(INT8_MIN))
    {
        return static_cast<int8_t>(INT8_MIN);
    }
    if (value >= static_cast<double>(INT8_MAX))
    {
        return static_cast<int8_t>(INT8_MAX);
    }
    return static_cast<int8_t>(value);
}

int16_t POD_CAST::double_to_int16(double value)
{
    if (value <= static_cast<double>(INT16_MIN))
    {
        return static_cast<int16_t>(INT16_MIN);
    }
    if (value >= static_cast<double>(INT16_MAX))
    {
        return static_cast<int16_t>(INT16_MAX);
    }
    return static_cast<int16_t>(value);
}

int32_t POD_CAST::double_to_int32(double value)
{
    if (value <= static_cast<double>(INT32_MIN))
    {
        return static_cast<int32_t>(INT32_MIN);
    }
    if (value >= static_cast<double>(INT32_MAX))
    {
        return static_cast<int32_t>(INT32_MAX);
    }
    return static_cast<int32_t>(value);
}

int64_t POD_CAST::double_to_int64(double value)
{
    if (value <= static_cast<double>(INT64_MIN))
    {
        return static_cast<int64_t>(INT64_MIN);
    }
    if (value >= static_cast<double>(INT64_MAX))
    {
        return static_cast<int64_t>(INT64_MAX);
    }
    return static_cast<int64_t>(value);
}

uint8_t POD_CAST::double_to_uint8(double value)
{
    if (value <= 0.0)
    {
        return 0U;
    }
    if (value >= static_cast<double>(UINT8_MAX))
    {
        return static_cast<uint8_t>(UINT8_MAX);
    }
    return static_cast<uint8_t>(value);
}

uint16_t POD_CAST::double_to_uint16(double value)
{
    if (value <= 0.0)
    {
        return 0U;
    }
    if (value >= static_cast<double>(UINT16_MAX))
    {
        return static_cast<uint16_t>(UINT16_MAX);
    }
    return static_cast<uint16_t>(value);
}

uint32_t POD_CAST::double_to_uint32(double value)
{
    if (value <= 0.0)
    {
        return 0U;
    }
    if (value >= static_cast<double>(UINT32_MAX))
    {
        return static_cast<uint32_t>(UINT32_MAX);
    }
    return static_cast<uint32_t>(value);
}

uint64_t POD_CAST::double_to_uint64(double value)
{
    if (value <= 0.0)
    {
        return 0U;
    }
    if (value >= static_cast<double>(UINT64_MAX))
    {
        return static_cast<uint64_t>(UINT64_MAX);
    }
    return static_cast<uint64_t>(value);
}

float POD_CAST::double_to_float(double value)
{
    if (value <= static_cast<double>(-FLT_MAX))
    {
        return -FLT_MAX;
    }
    if (value >= static_cast<double>(FLT_MAX))
    {
        return FLT_MAX;
    }
    return static_cast<float>(value);
}

template<>
int8_t POD_CAST<bool, int8_t>(bool value)
{
    return POD_CAST::bool_to_int8(value);
}

template<>
int16_t POD_CAST<bool, int16_t>(bool value)
{
    return POD_CAST::bool_to_int16(value);
}

template<>
int32_t POD_CAST<bool, int32_t>(bool value)
{
    return POD_CAST::bool_to_int32(value);
}

template<>
int64_t POD_CAST<bool, int64_t>(bool value)
{
    return POD_CAST::bool_to_int64(value);
}

template<>
uint8_t POD_CAST<bool, uint8_t>(bool value)
{
    return POD_CAST::bool_to_uint8(value);
}

template<>
uint16_t POD_CAST<bool, uint16_t>(bool value)
{
    return POD_CAST::bool_to_uint16(value);
}

template<>
uint32_t POD_CAST<bool, uint32_t>(bool value)
{
    return POD_CAST::bool_to_uint32(value);
}

template<>
uint64_t POD_CAST<bool, uint64_t>(bool value)
{
    return POD_CAST::bool_to_uint64(value);
}

template<>
float POD_CAST<bool, float>(bool value)
{
    return POD_CAST::bool_to_float(value);
}

template<>
double POD_CAST<bool, double>(bool value)
{
    return POD_CAST::bool_to_double(value);
}

// int8_t conversions
template<>
bool POD_CAST<int8_t, bool>(int8_t value)
{
    return POD_CAST::int8_to_bool(value);
}

template<>
int16_t POD_CAST<int8_t, int16_t>(int8_t value)
{
    return POD_CAST::int8_to_int16(value);
}

template<>
int32_t POD_CAST<int8_t, int32_t>(int8_t value)
{
    return POD_CAST::int8_to_int32(value);
}

template<>
int64_t POD_CAST<int8_t, int64_t>(int8_t value)
{
    return POD_CAST::int8_to_int64(value);
}

template<>
uint8_t POD_CAST<int8_t, uint8_t>(int8_t value)
{
    return POD_CAST::int8_to_uint8(value);
}

template<>
uint16_t POD_CAST<int8_t, uint16_t>(int8_t value)
{
    return POD_CAST::int8_to_uint16(value);
}

template<>
uint32_t POD_CAST<int8_t, uint32_t>(int8_t value)
{
    return POD_CAST::int8_to_uint32(value);
}

template<>
uint64_t POD_CAST<int8_t, uint64_t>(int8_t value)
{
    return POD_CAST::int8_to_uint64(value);
}

template<>
float POD_CAST<int8_t, float>(int8_t value)
{
    return POD_CAST::int8_to_float(value);
}

template<>
double POD_CAST<int8_t, double>(int8_t value)
{
    return POD_CAST::int8_to_double(value);
}

// int16_t conversions
template<>
bool POD_CAST<int16_t, bool>(int16_t value)
{
    return POD_CAST::int16_to_bool(value);
}

template<>
int8_t POD_CAST<int16_t, int8_t>(int16_t value)
{
    return POD_CAST::int16_to_int8(value);
}

template<>
int32_t POD_CAST<int16_t, int32_t>(int16_t value)
{
    return POD_CAST::int16_to_int32(value);
}

template<>
int64_t POD_CAST<int16_t, int64_t>(int16_t value)
{
    return POD_CAST::int16_to_int64(value);
}

template<>
uint8_t POD_CAST<int16_t, uint8_t>(int16_t value)
{
    return POD_CAST::int16_to_uint8(value);
}

template<>
uint16_t POD_CAST<int16_t, uint16_t>(int16_t value)
{
    return POD_CAST::int16_to_uint16(value);
}

template<>
uint32_t POD_CAST<int16_t, uint32_t>(int16_t value)
{
    return POD_CAST::int16_to_uint32(value);
}

template<>
uint64_t POD_CAST<int16_t, uint64_t>(int16_t value)
{
    return POD_CAST::int16_to_uint64(value);
}

template<>
float POD_CAST<int16_t, float>(int16_t value)
{
    return POD_CAST::int16_to_float(value);
}

template<>
double POD_CAST<int16_t, double>(int16_t value)
{
    return POD_CAST::int16_to_double(value);
}

// int32_t conversions
template<>
bool POD_CAST<int32_t, bool>(int32_t value)
{
    return POD_CAST::int32_to_bool(value);
}

template<>
int8_t POD_CAST<int32_t, int8_t>(int32_t value)
{
    return POD_CAST::int32_to_int8(value);
}

template<>
int16_t POD_CAST<int32_t, int16_t>(int32_t value)
{
    return POD_CAST::int32_to_int16(value);
}

template<>
int64_t POD_CAST<int32_t, int64_t>(int32_t value)
{
    return POD_CAST::int32_to_int64(value);
}

template<>
uint8_t POD_CAST<int32_t, uint8_t>(int32_t value)
{
    return POD_CAST::int32_to_uint8(value);
}

template<>
uint16_t POD_CAST<int32_t, uint16_t>(int32_t value)
{
    return POD_CAST::int32_to_uint16(value);
}

template<>
uint32_t POD_CAST<int32_t, uint32_t>(int32_t value)
{
    return POD_CAST::int32_to_uint32(value);
}

template<>
uint64_t POD_CAST<int32_t, uint64_t>(int32_t value)
{
    return POD_CAST::int32_to_uint64(value);
}

template<>
float POD_CAST<int32_t, float>(int32_t value)
{
    return POD_CAST::int32_to_float(value);
}

template<>
double POD_CAST<int32_t, double>(int32_t value)
{
    return POD_CAST::int32_to_double(value);
}

// int64_t conversions
template<>
bool POD_CAST<int64_t, bool>(int64_t value)
{
    return POD_CAST::int64_to_bool(value);
}

template<>
int8_t POD_CAST<int64_t, int8_t>(int64_t value)
{
    return POD_CAST::int64_to_int8(value);
}

template<>
int16_t POD_CAST<int64_t, int16_t>(int64_t value)
{
    return POD_CAST::int64_to_int16(value);
}

template<>
int32_t POD_CAST<int64_t, int32_t>(int64_t value)
{
    return POD_CAST::int64_to_int32(value);
}

template<>
uint8_t POD_CAST<int64_t, uint8_t>(int64_t value)
{
    return POD_CAST::int64_to_uint8(value);
}

template<>
uint16_t POD_CAST<int64_t, uint16_t>(int64_t value)
{
    return POD_CAST::int64_to_uint16(value);
}

template<>
uint32_t POD_CAST<int64_t, uint32_t>(int64_t value)
{
    return POD_CAST::int64_to_uint32(value);
}

template<>
uint64_t POD_CAST<int64_t, uint64_t>(int64_t value)
{
    return POD_CAST::int64_to_uint64(value);
}

template<>
float POD_CAST<int64_t, float>(int64_t value)
{
    return POD_CAST::int64_to_float(value);
}

template<>
double POD_CAST<int64_t, double>(int64_t value)
{
    return POD_CAST::int64_to_double(value);
}

// uint8_t conversions
template<>
bool POD_CAST<uint8_t, bool>(uint8_t value)
{
    return POD_CAST::uint8_to_bool(value);
}

template<>
int8_t POD_CAST<uint8_t, int8_t>(uint8_t value)
{
    return POD_CAST::uint8_to_int8(value);
}

template<>
int16_t POD_CAST<uint8_t, int16_t>(uint8_t value)
{
    return POD_CAST::uint8_to_int16(value);
}

template<>
int32_t POD_CAST<uint8_t, int32_t>(uint8_t value)
{
    return POD_CAST::uint8_to_int32(value);
}

template<>
int64_t POD_CAST<uint8_t, int64_t>(uint8_t value)
{
    return POD_CAST::uint8_to_int64(value);
}

template<>
uint16_t POD_CAST<uint8_t, uint16_t>(uint8_t value)
{
    return POD_CAST::uint8_to_uint16(value);
}

template<>
uint32_t POD_CAST<uint8_t, uint32_t>(uint8_t value)
{
    return POD_CAST::uint8_to_uint32(value);
}

template<>
uint64_t POD_CAST<uint8_t, uint64_t>(uint8_t value)
{
    return POD_CAST::uint8_to_uint64(value);
}

template<>
float POD_CAST<uint8_t, float>(uint8_t value)
{
    return POD_CAST::uint8_to_float(value);
}

template<>
double POD_CAST<uint8_t, double>(uint8_t value)
{
    return POD_CAST::uint8_to_double(value);
}

// uint16_t conversions
template<>
bool POD_CAST<uint16_t, bool>(uint16_t value)
{
    return POD_CAST::uint16_to_bool(value);
}

template<>
int8_t POD_CAST<uint16_t, int8_t>(uint16_t value)
{
    return POD_CAST::uint16_to_int8(value);
}

template<>
int16_t POD_CAST<uint16_t, int16_t>(uint16_t value)
{
    return POD_CAST::uint16_to_int16(value);
}

template<>
int32_t POD_CAST<uint16_t, int32_t>(uint16_t value)
{
    return POD_CAST::uint16_to_int32(value);
}

template<>
int64_t POD_CAST<uint16_t, int64_t>(uint16_t value)
{
    return POD_CAST::uint16_to_int64(value);
}

template<>
uint8_t POD_CAST<uint16_t, uint8_t>(uint16_t value)
{
    return POD_CAST::uint16_to_uint8(value);
}

template<>
uint32_t POD_CAST<uint16_t, uint32_t>(uint16_t value)
{
    return POD_CAST::uint16_to_uint32(value);
}

template<>
uint64_t POD_CAST<uint16_t, uint64_t>(uint16_t value)
{
    return POD_CAST::uint16_to_uint64(value);
}

template<>
float POD_CAST<uint16_t, float>(uint16_t value)
{
    return POD_CAST::uint16_to_float(value);
}

template<>
double POD_CAST<uint16_t, double>(uint16_t value)
{
    return POD_CAST::uint16_to_double(value);
}

// uint32_t conversions
template<>
bool POD_CAST<uint32_t, bool>(uint32_t value)
{
    return POD_CAST::uint32_to_bool(value);
}

template<>
int8_t POD_CAST<uint32_t, int8_t>(uint32_t value)
{
    return POD_CAST::uint32_to_int8(value);
}

template<>
int16_t POD_CAST<uint32_t, int16_t>(uint32_t value)
{
    return POD_CAST::uint32_to_int16(value);
}

template<>
int32_t POD_CAST<uint32_t, int32_t>(uint32_t value)
{
    return POD_CAST::uint32_to_int32(value);
}

template<>
int64_t POD_CAST<uint32_t, int64_t>(uint32_t value)
{
    return POD_CAST::uint32_to_int64(value);
}

template<>
uint8_t POD_CAST<uint32_t, uint8_t>(uint32_t value)
{
    return POD_CAST::uint32_to_uint8(value);
}

template<>
uint16_t POD_CAST<uint32_t, uint16_t>(uint32_t value)
{
    return POD_CAST::uint32_to_uint16(value);
}

template<>
uint64_t POD_CAST<uint32_t, uint64_t>(uint32_t value)
{
    return POD_CAST::uint32_to_uint64(value);
}

template<>
float POD_CAST<uint32_t, float>(uint32_t value)
{
    return POD_CAST::uint32_to_float(value);
}

template<>
double POD_CAST<uint32_t, double>(uint32_t value)
{
    return POD_CAST::uint32_to_double(value);
}

// uint64_t conversions
template<>
bool POD_CAST<uint64_t, bool>(uint64_t value)
{
    return POD_CAST::uint64_to_bool(value);
}

template<>
int8_t POD_CAST<uint64_t, int8_t>(uint64_t value)
{
    return POD_CAST::uint64_to_int8(value);
}

template<>
int16_t POD_CAST<uint64_t, int16_t>(uint64_t value)
{
    return POD_CAST::uint64_to_int16(value);
}

template<>
int32_t POD_CAST<uint64_t, int32_t>(uint64_t value)
{
    return POD_CAST::uint64_to_int32(value);
}

template<>
int64_t POD_CAST<uint64_t, int64_t>(uint64_t value)
{
    return POD_CAST::uint64_to_int64(value);
}

template<>
uint8_t POD_CAST<uint64_t, uint8_t>(uint64_t value)
{
    return POD_CAST::uint64_to_uint8(value);
}

template<>
uint16_t POD_CAST<uint64_t, uint16_t>(uint64_t value)
{
    return POD_CAST::uint64_to_uint16(value);
}

template<>
uint32_t POD_CAST<uint64_t, uint32_t>(uint64_t value)
{
    return POD_CAST::uint64_to_uint32(value);
}

template<>
float POD_CAST<uint64_t, float>(uint64_t value)
{
    return POD_CAST::uint64_to_float(value);
}

template<>
double POD_CAST<uint64_t, double>(uint64_t value)
{
    return POD_CAST::uint64_to_double(value);
}

// float conversions
template<>
bool POD_CAST<float, bool>(float value)
{
    return POD_CAST::float_to_bool(value);
}

template<>
int8_t POD_CAST<float, int8_t>(float value)
{
    return POD_CAST::float_to_int8(value);
}

template<>
int16_t POD_CAST<float, int16_t>(float value)
{
    return POD_CAST::float_to_int16(value);
}

template<>
int32_t POD_CAST<float, int32_t>(float value)
{
    return POD_CAST::float_to_int32(value);
}

template<>
int64_t POD_CAST<float, int64_t>(float value)
{
    return POD_CAST::float_to_int64(value);
}

template<>
uint8_t POD_CAST<float, uint8_t>(float value)
{
    return POD_CAST::float_to_uint8(value);
}

template<>
uint16_t POD_CAST<float, uint16_t>(float value)
{
    return POD_CAST::float_to_uint16(value);
}

template<>
uint32_t POD_CAST<float, uint32_t>(float value)
{
    return POD_CAST::float_to_uint32(value);
}

template<>
uint64_t POD_CAST<float, uint64_t>(float value)
{
    return POD_CAST::float_to_uint64(value);
}

template<>
double POD_CAST<float, double>(float value)
{
    return POD_CAST::float_to_double(value);
}

// double conversions
template<>
bool POD_CAST<double, bool>(double value)
{
    return POD_CAST::double_to_bool(value);
}

template<>
int8_t POD_CAST<double, int8_t>(double value)
{
    return POD_CAST::double_to_int8(value);
}

template<>
int16_t POD_CAST<double, int16_t>(double value)
{
    return POD_CAST::double_to_int16(value);
}

template<>
int32_t POD_CAST<double, int32_t>(double value)
{
    return POD_CAST::double_to_int32(value);
}

template<>
int64_t POD_CAST<double, int64_t>(double value)
{
    return POD_CAST::double_to_int64(value);
}

template<>
uint8_t POD_CAST<double, uint8_t>(double value)
{
    return POD_CAST::double_to_uint8(value);
}

template<>
uint16_t POD_CAST<double, uint16_t>(double value)
{
    return POD_CAST::double_to_uint16(value);
}

template<>
uint32_t POD_CAST<double, uint32_t>(double value)
{
    return POD_CAST::double_to_uint32(value);
}

template<>
uint64_t POD_CAST<double, uint64_t>(double value)
{
    return POD_CAST::double_to_uint64(value);
}

template<>
float POD_CAST<double, float>(double value)
{
    return POD_CAST::double_to_float(value);
}