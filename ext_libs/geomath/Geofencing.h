#ifndef REGIONAL_H
#define REGIONAL_H

#include "Geometric.h"
#include "singleton.h"

#include <vector>
#include <memory>
#include <string>

class Region
{
public:
    Region(const Polygon& boundary);

    bool checkPointInside(const Point& pos) const;

private:
    Polygon mBoundary;
};

class Geofencing
{
    friend class UniqueInstance<Geofencing>;

public:
    enum REGION_ID
    {
        REGION_UNKNOWN = 0,
        REGION_CHN,

        REGION_TOTAL
    };

    virtual ~Geofencing();

    void UpdateRegion(double lon, double lat);

    inline bool IsChinaRegion() const
    {
        return mGeofenceRegionID == static_cast<int8_t>(REGION_ID::REGION_CHN);
    }

private:
    explicit Geofencing();

private:
    int8_t mGeofenceRegionID;

    // Below are the list of region to manage
    static Region mChinaRegion;
};

#endif // REGIONAL_H