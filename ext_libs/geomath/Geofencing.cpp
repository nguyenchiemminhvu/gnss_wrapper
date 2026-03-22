#include "Geofencing.h"

#include "logging.h"

const std::vector<Point> chinaBoundary =
{  // lon               , lat
    { 107.8973733121041 , 21.29206943536157 },
    { 108.6066184329415 , 17.75823240643058 },
    { 125.9130060440320 , 18.90784319702411 },
    { 128.5628212270744 , 31.13811552514781 },
    { 124.9370956116744 , 33.81966914945097 },
    { 124.0232085095437 , 39.13082542030939 },
    { 131.9132668674246 , 41.52880010079262 },
    { 135.5923390136213 , 48.37896049292777 },
    { 127.4948282640856 , 53.24011751868090 },
    { 121.8292048750092 , 54.51500942013367 },
    { 111.8942585488605 , 45.45218452643208 },
    { 109.8113155785911 , 42.94257243019249 },
    { 96.84425525666065 , 43.08617185473222 },
    { 96.28206660835602 , 44.38961290887570 },
    { 91.46062858375311 , 45.53328504242598 },
    { 91.42377680197887 , 47.77586155528959 },
    { 87.00984935333790 , 49.58468867471388 },
    { 82.70286735723320 , 47.46079695870164 },
    { 79.66597344728299 , 45.26213655741632 },
    { 79.86445429887883 , 42.68018245032157 },
    { 74.42057874144240 , 40.86161040366930 },
    { 73.31406885411333 , 39.43855188692437 },
    { 72.60879732558129 , 36.40141615382820 },
    { 73.94200791623562 , 33.26550801710336 },
    { 83.90999314707722 , 27.77975358751668 },
    { 90.47821944445789 , 27.11538043714165 },
    { 97.61237960254650 , 27.07390912431352 },
    { 96.81733635768252 , 23.65001692519949 },
    { 101.0014362766858 , 20.58000957873124 },
    { 105.3101117918342 , 22.99096565142264 }
};

Region Geofencing::mChinaRegion = Region(chinaBoundary);

Region::Region(const Polygon& boundary)
    : mBoundary(boundary)
{

}

bool Region::checkPointInside(const Point& pos) const
{
    bool ret = false;

    if (mBoundary.isPointInside_JordanCurveTheorem(pos))
    {
        ret = true;
    }

    return ret;
}

Geofencing::Geofencing()
    : mGeofenceRegionID(static_cast<int8_t>(REGION_ID::REGION_UNKNOWN))
{
}

Geofencing::~Geofencing()
{
}

void Geofencing::UpdateRegion(double lon, double lat)
{
    REGION_ID reg = REGION_ID::REGION_UNKNOWN;
    Point curPos(lon, lat);

    if (Geofencing::mChinaRegion.checkPointInside(curPos))
    {
        reg = REGION_ID::REGION_CHN;
    }
    else
    {
        reg = REGION_ID::REGION_UNKNOWN;
    }

    mGeofenceRegionID = static_cast<int8_t>(reg);
    LOG_INFO("[Geofencing] Geofence algorithm successfully identified user region: [", static_cast<int>(mGeofenceRegionID), "]");
}