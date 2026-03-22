#ifndef GEO_DISTANCE_CALCULATION_H_
#define GEO_DISTANCE_CALCULATION_H_

#include <cstdint>
#include <cstdbool>
#include <cmath>
#include <algorithm>

struct GeoDistanceAndBearing
{
    float mDistance;
    float mInitialBearing;
    float mFinalBearing;
};

// Vincenty Inverse Formula
void computeVincentyInverse(double lat1, double lon1, double lat2, double lon2, GeoDistanceAndBearing& results)
{
    // Based on http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf
    // using the "Inverse Formula" (section 4)

    const int MAXITERS = 20;
    // Convert lat/long to radians
    lat1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    lon1 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;

    const double a = 6378137.0; // WGS84 major axis
    const double b = 6356752.3142; // WGS84 semi-major axis
    const double f = (a - b) / a;
    const double aSqMinusBSqOverBSq = (a * a - b * b) / (b * b);

    const double L = lon2 - lon1;
    double A = 0.0;
    const double U1 = atan((1.0 - f) * tan(lat1));
    const double U2 = atan((1.0 - f) * tan(lat2));

    const double cosU1 = cos(U1);
    const double cosU2 = cos(U2);
    const double sinU1 = sin(U1);
    const double sinU2 = sin(U2);
    const double cosU1cosU2 = cosU1 * cosU2;
    const double sinU1sinU2 = sinU1 * sinU2;

    double sigma = 0.0;
    double deltaSigma = 0.0;
    double cosSqAlpha = 0.0;
    double cos2SM = 0.0;
    double cosSigma = 0.0;
    double sinSigma = 0.0;
    double cosLambda = 0.0;
    double sinLambda = 0.0;

    double lambda = L; // initial guess
    for (int iter = 0; iter < MAXITERS; iter++) {
        const double lambdaOrig = lambda;
        cosLambda = cos(lambda);
        sinLambda = sin(lambda);
        const double t1 = cosU2 * sinLambda;
        const double t2 = cosU1 * sinU2 - sinU1 * cosU2 * cosLambda;
        const double sinSqSigma = t1 * t1 + t2 * t2; // (14)
        sinSigma = sqrt(sinSqSigma);
        cosSigma = sinU1sinU2 + cosU1cosU2 * cosLambda; // (15)
        sigma = atan2(sinSigma, cosSigma); // (16)
        const double sinAlpha = (std::fabs(sinSigma) < 1e-9) ? 0.0 :
            cosU1cosU2 * sinLambda / sinSigma; // (17)
        cosSqAlpha = 1.0 - sinAlpha * sinAlpha;
        cos2SM = (std::fabs(cosSqAlpha) < 1e-9) ? 0.0 :
            cosSigma - 2.0 * sinU1sinU2 / cosSqAlpha; // (18)

        const double uSquared = cosSqAlpha * aSqMinusBSqOverBSq; // defn
        A = 1.0 + (uSquared / 16384.0) * // (3)
            (4096.0 + uSquared *
            (-768.0 + uSquared * (320.0 - 175.0 * uSquared)));
        const double B = (uSquared / 1024.0) * // (4)
            (256.0 + uSquared *
            (-128.0 + uSquared * (74.0 - 47.0 * uSquared)));
        const double C = (f / 16.0) *
            cosSqAlpha *
            (4.0 + f * (4.0 - 3.0 * cosSqAlpha)); // (10)
        const double cos2SMSq = cos2SM * cos2SM;
            deltaSigma = B * sinSigma * // (6)
            (cos2SM + (B / 4.0) *
            (cosSigma * (-1.0 + 2.0 * cos2SMSq) -
            (B / 6.0) * cos2SM *
            (-3.0 + 4.0 * sinSigma * sinSigma) *
            (-3.0 + 4.0 * cos2SMSq)));

        lambda = L +
            (1.0 - C) * f * sinAlpha *
            (sigma + C * sinSigma *
            (cos2SM + C * cosSigma *
            (-1.0 + 2.0 * cos2SM * cos2SM))); // (11)

        const double delta = (lambda - lambdaOrig) / lambda;
        if (std::abs(delta) < 1.0e-12) {
            break;
        }
    }

    const float distance = static_cast<float>(b * A * (sigma - deltaSigma));
    results.mDistance = distance;
    float initialBearing = static_cast<float> (atan2(cosU2 * sinLambda,
        cosU1 * sinU2 - sinU1 * cosU2 * cosLambda));
    initialBearing *= 180.0F / M_PI;
    results.mInitialBearing = initialBearing;
    float finalBearing = static_cast<float> (atan2(cosU1 * sinLambda,
            -sinU1 * cosU2 + cosU1 * sinU2 * cosLambda));
    finalBearing *= 180.0F / M_PI;
    results.mFinalBearing = finalBearing;
}

#endif // GEO_DISTANCE_CALCULATION_H_