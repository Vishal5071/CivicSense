#include <iostream>
#include <cmath>
#include <utility> // for std::pair

const double PI = 3.14159265358979323846;

// ConvertTM (EPSG:32244 — WGS72 / UTM Zone 44N) → Latitude & Longitude (WGS72)
std::pair<double, double> utm32244ToLatLon(double easting, double northing) {
    // --- Constants for WGS72 Ellipsoid ---
    const double a = 6378135.0;              // Semi-major axis (meters)
    const double f = 1.0 / 298.26;           // Flattening
    const double k0 = 0.9996;                // Scale factor
    const double e = std::sqrt(2 * f - f * f); // Eccentricity

    // --- UTM Zone 44N Parameters ---
    const double lon0_deg = 81.0;            // Central meridian (degrees)
    const double falseEasting = 500000.0;    // Meters
    const double falseNorthing = 0.0;        // Meters (for northern hemisphere)

    // --- Remove false origins ---
    double x = easting - falseEasting;
    double y = northing - falseNorthing;

    // --- Compute Footprint Latitude ---
    double M = y / k0;
    double mu = M / (a * (1 - pow(e,2)/4 - 3*pow(e,4)/64 - 5*pow(e,6)/256));

    double e1 = (1 - std::sqrt(1 - pow(e,2))) / (1 + std::sqrt(1 - pow(e,2)));

    double J1 = (3*e1/2 - 27*pow(e1,3)/32);
    double J2 = (21*pow(e1,2)/16 - 55*pow(e1,4)/32);
    double J3 = (151*pow(e1,3)/96);
    double J4 = (1097*pow(e1,4)/512);

    double fp = mu + J1*std::sin(2*mu) + J2*std::sin(4*mu) + J3*std::sin(6*mu) + J4*std::sin(8*mu);

    // --- Compute Latitude and Longitude ---
    double e2 = (pow(e*a,2) / pow(a,2 - pow(e,2)));
    double C1 = pow(e,2) / (1 - pow(e,2)) * pow(std::cos(fp),2);
    double T1 = pow(std::tan(fp),2);
    double R1 = a*(1 - pow(e,2)) / pow(1 - pow(e*std::sin(fp),2), 1.5);
    double N1 = a / std::sqrt(1 - pow(e*std::sin(fp),2));
    double D = x / (N1 * k0);

    double lat = fp - (N1*std::tan(fp)/R1) * (pow(D,2)/2 - (5 + 3*T1 + 10*C1 - 4*pow(C1,2) - 9*e2)*pow(D,4)/24
               + (61 + 90*T1 + 298*C1 + 45*pow(T1,2) - 252*e2 - 3*pow(C1,2))*pow(D,6)/720);

    double lon = (D - (1 + 2*T1 + C1)*pow(D,3)/6
               + (5 - 2*C1 + 28*T1 - 3*pow(C1,2) + 8*e2 + 24*pow(T1,2))*pow(D,5)/120) / std::cos(fp);

    // --- Convert from radians to degrees ---
    lat = lat * 180.0 / PI;
    lon = lon0_deg + lon * 180.0 / PI;

    return std::make_pair(lat, lon);
}
