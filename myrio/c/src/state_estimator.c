#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define RADIUS_OF_THE_EARTH 6371000.0

/**
 * Estimates roll using a complementary filter
 *
 * @param Ts Time step [s]
 * @param wheelbase Distance between contact points of the bike's wheels [m]
 * @param forkAngle Fork angle of the bike [rad]
 * @param speed Approximative current speed of bike, e.g. reference speed [m/s]
 * @param steeringAngle Current sterring angle of bike [rad]
 * @param gyroX Accelerometer roll rate (around X axis) [rad/s]
 * @param accY Accelerometer Y value [m/s²]
 * @param accZ Accelerometer Z value [m/s²]
 * @return Approximated roll angle (around X axis) [rad]
 *
 * @author Ossian Eriksson
 */
static double rollComplementaryFilter(double Ts, double wheelbase, double forkAngle, double speed, double steeringAngle, double gyroX, double accY, double accZ)
{
    static double lastEstimatedRoll = 0;

    // From Umer's report. He used a time step of 0.04 seconds
    const double Cref = 0.985;
    const double TSref = 0.04;

    // Update Umer's C value to match our time step
    double timeConstant = Cref * TSref / (1 - Cref);
    double C = timeConstant / (timeConstant + Ts);
    
    double ac = speed * speed / wheelbase * tan(steeringAngle) * sin(forkAngle);
    double accelerationRoll = atan2(accY - ac * cos(lastEstimatedRoll), accZ + ac * sin(lastEstimatedRoll));

    // Estimated roll is LP filter applied to acceleration roll approximation + HP filter applied to roll rate roll approximation
    double estimatedRoll = (1 - C) * accelerationRoll + C * (lastEstimatedRoll + Ts * gyroX);
    lastEstimatedRoll = estimatedRoll;
    return estimatedRoll;
}

/**
 * Sets *value to *lastValue if *value is NaN.
 * Otherwise *lastValue is set to *value.
 * 
 * @param value Current value
 * @param lastValue The last known good value
 * 
 * @author Ossian Eriksson
*/
static void useLastValueIfNaN(double *value, double *lastValue)
{
    if (isnan(*value))
    {
        *value = *lastValue;
    }
    else
    {
        *lastValue = *value;
    }
}

/**
 * Estimates roll using a complementary filter
 *
 * @param X Output for x position [m]
 * @param Y Output for y position [m]
 * @param Psi Output for track/yaw angle [rad]
 * @param roll Output for roll angle [rad]
 * @param rollRate Output for time derivative of roll angle [rad/s]
 * @param Ts Time step [s]
 * @param wheelbase Distance between contact points of the bike's wheels [m]
 * @param forkAngle Fork angle of the bike [rad]
 * @param latitude GPS latitude [rad]
 * @param longitude GPS longitude [rad]
 * @param speed GPS speed [m/s]
 * @param headingAngle GPS headingAngle [m/s]
 * @param steeringAngle Current sterring angle of bike [rad]
 * @param gyroX IMU roll rate (around X axis) [rad/s]
 * @param gyroY IMU pitch rate (around Y axis) [rad/s]
 * @param gyroZ IMU yaw rate (around Z axis) [rad/s]
 * @param accY Accelerometer X value [m/s²]
 * @param accY Accelerometer Y value [m/s²]
 * @param accZ Accelerometer Z value [m/s²]
 *
 * @author Ossian Eriksson
 */
extern void stateEstimator(double *X, double *Y, double *Psi, double *roll, double *rollRate, double Ts, double wheelbase, double forkAngle, double latitude, double longitude, double speed, double headingAngle, double steeringAngle, double gyroX, double gyroY, double gyroZ, double accX, double accY, double accZ)
{
    static bool initializedLatLon = false;
    static double latitude0, longitude0;

    static double lastLatitude = NAN;
    static double lastLongitude = NAN;
    static double lastSpeed = 0;
    static double lastHeadingAngle = NAN;

    useLastValueIfNaN(&latitude, &lastLatitude);
    useLastValueIfNaN(&longitude, &lastLongitude);
    useLastValueIfNaN(&speed, &lastSpeed);
    useLastValueIfNaN(&headingAngle, &lastHeadingAngle);

    if (!initializedLatLon && !isnan(latitude) && !isnan(longitude))
    {
        latitude0 = latitude;
        longitude0 = longitude;

        initializedLatLon = true;
    }

    if (initializedLatLon)
    {
        *X = RADIUS_OF_THE_EARTH * (longitude - longitude0) * cos(latitude0);
        *Y = RADIUS_OF_THE_EARTH * (latitude - latitude0);
    }

    *Psi = headingAngle;
    *roll = rollComplementaryFilter(Ts, wheelbase, forkAngle, speed, steeringAngle, gyroX, accY, accZ);
    *rollRate = gyroX;
}
