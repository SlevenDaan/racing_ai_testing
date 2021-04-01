using UnityEngine;

public class Car_WaypointFollow : WaypointCar
{
    protected override void Update()
    {
        CalculateStearing();
        CalculateThrottle();
        base.Update();
    }

    private void CalculateStearing()
    {
        Vector3 vectorTowardsWaypoint = waypoints[currentWaypointIndex] - transform.position;
        float angleToWaypoint = Vector3.SignedAngle(transform.forward, vectorTowardsWaypoint, Vector3.up);
        stearing = Mathf.Clamp(angleToWaypoint / maxStearingError, -1f, 1f);
    }

    private void CalculateThrottle()
    {
        bool canTurn = true;

        float requiredAngleToReachWaypoint = 0;
        float distanceToWaypoint = 0;
        for (int i = 0; i < 4; i++)
        {
            Vector3 waypoint = waypoints[currentWaypointIndex + i];
            Vector3 currentDirection = i != 0 ? (waypoints[currentWaypointIndex + i + 1] - waypoints[currentWaypointIndex + i]) : transform.forward;
            Vector3 vectorToWaypoint = waypoint - transform.position;

            requiredAngleToReachWaypoint += Vector3.Angle(currentDirection, vectorToWaypoint);
            distanceToWaypoint += vectorToWaypoint.magnitude;

            float timeToWaypoint = distanceToWaypoint / speed;
            float maxTurnableToWaypoint = timeToWaypoint * turnspeed;

            if (maxTurnableToWaypoint < requiredAngleToReachWaypoint)
            {
                canTurn = false;
            }
        }

        throttle = canTurn ? 1f : -1f;
    }
}
