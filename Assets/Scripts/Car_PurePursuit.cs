using System;
using UnityEngine;

public class Car_PurePursuit: WaypointCar
{
    [Header("-----Car_PurePursuit-----")]
    [SerializeField] [Tooltip("The base distance for the AI too look ahead")] private float lookAheadConstant = 5;
    [SerializeField] [Tooltip("The multiplier per step of look ahead")] private float lookAheadMultiplier = 2;
    [SerializeField] [Tooltip("The amount of look ahead steps")] private float lookAheadSteps = 3;
    [SerializeField] [Range(1,2)] private float safety = 1f;

    protected override void OnDrawGizmos()
    {
        base.OnDrawGizmos();

        float lookAheadDistance = lookAheadConstant;
        float gizmoSize = 0.5f;
        for (int i = 0; i < lookAheadSteps; i++)
        {
            Gizmos.DrawSphere(GetLookAheadPointOnTrack(ProjectPositionOntoTrack(), lookAheadDistance), gizmoSize);

            lookAheadDistance *= lookAheadMultiplier;
            gizmoSize -= 0.1f;
        }
    }

    protected override void Update()
    {
        CalculateStearing();
        CalculateThrottle();
        base.Update();
    }

    #region Helper Functions
    private Vector3 ProjectPositionOntoTrack()
    {
        return ProjectPointOntoLine(transform.position, waypoints[currentWaypointIndex - 1], waypoints[currentWaypointIndex]);
    }

    private Vector3 GetLookAheadPointOnTrack(Vector3 currentTrackPoint, float lookAheadDistance)
    {
        int waypointIndex = currentWaypointIndex;

        int safety = 10;

        while ((waypoints[waypointIndex] - currentTrackPoint).magnitude < lookAheadDistance)
        {
            lookAheadDistance -= (waypoints[waypointIndex] - currentTrackPoint).magnitude;
            currentTrackPoint = waypoints[waypointIndex];
            waypointIndex++;

            if (safety < 0) { return Vector3.zero; }
            safety--;
        }

        Vector3 direction = (waypoints[waypointIndex] - currentTrackPoint).normalized;
        return currentTrackPoint + direction * lookAheadDistance;
    }

    private Vector3 ProjectPointOntoLine(Vector3 point, Vector3 linePoint1, Vector3 linePoint2)
    {
        Vector3 waypointVector = linePoint2 - linePoint1;
        Vector3 positionVector = point - linePoint1;

        Vector3 projectedPoint = linePoint1 + Vector3.Dot(positionVector, waypointVector) / Vector3.Dot(waypointVector, waypointVector) * waypointVector;

        return projectedPoint;
    }
    #endregion

    #region Throttle
    private void CalculateThrottle()
    {
        float lookAheadDistance = lookAheadConstant;
        bool turnable = true;
        for (int i = 0; i < lookAheadSteps; i++)
        {
            lookAheadDistance *= lookAheadMultiplier;
            if (!IsTurnable(lookAheadDistance))
                turnable = false;
        }

        throttle = turnable ? 1f : -1f;
    }

    private bool IsTurnable(float lookAheadDistance)
    {
        Vector3 lookAheadPoint = GetLookAheadPointOnTrack(ProjectPositionOntoTrack(), lookAheadDistance);

        Vector3 lookAheadVector = lookAheadPoint - transform.position;
        Vector3 velocity = transform.forward * speed;

        float angleToLookAhead = Vector3.Angle(velocity, lookAheadVector);
        float distanceToLookAhead = lookAheadVector.magnitude;

        float timeToLookAhead = distanceToLookAhead / speed;
        float maxTurnableToWaypoint = timeToLookAhead * turnspeed;

        return maxTurnableToWaypoint >= angleToLookAhead*safety;
    }
    #endregion

    #region Stearing
    private void CalculateStearing()
    {
        Vector3 lookAheadPoint = GetLookAheadPointOnTrack(ProjectPositionOntoTrack(), lookAheadConstant);

        Vector3 lookAheadVector = lookAheadPoint - transform.position;
        Vector3 velocity = transform.forward * speed;

        float signedAngleToLookAhead = Vector3.SignedAngle(velocity, lookAheadVector, Vector3.up);
        float stearingTowardsLookAheadAngle = Mathf.Clamp(signedAngleToLookAhead / maxStearingError, -1f, 1f);

        stearing = Mathf.Clamp(stearingTowardsLookAheadAngle, -1f, 1f);
    }
    #endregion
}
