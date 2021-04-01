using System;
using UnityEngine;

public class Car_PurePursuitPlus_Old : WaypointCar
{
    [Header("-----Car_PurePursuitPlus_Old-----")]
    [SerializeField] [Tooltip("The base distance for the AI too look ahead")] private float lookAheadConstant = 5;
    [SerializeField] [Tooltip("The multiplier per step of look ahead")] private float lookAheadMultiplier = 2;
    [SerializeField] [Tooltip("The amount of look ahead steps")] private float lookAheadSteps = 3;
    [SerializeField] [Range(1,2)] private float safety = 1f;
    [Space]
    [SerializeField] private bool avoidCollision = false;
    [SerializeField] private LayerMask collisionMask;
    [SerializeField] private float frontalCollisionDetectionDistance = 5f;
    [SerializeField] private float sideCollisionDetectionDistance = 3f;
    [SerializeField] private float frontalCollisionOffset = 0.8f;
    [SerializeField] private float sideCollisionOffset = 0.4f;
    [SerializeField] private float preferedCollisionAvoidanceAngle = 0.5f;
    [SerializeField] private float maxOfftrackDistance = 1.5f;

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

        Vector3 vectorToTrack = ProjectPositionOntoTrack() - transform.position;
        float angleToTrack = Vector3.Angle(transform.forward, vectorToTrack);
        float collisionAvoidanceAngle = 0;// = avoidCollision ? GetCollisionAvoidance(Mathf.Sign(signedAngleToLookAhead) * 0.5f) : 0f;
        if (avoidCollision)
        {
            if (vectorToTrack.magnitude < maxOfftrackDistance)
            {
                collisionAvoidanceAngle = GetCollisionAvoidance(preferedCollisionAvoidanceAngle);
            }
        }
        collisionAvoidanceAngle = GetCollisionAvoidance(preferedCollisionAvoidanceAngle);

        stearing = Mathf.Clamp(stearingTowardsLookAheadAngle + collisionAvoidanceAngle * 1.5f, -1f, 1f);
    }

    private float GetCollisionAvoidance(float frontalStearingAngle)
    {
        float stearingAngle = 0;

        Vector3 frontalPosition = transform.position + transform.forward * frontalCollisionOffset;
        Vector3 rightSidePosition = transform.position + transform.right * sideCollisionOffset;
        Vector3 leftSidePosition = transform.position - transform.right * sideCollisionOffset;
        Vector3 rightFrontalPosition = frontalPosition + transform.right * sideCollisionOffset;
        Vector3 leftFrontalPosition = frontalPosition - transform.right * sideCollisionOffset;

        //Frontal
        if (IsCollision(frontalPosition, 0, frontalCollisionDetectionDistance * speed / maxSpeed))
        {
            Debug.DrawRay(frontalPosition, transform.forward * frontalCollisionDetectionDistance, Color.cyan);
            stearingAngle += frontalStearingAngle;
        }

        if (IsCollision(rightFrontalPosition, 0, frontalCollisionDetectionDistance * speed / maxSpeed))
        {
            Debug.DrawRay(rightFrontalPosition, transform.forward * frontalCollisionDetectionDistance, Color.cyan);
            stearingAngle += frontalStearingAngle;
        }
        if (IsCollision(leftFrontalPosition, 0, frontalCollisionDetectionDistance * speed / maxSpeed))
        {
            Debug.DrawRay(leftFrontalPosition, transform.forward * frontalCollisionDetectionDistance, Color.cyan);
            stearingAngle += frontalStearingAngle;
        }

        //Frontal side
        if (IsCollision(rightFrontalPosition, 25f, sideCollisionDetectionDistance))
        {
            Debug.DrawRay(rightFrontalPosition, RotateVector(transform.forward * sideCollisionDetectionDistance, 25f), Color.cyan);
            stearingAngle -= 0.75f;
        }
        if (IsCollision(leftFrontalPosition, -25f, sideCollisionDetectionDistance))
        {
            Debug.DrawRay(leftFrontalPosition, RotateVector(transform.forward * sideCollisionDetectionDistance, -25f), Color.cyan);
            stearingAngle += 0.75f;

        }

        //Side
        if (IsCollision(rightSidePosition, 55f, sideCollisionDetectionDistance))
        {
            Debug.DrawRay(rightSidePosition, RotateVector(transform.forward * sideCollisionDetectionDistance, 55f), Color.cyan);
            stearingAngle -= 1;
        }
        if (IsCollision(leftSidePosition, -55f, sideCollisionDetectionDistance))
        {
            Debug.DrawRay(leftSidePosition, RotateVector(transform.forward * sideCollisionDetectionDistance, -55f), Color.cyan);
            stearingAngle += 1;
        }

        return Mathf.Clamp(stearingAngle, -1f, 1f);
    }

    private bool IsCollision(Vector3 origin, float angleInDegrees, float distance)
    {
        return Physics.Raycast(origin, RotateVector(transform.forward, angleInDegrees), distance, collisionMask);
    }

    private Vector3 RotateVector(Vector3 vector, float angleInDegrees)
    {
        float radians = -DegreesToRadians(angleInDegrees);

        return new Vector3(
            vector.x * Mathf.Cos(radians) - vector.z * Mathf.Sin(radians),
            0,
            vector.x * Mathf.Sin(radians) + vector.z * Mathf.Cos(radians)
        );
    }

    private float DegreesToRadians(float degrees)
    {
        return degrees * Convert.ToSingle(Math.PI) / 180f;
    }
    #endregion
}
