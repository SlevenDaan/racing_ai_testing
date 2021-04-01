using System;
using UnityEngine;

public class Car_DynamicPathfindingPlus : Car
{
    private struct CollisionInfo
    {
        public float distanceToCollision;
        public float distanceOfRaycast;
    }

    [Header("-----Car_DynamicPathfindingPlus-----")]
    [SerializeField] private LayerMask wallCollisionMask;
    [SerializeField] private LayerMask carCollisionMask;
    [SerializeField] private float carCollisionDetectionDistance = 3f;
    [SerializeField] private float frontalCollisionDetectionDistance = 5f;
    [SerializeField] private float sideCollisionDetectionDistance = 3f;
    [Space]
    [SerializeField] private float frontalCollisionOffset = 0.8f;
    [SerializeField] private float sideCollisionOffset = 0.4f;
    [Space]
    [SerializeField] private float preferedCollisionAvoidanceAngle = 10f;


    protected override void OnDrawGizmos()
    {
        base.OnDrawGizmos();
    }

    protected override void Update()
    {
        CalculateStearing();
        CalculateThrottle();
        base.Update();
    }

    #region Throttle
    private void CalculateThrottle()
    {
        Vector3 frontalPosition = transform.position + transform.forward * frontalCollisionOffset;
        
        if (speed!=0 && GetCollision(frontalPosition, 0, speed, speed/maxSpeed * carCollisionDetectionDistance, out CollisionInfo collisionInfo, false))
        {
            throttle = -1f * (collisionInfo.distanceToCollision / collisionInfo.distanceOfRaycast);
        }
        else
        {
            throttle = 1f;
        }
    }
    #endregion

    #region Stearing
    private void CalculateStearing()
    {
        float stearingAngle = 0;
        Vector3 frontalPosition = transform.position + transform.forward * frontalCollisionOffset;
        Vector3 rightFrontalPosition = frontalPosition + transform.right * sideCollisionOffset;
        Vector3 leftFrontalPosition = frontalPosition - transform.right * sideCollisionOffset;

        //Frontal
        if (GetCollision(frontalPosition, 0, frontalCollisionDetectionDistance, carCollisionDetectionDistance, out CollisionInfo _, true))
        {
            stearingAngle += preferedCollisionAvoidanceAngle;
        }

        float angleDifferenceFrontAndSide = frontalCollisionDetectionDistance - sideCollisionDetectionDistance;
        for (float i = 0; i <= 9f; i++)
        {
            float multiplier = (9f - i) / 9f;
            float distance = sideCollisionDetectionDistance + (multiplier * angleDifferenceFrontAndSide);

            if (distance == 0) { return; }

            if (GetCollision(rightFrontalPosition, 10 * i, distance, carCollisionDetectionDistance, out CollisionInfo _, true))
            {
                stearingAngle -= 10;
            }
            if (GetCollision(leftFrontalPosition, -10 * i, distance, carCollisionDetectionDistance, out CollisionInfo _, true))
            {
                stearingAngle += 10;
            }
        }

        stearing = Mathf.Clamp(stearingAngle/maxStearingError, -1f, 1f);
    }

    private bool GetCollision(Vector3 origin, float angleInDegrees, float wallDistance, float carDistance, out CollisionInfo collisionInfo, bool drawDebug = true)
    {
        collisionInfo = new CollisionInfo();
        Vector3 angleVector = RotateVector(transform.forward, angleInDegrees);
        bool collidedWithWall = Physics.Raycast(origin, angleVector, out RaycastHit wallRaycastHit, wallDistance, wallCollisionMask);
        bool collidedWithCar = Physics.Raycast(origin, angleVector, out RaycastHit carRaycastHit, carDistance, carCollisionMask);

        if (drawDebug)
        {
            if (collidedWithWall) { Debug.DrawRay(origin, angleVector.normalized * wallDistance, Color.cyan); }
            if (collidedWithCar) { Debug.DrawRay(origin, angleVector.normalized * carDistance, Color.magenta); }
        }

        if(collidedWithCar && collidedWithWall)
        {
            if (wallRaycastHit.distance > carRaycastHit.distance)
            {
                collisionInfo.distanceToCollision = wallRaycastHit.distance;
                collisionInfo.distanceOfRaycast = wallDistance;
            }
            else
            {
                collisionInfo.distanceToCollision = carRaycastHit.distance;
                collisionInfo.distanceOfRaycast = carDistance;
            }
        }
        else if (collidedWithWall)
        {
            collisionInfo.distanceToCollision = wallRaycastHit.distance;
            collisionInfo.distanceOfRaycast = wallDistance;
        }
        else if (collidedWithCar)
        {
            collisionInfo.distanceToCollision = carRaycastHit.distance;
            collisionInfo.distanceOfRaycast = carDistance;
        }

        return collidedWithWall || collidedWithCar;
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
