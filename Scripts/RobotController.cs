using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class RobotController : MonoBehaviour
{
    [Header("Start")]
    public Vector3 startingPosition = new Vector3(0f, 0.5f, 0f);
    public float startingRotation = 0f;
    [FormerlySerializedAs("carVisualObject")]
    public Transform visualModel;
    [FormerlySerializedAs("carVisualOffset")]
    public Vector3 visualOffset = new Vector3(0f, 0.5f, 0f);

    [Header("Movement")]
    public float baseForwardSpeed = 600f;
    public float maxVelocity = 8f;
    public float maxCurveVelocity = 4f;
    public float minimumSpeed = 1.0f;
    [Range(0.5f, 3f)]
    public float brakingAggressiveness = 1.5f;

    [Header("Steering Controller")]
    public float Kp = 1.2f;
    public float Kd = 0.35f;
    public bool usePDControl = true;
    public float Kh = 1.4f;
    public float maxTurnRate = 2.5f;

    [Header("Integral Control")]
    [Tooltip("Integral gain.")]
    public float Ki = 0.08f;

    [Tooltip("Clamp for the integral accumulator.")]
    public float integralClamp = 0.45f;

    [Tooltip("Integral leak per second.")]
    [Range(0f, 2f)]
    public float integralLeakPerSecond = 0.35f;

    [Tooltip("Only integrate when the lane centre is available.")]
    public bool integrateOnlyWhenHasCenter = true;

    [Tooltip("Only integrate when both lane edges are visible.")]
    public bool integrateOnlyWhenBothEdges = true;

    [Tooltip("Disable the integral term in curves.")]
    public bool disableIntegralInCurves = true;

    [Tooltip("Disable the integral term during avoidance.")]
    public bool disableIntegralDuringAvoidance = true;

    [Tooltip("Freeze integral accumulation when steering is saturated.")]
    public bool freezeIntegralWhenSaturated = true;

    [Tooltip("Dead-zone scale used by the integral term.")]
    [Range(0f, 1f)]
    public float integralDeadZoneFactor = 0.5f;

    [Tooltip("Smoothing time for the integral output.")]
    [Range(0.0f, 0.5f)]
    public float integralSmoothingTime = 0.08f;

    [Header("Stability")]
    [Range(0f, 0.15f)]
    public float deadZone = 0.03f;
    [Range(0f, 0.99f)]
    public float derivativeSmoothing = 0.85f;
    [Range(0.01f, 0.5f)]
    public float errorSmoothingTime = 0.12f;
    [Range(0.01f, 0.5f)]
    public float headingSmoothingTime = 0.10f;

    [Header("Lookahead")]
    public float lookaheadMin = 1.5f;
    public float lookaheadMax = 5.0f;
    [Range(0.5f, 2f)]
    public float lookaheadSpeedFactor = 1.2f;
    public float predictionDistance = 6.0f;
    public float maxHeadingRadians = 0.8f;
    [Range(0.01f, 0.3f)]
    public float aimPointSmoothingTime = 0.08f;

    [Header("Feedforward")]
    public float Kff = 0.7f;
    [Range(0.01f, 0.3f)]
    public float feedforwardSmoothingTime = 0.10f;

    [Header("Rate Limits")]
    public float maxTurnRateChangePerSecond = 8.0f;
    public float maxAngularAcceleration = 10.0f;

    [Header("Obstacle Avoidance")]
    public LayerMask obstacleLayer;
    public float obstacleDetectDistance = 16.0f;
    public float obstacleAvoidStartDistance = 10.0f;
    public float obstacleStopDistance = 2.5f;
    public float obstacleSphereRadius = 0.35f;
    public float obstacleSensorForwardOffset = 1.4f;
    public float obstacleSensorHeight = 0.75f;

    [Header("Obstacle Fan Sensors")]
    public int obstacleFanRayCount = 9;
    [Range(5f, 60f)]
    public float obstacleFanHalfAngle = 25f;
    [Tooltip("Lateral spacing between the three sensor columns.")]
    public float obstacleFanLateralSpread = 0.5f;

    [Header("Avoidance Stability")]
    [Tooltip("Scale Kp during avoidance.")]
    [Range(0.1f, 1f)]
    public float avoidKpScale = 0.5f;

    [Tooltip("Scale Kh during avoidance.")]
    [Range(0.1f, 1f)]
    public float avoidKhScale = 0.6f;

    [Tooltip("Boost Kd during avoidance.")]
    [Range(1f, 3f)]
    public float avoidKdBoost = 1.8f;

    [Tooltip("Turn-rate limit during avoidance.")]
    [Range(0.5f, 3f)]
    public float avoidMaxTurnRate = 1.8f;

    [Tooltip("Rate-of-change limit during avoidance.")]
    public float avoidTurnRateChangePerSecond = 5.0f;
    public float maxDodgeLeftOffset = 1.0f;
    public float laneEdgeSafetyMargin = 0.45f;
    public float dodgeSmoothTime = 0.50f;
    public float mergeSmoothTime = 0.65f;
    public float obstacleDodgeSpeed = 3.0f;
    public float obstacleMaxBrakeAccel = 18f;
    public float obstacleBrakeGain = 8f;
    public float obstacleReactionTime = 0.25f;
    public bool checkLeftPath = true;
    public float leftPathCheckExtra = 1.0f;

    [Header("Side Sensor")]
    public float sideSensorRange = 3.0f;
    public float sideSensorRadius = 0.3f;
    public float sideSensorHeight = 0.6f;
    public int sideSensorCount = 3;
    public float sideSensorLengthSpread = 1.5f;
    public float sideClearConfirmTime = 0.3f;

    [Header("Lane Sensors")]
    public float sensorForwardDistance = 1.2f;
    public float sensorHeightOffset = 0.5f;
    public float raycastDownDistance = 3.0f;
    public float expectedLaneWidth = 4.0f;

    [Header("Lane Sensor Array")]
    public int sensorsPerSide = 5;
    public float sensorSpread = 2.5f;

    [Header("Lane Layers")]
    public LayerMask leftLaneLayer;
    public LayerMask rightLaneLayer;

    [Header("Lane Robustness")]
    [Range(0f, 1f)]
    public float widthBlendToMeasured = 0.6f;

    [Header("Curve Detection")]
    [Range(0f, 1f)]
    public float curveEnterThreshold = 0.10f;
    [Range(0f, 1f)]
    public float curveExitThreshold = 0.06f;
    [Range(0.1f, 1f)]
    public float curveExitHoldTime = 0.4f;

    [Header("Recovery")]
    public float recoveryTimeout = 5.0f;
    public float recoverySteeringMultiplier = 1.0f;
    public bool keepMovingDuringRecovery = true;

    [Header("Debug")]
    public bool showDebugRays = true;
    public bool showLookaheadRays = true;
    public bool showPredictionRays = false;
    public bool showObstacleRays = true;
    public bool showDebugLog = false;

    private Rigidbody rb;

    private float currentLateralError = 0f;
    private float previousLateralError = 0f;
    private float smoothedError = 0f;
    private float smoothedDerivative = 0f;

    private float currentHeadingError = 0f;
    private float smoothedHeadingError = 0f;

    private float appliedTurnRate = 0f;
    private float rateLimitedTurnRate = 0f;
    private float dynamicLookahead = 2.5f;

    private Vector3 smoothedAimLocal = new Vector3(0f, 0f, 2f);
    private float smoothedYawRateFF = 0f;

    private float targetVelocity;
    private float smoothedTargetVelocity;

    private bool leftLineDetected = false;
    private bool rightLineDetected = false;
    private float leftLineDistance = 0f;
    private float rightLineDistance = 0f;

    private float lastKnownError = 0f;
    private float timeSinceLineDetected = 0f;
    private bool isRecovering = false;

    private bool isInCurve = false;
    private float curveExitCounter = 0f;
    private float predictedCurvature = 0f;

    private float avoidOffsetX = 0f;
    private float avoidOffsetVelocity = 0f;
    private float obstacleSpeedCap = float.PositiveInfinity;
    private bool obstacleMustStop = false;
    private float lastObstacleDistance = 999f;

    private enum AvoidState { None, Dodging, Passing, Merging }
    private AvoidState avoidState = AvoidState.None;
    private AvoidState previousAvoidState = AvoidState.None;
    private float sideClearTimer = 0f;
    private bool suppressRightLaneSensors = false;

    private LaneSample nearSample;
    private LaneSample farSample;
    private LaneSample predictionSample;

    private float integralError = 0f;
    private float integralErrorSmoothed = 0f;
    private float integralVelocity = 0f;

    [HideInInspector] public float debugPTerm;
    [HideInInspector] public float debugITerm;
    [HideInInspector] public float debugDTerm;
    [HideInInspector] public float debugHTerm;
    [HideInInspector] public float debugFFTerm;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("[RobotController] Rigidbody missing!");
            return;
        }

        // Keep the robot upright so the tests focus on lane-following behaviour rather than body roll.
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        transform.position = startingPosition;
        transform.rotation = Quaternion.Euler(0f, startingRotation, 0f);
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        smoothedAimLocal = new Vector3(0f, 0f, lookaheadMin);
        targetVelocity = maxVelocity;
        smoothedTargetVelocity = maxVelocity;

        integralError = 0f;
        integralErrorSmoothed = 0f;
        integralVelocity = 0f;

        Debug.Log("[RobotController v10] Started - PID lateral (I-term) + anti-windup + gated integral");
    }

    void FixedUpdate()
    {
        if (rb == null) return;

        UpdateVisualModel();

        float speed = rb.linearVelocity.magnitude;
        float speed01 = Mathf.Clamp01(speed / Mathf.Max(0.01f, maxVelocity));

        // Increase lookahead with speed, then shorten it again in curves so the robot reacts earlier.
        float baseLookahead = Mathf.Lerp(lookaheadMin, lookaheadMax, speed01 * lookaheadSpeedFactor);
        if (isInCurve) baseLookahead *= 0.7f;
        dynamicLookahead = Mathf.Max(lookaheadMin, baseLookahead);

        // Three lane samples are used: near for centring, far for heading, and prediction for future curvature.
        nearSample = SampleLane(sensorForwardDistance, showDebugRays, 0);
        farSample = SampleLane(dynamicLookahead, showLookaheadRays, 1);
        predictionSample = SampleLane(predictionDistance, showPredictionRays, 2);

        UpdateDetectionFlags();

        previousAvoidState = avoidState;
        UpdateObstacleAvoidance(speed);

        PredictUpcomingCurvature();
        CalculateErrors();
        CalculateTargetSpeed();
        ApplyControl();
    }

    private void UpdateVisualModel()
    {
        if (visualModel != null)
            visualModel.localPosition = visualOffset;
    }

    // Each sample stores what the lane sensors saw at one distance ahead of the robot.
    private struct LaneSample
    {
        public bool leftDetected;
        public bool rightDetected;
        public bool hasCenter;
        public float leftOffset;
        public float rightOffset;
        public float halfWidthUsed;
        public Vector3 centerWorld;
        public Vector3 centerLocal;
    }

    private LaneSample SampleLane(float forwardDist, bool drawRays, int rayTint)
    {
        LaneSample sample = new LaneSample
        {
            leftDetected = false,
            rightDetected = false,
            hasCenter = false,
            leftOffset = 0f,
            rightOffset = 0f,
            halfWidthUsed = Mathf.Max(0.001f, expectedLaneWidth * 0.5f),
            centerWorld = transform.position + transform.forward * forwardDist,
            centerLocal = new Vector3(0f, 0f, forwardDist)
        };

        Vector3 sensorOrigin = transform.position + transform.forward * forwardDist;
        sensorOrigin.y = transform.position.y + sensorHeightOffset;

        Color[] tintColors = { Color.green, Color.cyan, Color.magenta };
        Color[] missColorsL = { Color.white, new Color(1f, 1f, 1f, 0.4f), new Color(1f, 0.5f, 1f, 0.3f) };
        Color[] missColorsR = { Color.yellow, new Color(1f, 1f, 0f, 0.4f), new Color(1f, 1f, 0.5f, 0.3f) };

        // Multiple rays per side make the estimate more robust when one or two sensors miss the lane line.
        List<float> leftOffsets = new List<float>(sensorsPerSide);
        List<Vector3> leftPoints = new List<Vector3>(sensorsPerSide);
        List<float> rightOffsets = new List<float>(sensorsPerSide);
        List<Vector3> rightPoints = new List<Vector3>(sensorsPerSide);

        for (int i = 0; i < sensorsPerSide; i++)
        {
            float t = (sensorsPerSide > 1) ? (float)i / (sensorsPerSide - 1) : 0f;
            float offset = Mathf.Lerp(0.3f, sensorSpread, t);

            Vector3 leftPos = sensorOrigin - transform.right * offset;
            if (Physics.Raycast(leftPos, Vector3.down, out RaycastHit leftHit, raycastDownDistance, leftLaneLayer, QueryTriggerInteraction.Ignore))
            {
                sample.leftDetected = true;
                leftOffsets.Add(offset);
                leftPoints.Add(leftHit.point);
                if (drawRays) Debug.DrawLine(leftPos, leftHit.point, tintColors[rayTint]);
            }
            else if (drawRays)
            {
                Debug.DrawRay(leftPos, Vector3.down * raycastDownDistance, missColorsL[rayTint]);
            }

            if (!suppressRightLaneSensors)
            {
                Vector3 rightPos = sensorOrigin + transform.right * offset;
                if (Physics.Raycast(rightPos, Vector3.down, out RaycastHit rightHit, raycastDownDistance, rightLaneLayer, QueryTriggerInteraction.Ignore))
                {
                    sample.rightDetected = true;
                    rightOffsets.Add(offset);
                    rightPoints.Add(rightHit.point);
                    if (drawRays) Debug.DrawLine(rightPos, rightHit.point, tintColors[rayTint]);
                }
                else if (drawRays)
                {
                    Debug.DrawRay(rightPos, Vector3.down * raycastDownDistance, missColorsR[rayTint]);
                }
            }
        }

        Vector3 leftPoint = Vector3.zero;
        Vector3 rightPoint = Vector3.zero;
        bool hasLeft = sample.leftDetected && leftOffsets.Count > 0;
        bool hasRight = sample.rightDetected && rightOffsets.Count > 0;

        if (hasLeft)
        {
            // Median selection rejects single outliers better than taking the widest or average hit directly.
            int idx = GetMedianIndex(leftOffsets);
            sample.leftOffset = leftOffsets[idx];
            leftPoint = leftPoints[idx];
        }

        if (hasRight)
        {
            int idx = GetMedianIndex(rightOffsets);
            sample.rightOffset = rightOffsets[idx];
            rightPoint = rightPoints[idx];
        }

        // If only one edge is visible, rebuild the lane centre from the expected width instead of dropping it.
        if (hasLeft && hasRight)
        {
            sample.centerWorld = (leftPoint + rightPoint) * 0.5f;
            float measuredHalf = (sample.leftOffset + sample.rightOffset) * 0.5f;
            float expectedHalf = expectedLaneWidth * 0.5f;
            sample.halfWidthUsed = Mathf.Lerp(expectedHalf, measuredHalf, widthBlendToMeasured);
            sample.hasCenter = true;
        }
        else if (hasLeft)
        {
            sample.centerWorld = leftPoint + transform.right * (expectedLaneWidth * 0.5f);
            sample.halfWidthUsed = expectedLaneWidth * 0.5f;
            sample.hasCenter = true;
        }
        else if (hasRight)
        {
            sample.centerWorld = rightPoint - transform.right * (expectedLaneWidth * 0.5f);
            sample.halfWidthUsed = expectedLaneWidth * 0.5f;
            sample.hasCenter = true;
        }

        if (sample.hasCenter)
            sample.centerLocal = transform.InverseTransformPoint(sample.centerWorld);

        return sample;
    }

    private int GetMedianIndex(List<float> values)
    {
        if (values.Count <= 1) return 0;

        List<int> indices = new List<int>(values.Count);
        for (int i = 0; i < values.Count; i++) indices.Add(i);
        indices.Sort((a, b) => values[a].CompareTo(values[b]));
        return indices[indices.Count / 2];
    }

    private void UpdateDetectionFlags()
    {
        leftLineDetected = nearSample.leftDetected;
        rightLineDetected = nearSample.rightDetected;
        leftLineDistance = nearSample.leftOffset;
        rightLineDistance = nearSample.rightOffset;
    }

    private void UpdateObstacleAvoidance(float speed)
    {
        float dt = Time.fixedDeltaTime;
        obstacleMustStop = false;
        obstacleSpeedCap = float.PositiveInfinity;

        Vector3 originBase = transform.position + Vector3.up * obstacleSensorHeight + transform.forward * obstacleSensorForwardOffset;

        float brakeAccel = Mathf.Max(0.1f, obstacleMaxBrakeAccel);
        float reactionDist = speed * Mathf.Max(0.0f, obstacleReactionTime);
        float stopDist = (speed * speed) / (2f * brakeAccel);
        float shiftDist = speed * Mathf.Max(0.10f, dodgeSmoothTime * 2.0f);
        // Start avoidance earlier at higher speed so there is enough distance for braking and lateral motion.
        float dynamicStartDist = Mathf.Max(obstacleAvoidStartDistance, reactionDist + stopDist + 0.7f, shiftDist + 0.8f);
        float detectDist = Mathf.Max(dynamicStartDist + 2f, obstacleDetectDistance);

        bool forwardHit = false;
        float forwardDist = float.PositiveInfinity;

        int raysPerColumn = Mathf.Max(1, obstacleFanRayCount / 3);
        float[] lateralOffsets = { -obstacleFanLateralSpread, 0f, +obstacleFanLateralSpread };

        for (int col = 0; col < 3; col++)
        {
            Vector3 colOrigin = originBase + transform.right * lateralOffsets[col];

            for (int r = 0; r < raysPerColumn; r++)
            {
                float t = (raysPerColumn > 1) ? (float)r / (raysPerColumn - 1) : 0.5f;
                float angle = Mathf.Lerp(-obstacleFanHalfAngle, +obstacleFanHalfAngle, t);
                Vector3 rayDir = Quaternion.AngleAxis(angle, Vector3.up) * transform.forward;

                bool hit = Physics.SphereCast(colOrigin, obstacleSphereRadius, rayDir, out RaycastHit hitInfo, detectDist, obstacleLayer, QueryTriggerInteraction.Ignore);

                if (showObstacleRays)
                {
                    Color rayColor = hit ? Color.red : new Color(1f, 0.2f, 0.2f, 0.15f);
                    Debug.DrawRay(colOrigin, rayDir * detectDist, rayColor);
                    if (hit) Debug.DrawLine(colOrigin, hitInfo.point, Color.red);
                }

                if (hit)
                {
                    // Use forward distance rather than diagonal ray distance.
                    float angleRad = angle * Mathf.Deg2Rad;
                    float projectedDist = hitInfo.distance * Mathf.Cos(angleRad);

                    if (projectedDist < forwardDist)
                    {
                        forwardHit = true;
                        forwardDist = projectedDist;
                    }
                }
            }
        }

        if (forwardHit)
            lastObstacleDistance = forwardDist;

        bool sideSeesObstacle = false;
        if (avoidState == AvoidState.Dodging || avoidState == AvoidState.Passing)
            sideSeesObstacle = HasSideObstacle();

        float targetOffset = 0f;

        // Main avoidance sequence: start dodge, hold the offset while passing, then merge back when clear.
        switch (avoidState)
        {
            case AvoidState.None:
                suppressRightLaneSensors = false;
                if (forwardHit && forwardDist <= dynamicStartDist)
                {
                    avoidState = AvoidState.Dodging;
                    suppressRightLaneSensors = true;
                    sideClearTimer = 0f;
                }
                break;

            case AvoidState.Dodging:
                // Ignore the right lane during the dodge so the controller does not merge back too early.
                suppressRightLaneSensors = true;

                if (forwardHit)
                {
                    float d = forwardDist;

                    if (d <= obstacleStopDistance)
                    {
                        if (checkLeftPath)
                        {
                            // Final left-path check prevents committing to the dodge if the escape path is blocked.
                            Vector3 leftPathOrigin = originBase + transform.right * (-maxDodgeLeftOffset);
                            float leftCheckDist = Mathf.Min(detectDist, d + leftPathCheckExtra);
                            bool leftBlocked = Physics.SphereCast(leftPathOrigin, obstacleSphereRadius, transform.forward, out RaycastHit leftHit, leftCheckDist, obstacleLayer, QueryTriggerInteraction.Ignore);

                            if (showObstacleRays)
                            {
                                Color lc = leftBlocked ? new Color(1f, 0.5f, 0f, 1f) : new Color(0.2f, 1f, 0.4f, 0.8f);
                                Debug.DrawRay(leftPathOrigin, transform.forward * leftCheckDist, lc);
                            }

                            if (leftBlocked)
                            {
                                obstacleMustStop = true;
                                obstacleSpeedCap = 0f;
                                break;
                            }
                        }
                    }

                    float t = Mathf.InverseLerp(dynamicStartDist, obstacleStopDistance, d);
                    t = Mathf.Clamp01(t);
                    targetOffset = -Mathf.Lerp(0f, maxDodgeLeftOffset, t);

                    float halfWidth = nearSample.hasCenter ? nearSample.halfWidthUsed : (expectedLaneWidth * 0.5f);
                    float maxOffsetInsideLane = Mathf.Max(0.1f, halfWidth - laneEdgeSafetyMargin);
                    targetOffset = Mathf.Clamp(targetOffset, -maxOffsetInsideLane, +maxOffsetInsideLane);

                    if (!obstacleMustStop)
                    {
                        float distToStopMargin = Mathf.Max(0f, d - obstacleStopDistance);
                        float vAllow = Mathf.Sqrt(2f * brakeAccel * distToStopMargin);
                        float capWanted = Mathf.Lerp(maxVelocity, obstacleDodgeSpeed, t * t);
                        obstacleSpeedCap = Mathf.Min(capWanted, vAllow);
                    }
                }
                else
                {
                    targetOffset = -maxDodgeLeftOffset;
                    float halfWidth = nearSample.hasCenter ? nearSample.halfWidthUsed : (expectedLaneWidth * 0.5f);
                    float maxOffsetInsideLane = Mathf.Max(0.1f, halfWidth - laneEdgeSafetyMargin);
                    targetOffset = Mathf.Clamp(targetOffset, -maxOffsetInsideLane, +maxOffsetInsideLane);
                    obstacleSpeedCap = obstacleDodgeSpeed;
                }

                if (sideSeesObstacle)
                {
                    avoidState = AvoidState.Passing;
                    sideClearTimer = 0f;
                }
                else if (!forwardHit)
                {
                    sideClearTimer += dt;
                    if (sideClearTimer > 0.5f)
                        avoidState = AvoidState.Merging;
                }
                else
                {
                    sideClearTimer = 0f;
                }
                break;

            case AvoidState.Passing:
                suppressRightLaneSensors = true;

                targetOffset = -maxDodgeLeftOffset;
                float passHalfWidth = nearSample.hasCenter ? nearSample.halfWidthUsed : (expectedLaneWidth * 0.5f);
                float passMaxOffsetInsideLane = Mathf.Max(0.1f, passHalfWidth - laneEdgeSafetyMargin);
                targetOffset = Mathf.Clamp(targetOffset, -passMaxOffsetInsideLane, +passMaxOffsetInsideLane);
                obstacleSpeedCap = obstacleDodgeSpeed;

                if (!sideSeesObstacle)
                {
                    sideClearTimer += dt;
                    if (sideClearTimer >= sideClearConfirmTime)
                        avoidState = AvoidState.Merging;
                }
                else
                {
                    sideClearTimer = 0f;
                }

                if (forwardHit && forwardDist < obstacleStopDistance)
                {
                    obstacleMustStop = true;
                    obstacleSpeedCap = 0f;
                }
                break;

            case AvoidState.Merging:
                targetOffset = 0f;

                if (Mathf.Abs(avoidOffsetX) < 0.15f)
                {
                    suppressRightLaneSensors = false;
                    avoidState = AvoidState.None;
                }
                else
                {
                    suppressRightLaneSensors = true;
                }

                if (forwardHit && forwardDist <= dynamicStartDist)
                {
                    avoidState = AvoidState.Dodging;
                    suppressRightLaneSensors = true;
                    sideClearTimer = 0f;
                }
                break;
        }

        float currentSmoothTime = (avoidState == AvoidState.Merging) ? mergeSmoothTime : dodgeSmoothTime;
        avoidOffsetX = Mathf.SmoothDamp(avoidOffsetX, targetOffset, ref avoidOffsetVelocity, currentSmoothTime);
    }

    private bool HasSideObstacle()
    {
        bool hitAny = false;

        // Side checks confirm when the obstacle is alongside the robot rather than only in front of it.
        for (int i = 0; i < sideSensorCount; i++)
        {
            float t = (sideSensorCount > 1) ? (float)i / (sideSensorCount - 1) : 0.5f;
            float forwardOffset = Mathf.Lerp(-sideSensorLengthSpread, sideSensorLengthSpread, t);

            Vector3 origin = transform.position + Vector3.up * sideSensorHeight + transform.forward * forwardOffset;
            Vector3 direction = transform.right;

            bool hit = Physics.SphereCast(origin, sideSensorRadius, direction, out RaycastHit hitInfo, sideSensorRange, obstacleLayer, QueryTriggerInteraction.Ignore);

            if (showObstacleRays)
            {
                Color c = hit ? new Color(1f, 0.6f, 0f, 1f) : new Color(0.3f, 0.8f, 1f, 0.5f);
                Debug.DrawRay(origin, direction * sideSensorRange, c);
                if (hit) Debug.DrawLine(origin, hitInfo.point, Color.yellow);
            }

            if (hit) hitAny = true;
        }

        return hitAny;
    }

    private void PredictUpcomingCurvature()
    {
        if (nearSample.hasCenter && farSample.hasCenter)
        {
            Vector3 nearLocal = nearSample.centerLocal;
            Vector3 farLocal = farSample.centerLocal;

            // Compare how the lane centre moves between near and far samples to estimate curvature.
            float lateralDeviation = farLocal.x - nearLocal.x;
            float longitudinalDist = Mathf.Max(0.1f, farLocal.z - nearLocal.z);

            float curvatureEstimate = lateralDeviation / (longitudinalDist * longitudinalDist);

            if (predictionSample.hasCenter)
            {
                // A third sample further ahead helps stabilise the sign and strength on S-curves.
                Vector3 predLocal = predictionSample.centerLocal;
                float predLateral = predLocal.x - farLocal.x;
                float predLongitudinal = Mathf.Max(0.1f, predLocal.z - farLocal.z);
                float predCurvature = predLateral / (predLongitudinal * predLongitudinal);

                curvatureEstimate = Mathf.Max(Mathf.Abs(curvatureEstimate), Mathf.Abs(predCurvature)) * Mathf.Sign(curvatureEstimate + predCurvature);
            }

            predictedCurvature = Mathf.Lerp(predictedCurvature, curvatureEstimate, ExpAlpha(0.15f));
        }

        float absError = Mathf.Abs(smoothedError);
        float absHeading = Mathf.Abs(smoothedHeadingError);
        float absCurvature = Mathf.Abs(predictedCurvature);

        if (!isInCurve)
        {
            if (absError > curveEnterThreshold ||
                absHeading > maxHeadingRadians * 0.3f ||
                absCurvature > 0.15f)
            {
                isInCurve = true;
                curveExitCounter = 0f;
            }
        }
        else
        {
            // Hysteresis prevents fast switching at the threshold.
            if (absError < curveExitThreshold &&
                absHeading < maxHeadingRadians * 0.15f &&
                absCurvature < 0.08f)
            {
                curveExitCounter += Time.fixedDeltaTime;
                if (curveExitCounter >= curveExitHoldTime)
                {
                    isInCurve = false;
                    curveExitCounter = 0f;
                }
            }
            else
            {
                curveExitCounter = 0f;
            }
        }
    }

    private void CalculateErrors()
    {
        previousLateralError = currentLateralError;

        if (nearSample.hasCenter)
        {
            isRecovering = false;

            // Normalise by half-lane width so controller thresholds stay consistent across width estimates.
            float rawLateral = (nearSample.centerLocal.x + avoidOffsetX) / Mathf.Max(0.001f, nearSample.halfWidthUsed);
            currentLateralError = Mathf.Clamp(rawLateral, -1f, 1f);

            Vector3 aimWorld = farSample.hasCenter ? farSample.centerWorld : nearSample.centerWorld;
            Vector3 aimLocal = transform.InverseTransformPoint(aimWorld);
            aimLocal.x += avoidOffsetX;

            smoothedAimLocal = Vector3.Lerp(smoothedAimLocal, aimLocal, ExpAlpha(aimPointSmoothingTime));

            float rawHeading = Mathf.Atan2(smoothedAimLocal.x, Mathf.Max(0.001f, smoothedAimLocal.z));
            currentHeadingError = Mathf.Clamp(rawHeading, -maxHeadingRadians, maxHeadingRadians);

            timeSinceLineDetected = 0f;
            lastKnownError = currentLateralError;
        }
        else
        {
            // Recovery keeps steering from the last reliable estimate instead of snapping to zero immediately.
            isRecovering = true;
            timeSinceLineDetected += Time.fixedDeltaTime;

            if (timeSinceLineDetected < recoveryTimeout)
            {
                currentLateralError = Mathf.Clamp(lastKnownError * recoverySteeringMultiplier, -1f, 1f);
                currentHeadingError = 0f;
                isInCurve = true;
                curveExitCounter = 0f;
            }
            else if (!keepMovingDuringRecovery)
            {
                currentLateralError = 0f;
                currentHeadingError = 0f;
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                Debug.LogError("[TIMEOUT] Robot stopped");
            }
        }

        smoothedError = Mathf.Lerp(smoothedError, currentLateralError, ExpAlpha(errorSmoothingTime));
        smoothedHeadingError = Mathf.Lerp(smoothedHeadingError, currentHeadingError, ExpAlpha(headingSmoothingTime));

        float rawDerivative = (currentLateralError - previousLateralError) / Time.fixedDeltaTime;
        rawDerivative = Mathf.Clamp(rawDerivative, -10f, 10f);
        smoothedDerivative = Mathf.Lerp(smoothedDerivative, rawDerivative, 1f - derivativeSmoothing);
    }

    private void CalculateTargetSpeed()
    {
        float target = maxVelocity;

        float curvatureFactor = Mathf.Abs(predictedCurvature) * brakingAggressiveness;
        float headingFactor = Mathf.Abs(smoothedHeadingError) / maxHeadingRadians;
        float errorFactor = Mathf.Abs(smoothedError);

        // Slow down from the worst of curvature, heading and lateral error to preserve steering authority.
        float demand = Mathf.Clamp01(Mathf.Max(curvatureFactor, Mathf.Max(headingFactor * 0.8f, errorFactor * 0.5f)));
        float speedReduction = demand * demand;

        target = Mathf.Lerp(maxVelocity, maxCurveVelocity, speedReduction);

        if (obstacleMustStop) target = 0f;
        else if (!float.IsPositiveInfinity(obstacleSpeedCap)) target = Mathf.Min(target, obstacleSpeedCap);

        if (!obstacleMustStop)
            target = Mathf.Max(target, minimumSpeed);

        targetVelocity = target;

        float smoothTime = isInCurve ? 0.2f : 0.4f;
        smoothedTargetVelocity = Mathf.Lerp(smoothedTargetVelocity, targetVelocity, ExpAlpha(smoothTime));
    }

    private bool IntegralAllowed(float eUsed)
    {
        // The I-term is only trusted when the lane estimate is stable; otherwise it can wind up on bad data.
        if (Ki <= 0.00001f) return false;
        if (isRecovering) return false;

        if (integrateOnlyWhenHasCenter && !nearSample.hasCenter) return false;
        if (integrateOnlyWhenBothEdges && !(leftLineDetected && rightLineDetected)) return false;

        if (disableIntegralInCurves && isInCurve) return false;
        if (disableIntegralDuringAvoidance && (avoidState != AvoidState.None)) return false;

        float dz = Mathf.Max(0f, deadZone) * Mathf.Clamp01(integralDeadZoneFactor);
        if (Mathf.Abs(eUsed) < dz) return false;

        return true;
    }

    private void ResetIntegral()
    {
        integralError = 0f;
        integralErrorSmoothed = 0f;
        integralVelocity = 0f;
    }

    private void ApplyControl()
    {
        if (rb == null) return;

        float dt = Time.fixedDeltaTime;

        Vector3 vel = rb.linearVelocity;
        float speed = vel.magnitude;
        float forwardSpeed = Vector3.Dot(vel, transform.forward);

        float e = smoothedError;
        float h = smoothedHeadingError;

        if (Mathf.Abs(e) < deadZone && !isInCurve && !isRecovering)
            e = 0f;

        if (avoidState != previousAvoidState) ResetIntegral();
        if (!nearSample.hasCenter) ResetIntegral();
        if (isRecovering) ResetIntegral();

        // During avoidance the controller uses softer gains and tighter limits to keep the lane change stable.
        bool isAvoiding = (avoidState != AvoidState.None);
        float effectiveKp = isAvoiding ? (Kp * avoidKpScale) : Kp;
        float effectiveKh = isAvoiding ? (Kh * avoidKhScale) : Kh;
        float effectiveKdBoost = isAvoiding ? avoidKdBoost : 1f;
        float effectiveMaxTurn = isAvoiding ? avoidMaxTurnRate : maxTurnRate;
        float effectiveTurnRateChange = isAvoiding ? avoidTurnRateChangePerSecond : maxTurnRateChangePerSecond;

        // Final steering combines lateral correction, damping, heading alignment and pure-pursuit feedforward.
        float pTerm = effectiveKp * e;

        float dTerm = 0f;
        if (usePDControl && Mathf.Abs(e) >= deadZone)
        {
            float speedAdaptiveKd = Kd * effectiveKdBoost * (1f + 0.3f * (speed / Mathf.Max(0.01f, maxVelocity)));
            dTerm = speedAdaptiveKd * smoothedDerivative;
        }

        float hTerm = effectiveKh * h;

        float yawRateFF = 0f;
        if (Kff > 0.0001f && smoothedAimLocal.z > 0.05f)
        {
            yawRateFF = ComputePurePursuitYawRate(smoothedAimLocal, speed);
            yawRateFF = Mathf.Clamp(yawRateFF, -maxTurnRate, maxTurnRate);
        }
        smoothedYawRateFF = Mathf.Lerp(smoothedYawRateFF, yawRateFF, ExpAlpha(feedforwardSmoothingTime));
        float ffTerm = Kff * smoothedYawRateFF;

        bool allowed = IntegralAllowed(e);

        // The integral term acts more like a small straight-line bias corrector than a dominant steering term.
        if (integralLeakPerSecond > 0f)
        {
            float leak = integralLeakPerSecond * dt;
            integralError = Mathf.MoveTowards(integralError, 0f, leak);
        }

        float integralCandidate = integralError;
        if (allowed)
        {
            integralCandidate = integralError + (e * dt);
            integralCandidate = Mathf.Clamp(integralCandidate, -integralClamp, integralClamp);
        }

        if (integralSmoothingTime > 0.0001f)
        {
            integralErrorSmoothed = Mathf.SmoothDamp(integralErrorSmoothed, integralCandidate, ref integralVelocity, integralSmoothingTime);
        }
        else
        {
            integralErrorSmoothed = integralCandidate;
        }

        float iTerm = Ki * integralErrorSmoothed;

        float rawTurnRate = pTerm + iTerm + dTerm + hTerm + ffTerm;
        debugPTerm = pTerm;
        debugITerm = iTerm;
        debugDTerm = dTerm;
        debugHTerm = hTerm;
        debugFFTerm = ffTerm;
        float targetTurnRate = Mathf.Clamp(rawTurnRate, -effectiveMaxTurn, effectiveMaxTurn);

        if (freezeIntegralWhenSaturated && allowed)
        {
            // Anti-windup: freeze accumulation when the output is already clamped and still pushing the same way.
            bool saturated = Mathf.Abs(rawTurnRate) > effectiveMaxTurn * 0.995f;
            bool pushingSameWay = Mathf.Sign(rawTurnRate) == Mathf.Sign(e);

            if (saturated && pushingSameWay)
            {
                integralCandidate = integralError;

                if (integralSmoothingTime > 0.0001f)
                    integralErrorSmoothed = Mathf.SmoothDamp(integralErrorSmoothed, integralCandidate, ref integralVelocity, integralSmoothingTime);
                else
                    integralErrorSmoothed = integralCandidate;

                iTerm = Ki * integralErrorSmoothed;

                rawTurnRate = pTerm + iTerm + dTerm + hTerm + ffTerm;
                targetTurnRate = Mathf.Clamp(rawTurnRate, -effectiveMaxTurn, effectiveMaxTurn);
            }
            else
            {
                integralError = integralCandidate;
            }
        }
        else
        {
            if (allowed) integralError = integralCandidate;
        }

        // Apply rate limits in two stages so the rigidbody sees a smooth yaw command rather than a step change.
        float cmdStep = Mathf.Max(0.01f, effectiveTurnRateChange) * dt;
        rateLimitedTurnRate = Mathf.MoveTowards(rateLimitedTurnRate, targetTurnRate, cmdStep);

        float outStep = Mathf.Max(0.01f, maxAngularAcceleration) * dt;
        appliedTurnRate = Mathf.MoveTowards(appliedTurnRate, rateLimitedTurnRate, outStep);

        rb.angularVelocity = new Vector3(0f, appliedTurnRate, 0f);

        float desired = smoothedTargetVelocity;

        // Forward motion is handled with simple drive/brake forces instead of setting the velocity directly.
        if (desired <= 0.01f)
        {
            float brake = Mathf.Clamp(obstacleBrakeGain * Mathf.Max(0f, forwardSpeed), 0f, obstacleMaxBrakeAccel);
            rb.AddForce(-transform.forward * brake, ForceMode.Acceleration);

            if (speed < 0.2f)
                rb.linearVelocity = Vector3.zero;
        }
        else
        {
            float forwardForce = baseForwardSpeed;

            if (speed > desired * 1.05f) forwardForce *= 0.20f;
            else if (speed > desired * 0.95f) forwardForce *= 0.60f;

            rb.AddForce(transform.forward * forwardForce * dt, ForceMode.Acceleration);

            if (forwardSpeed > desired)
            {
                float brake = Mathf.Clamp(obstacleBrakeGain * (forwardSpeed - desired), 0f, obstacleMaxBrakeAccel);
                rb.AddForce(-transform.forward * brake, ForceMode.Acceleration);
            }
        }

        if (desired > 0.1f && speed > desired * 1.18f)
        {
            Vector3 clampedVel = rb.linearVelocity.normalized * desired;
            clampedVel.y = rb.linearVelocity.y;
            rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, clampedVel, 0.14f);
        }

        if (desired > 0.1f && speed < minimumSpeed && (isRecovering || isInCurve))
        {
            Vector3 minVel = transform.forward * minimumSpeed;
            minVel.y = rb.linearVelocity.y;
            rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, minVel, 0.15f);
        }

        if (showDebugLog)
        {
            Debug.Log($"[v10 PID] E:{e:F3} Iacc:{integralErrorSmoothed:F3} P:{pTerm:F3} I:{iTerm:F3} D:{dTerm:F3} " +
                      $"H:{hTerm:F3} FF:{ffTerm:F3} Turn:{appliedTurnRate:F3} V:{speed:F2} Target:{desired:F2} Avoid:{avoidState}");
        }
    }

    private float ComputePurePursuitYawRate(Vector3 aimLocal, float speed)
    {
        // Pure pursuit converts the lookahead point into a curvature request in the robot frame.
        float x = aimLocal.x;
        float z = aimLocal.z;
        float L2 = x * x + z * z;
        if (L2 < 0.0001f) return 0f;

        float kappa = 2f * x / L2;
        return speed * kappa;
    }

    private float ExpAlpha(float timeConstant)
    {
        // Convert a time constant into a frame-rate-independent interpolation factor.
        float tau = Mathf.Max(0.0001f, timeConstant);
        return 1f - Mathf.Exp(-Time.fixedDeltaTime / tau);
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 originNear = transform.position + transform.forward * sensorForwardDistance;
        originNear.y = transform.position.y + sensorHeightOffset;

        for (int i = 0; i < sensorsPerSide; i++)
        {
            float t = (sensorsPerSide > 1) ? (float)i / (sensorsPerSide - 1) : 0f;
            float offset = Mathf.Lerp(0.3f, sensorSpread, t);

            Gizmos.color = Color.white;
            Gizmos.DrawRay(originNear - transform.right * offset, Vector3.down * raycastDownDistance);

            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(originNear + transform.right * offset, Vector3.down * raycastDownDistance);
        }

        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(originNear, 0.1f);

        Vector3 fanOriginBase = transform.position + Vector3.up * obstacleSensorHeight + transform.forward * obstacleSensorForwardOffset;
        int raysPerCol = Mathf.Max(1, obstacleFanRayCount / 3);
        float[] latOffs = { -obstacleFanLateralSpread, 0f, +obstacleFanLateralSpread };

        for (int col = 0; col < 3; col++)
        {
            Vector3 colOrig = fanOriginBase + transform.right * latOffs[col];

            for (int r = 0; r < raysPerCol; r++)
            {
                float ft = (raysPerCol > 1) ? (float)r / (raysPerCol - 1) : 0.5f;
                float angle = Mathf.Lerp(-obstacleFanHalfAngle, +obstacleFanHalfAngle, ft);
                Vector3 rayDir = Quaternion.AngleAxis(angle, Vector3.up) * transform.forward;

                Gizmos.color = new Color(1f, 0.3f, 0.3f, 0.6f);
                Gizmos.DrawRay(colOrig, rayDir * obstacleDetectDistance);
                Gizmos.DrawWireSphere(colOrig, obstacleSphereRadius);
            }
        }

        Gizmos.color = new Color(1f, 0.6f, 0f, 0.8f);
        for (int i = 0; i < sideSensorCount; i++)
        {
            float st = (sideSensorCount > 1) ? (float)i / (sideSensorCount - 1) : 0.5f;
            float fwd = Mathf.Lerp(-sideSensorLengthSpread, sideSensorLengthSpread, st);
            Vector3 o = transform.position + Vector3.up * sideSensorHeight + transform.forward * fwd;
            Gizmos.DrawRay(o, transform.right * sideSensorRange);
            Gizmos.DrawWireSphere(o, sideSensorRadius);
        }
    }
}
