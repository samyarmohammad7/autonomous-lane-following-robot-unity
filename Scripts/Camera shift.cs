using UnityEngine;

public class TimedMultiViewFollowCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform target;

    [Header("Follow Settings")]
    public float positionSmoothTime = 0.18f;
    public float rotationSmoothSpeed = 8f;
    public float switchInterval = 6f;
    public float extraBlendFactor = 1.0f;
    public float fovLerpSpeed = 6f;

    [Header("Views")]
    public ViewPreset[] views;

    [System.Serializable]
    public class ViewPreset
    {
        public string name = "Follow";
        public Vector3 localOffset = new Vector3(0f, 3f, -8f);
        public Vector3 lookAtLocalPoint = new Vector3(0f, 1f, 0f);
        public float fieldOfView = 60f;
    }

    private int currentViewIndex = 0;
    private float viewTimer = 0f;
    private Vector3 positionVelocity;
    private Camera cam;

    void Awake()
    {
        cam = GetComponent<Camera>();
        if (cam == null) cam = Camera.main;
    }

    void Start()
    {
        if (views == null || views.Length == 0)
        {
            // Fall back to a few usable demo angles if nothing is set in the Inspector.
            SetDefaultViews();
        }

        if (currentViewIndex >= views.Length)
        {
            currentViewIndex = 0;
        }
    }

    void LateUpdate()
    {
        if (target == null || views == null || views.Length == 0) return;

        if (switchInterval > 0f)
        {
            viewTimer += Time.deltaTime;
            if (viewTimer >= switchInterval)
            {
                // Rotate through the presets automatically during the run.
                NextView();
            }
        }

        ViewPreset currentView = views[currentViewIndex];

        // Offsets are stored in target local space so each view stays aligned with the robot.
        Vector3 wantedPosition = target.TransformPoint(currentView.localOffset);
        float moveSmooth = Mathf.Max(0.01f, positionSmoothTime) / Mathf.Max(0.1f, extraBlendFactor);
        transform.position = Vector3.SmoothDamp(transform.position, wantedPosition, ref positionVelocity, moveSmooth);

        Vector3 lookPoint = target.TransformPoint(currentView.lookAtLocalPoint);
        Vector3 lookDirection = lookPoint - transform.position;

        if (lookDirection.sqrMagnitude > 0.001f)
        {
            // Avoid trying to build a rotation from a near-zero direction vector.
            Quaternion wantedRotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
            float turnSpeed = Mathf.Max(0.1f, rotationSmoothSpeed) * Mathf.Max(0.1f, extraBlendFactor);
            transform.rotation = Quaternion.Slerp(transform.rotation, wantedRotation, turnSpeed * Time.deltaTime);
        }

        if (cam != null && currentView.fieldOfView > 0f)
        {
            // Blend FOV so the switch between views feels less abrupt.
            cam.fieldOfView = Mathf.Lerp(cam.fieldOfView, currentView.fieldOfView, fovLerpSpeed * Time.deltaTime);
        }
    }

    private void SetDefaultViews()
    {
        // Basic angles for testing: follow, higher follow, top, side and front.
        views = new ViewPreset[]
        {
            new ViewPreset { name = "Follow", localOffset = new Vector3(0f, 3f, -8f), lookAtLocalPoint = new Vector3(0f, 1f, 0f), fieldOfView = 60f },
            new ViewPreset { name = "Higher", localOffset = new Vector3(0f, 5.5f, -10f), lookAtLocalPoint = new Vector3(0f, 1f, 0f), fieldOfView = 55f },
            new ViewPreset { name = "Top", localOffset = new Vector3(0f, 14f, -2f), lookAtLocalPoint = new Vector3(0f, 0.5f, 0f), fieldOfView = 60f },
            new ViewPreset { name = "Side", localOffset = new Vector3(7f, 3f, -2f), lookAtLocalPoint = new Vector3(0f, 1f, 0f), fieldOfView = 60f },
            new ViewPreset { name = "Front", localOffset = new Vector3(0f, 2.5f, 7f), lookAtLocalPoint = new Vector3(0f, 1f, 0f), fieldOfView = 65f }
        };
    }

    public void NextView()
    {
        if (views == null || views.Length == 0) return;

        currentViewIndex++;
        if (currentViewIndex >= views.Length)
        {
            // Wrap back to the first camera preset.
            currentViewIndex = 0;
        }

        viewTimer = 0f;
    }
}
