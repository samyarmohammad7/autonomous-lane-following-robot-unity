using UnityEngine;

// Simple script to rotate wheels visually based on the robot's movement
// Attach this to Car8 object

public class SimpleWheelRotation : MonoBehaviour
{
    [Header("Wheel References")]
    [Tooltip("Assign all wheel objects that should rotate")]
    public Transform[] wheels;

    [Header("Rotation Settings")]
    [Tooltip("Speed multiplier for wheel rotation")]
    public float rotationSpeed = 500f;

    [Tooltip("Reference to the Robot (parent) to get velocity")]
    public Transform robotTransform;

    private Rigidbody robotRigidbody;

    void Start()
    {
        // Auto-find the robot if not assigned
        if (robotTransform == null)
        {
            robotTransform = transform.parent;
        }

        if (robotTransform != null)
        {
            robotRigidbody = robotTransform.GetComponent<Rigidbody>();
        }

        if (wheels.Length == 0)
        {
            Debug.LogWarning("No wheels assigned! Please assign wheel transforms in the Inspector.");
        }

        Debug.Log("SimpleWheelRotation initialized!");
    }

    void Update()
    {
        if (wheels.Length == 0) return;

        // Get the robot's velocity
        float speed = 0f;
        if (robotRigidbody != null)
        {
            speed = robotRigidbody.linearVelocity.magnitude;
        }

        // Calculate rotation amount based on speed
        float rotationAmount = speed * rotationSpeed * Time.deltaTime;

        // Rotate all wheels around their local X axis
        foreach (Transform wheel in wheels)
        {
            if (wheel != null)
            {
                wheel.Rotate(rotationAmount, 0, 0, Space.Self);
            }
        }
    }
}