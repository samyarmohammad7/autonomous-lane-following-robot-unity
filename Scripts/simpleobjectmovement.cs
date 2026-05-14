using UnityEngine;

public class BackAndForth : MonoBehaviour
{
    [Header("Movement")]
    public float moveSpeed = 2f;
    public float moveTime = 2f;
    
    [Header("Turning")]
    public float spinTime = 0.5f;

    private float timer = 0f;
    private bool goingForward = true;
    private bool isSpinning = false;

    private Quaternion spinStart;
    private Quaternion spinEnd;

    void Update()
    {
        if (isSpinning)
        {
            timer += Time.deltaTime;
            float t = Mathf.Clamp01(timer / Mathf.Max(0.01f, spinTime));
            transform.localRotation = Quaternion.Slerp(spinStart, spinEnd, t);

            if (t >= 1f)
            {
                isSpinning = false;
                goingForward = !goingForward;
                timer = 0f;
            }

            return;
        }

        // The object moves in local forward space, so after the 180 turn it drives back along the same path.
        transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime, Space.Self);

        timer += Time.deltaTime;
        if (timer >= moveTime)
        {
            // Rotate in place before starting the return part of the movement.
            isSpinning = true;
            timer = 0f;
            spinStart = transform.localRotation;
            spinEnd = spinStart * Quaternion.Euler(0f, 180f, 0f);
        }
    }
}
