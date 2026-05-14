using UnityEngine;

public class TrackManager : MonoBehaviour
{
    [Header("Track Configuration")]
    public float roadWidth = 5f;
    public float laneHalfWidth = 2.0f;
    public float lineWidth = 0.15f;
    public float segmentLength = 10f;

    [Header("Visual Materials")]
    public Material roadMaterial;
    public Color roadColor = new Color(0.2f, 0.2f, 0.2f);

    [Header("Layer Names")]
    public string leftLaneLayerName = "LeftLane";
    public string rightLaneLayerName = "RightLane";

    private Vector3 currentPosition = Vector3.zero;
    private Quaternion currentRotation = Quaternion.identity;
    private int segmentCount = 0;

    [ContextMenu("Reset Track Builder")]
    public void ResetTrackBuilder()
    {
        currentPosition = transform.position;
        currentRotation = transform.rotation;
        segmentCount = 0;
        Debug.Log("[TrackManager] Track builder reset to origin.");
    }

    [ContextMenu("Create Basic Test Track")]
    public void CreateBasicTestTrack()
    {
        ClearTrack();
        ResetTrackBuilder();

        AddStraightSection(3);
        AddCurveSection(90f, 8f);
        AddStraightSection(3);
        AddCurveSection(90f, 8f);
        AddStraightSection(3);
        AddCurveSection(90f, 8f);
        AddStraightSection(3);
        AddCurveSection(90f, 8f);

        Debug.Log("[TrackManager] Basic test track created!");
    }

    [ContextMenu("Create Straight Test Track")]
    public void CreateStraightTestTrack()
    {
        ClearTrack();
        ResetTrackBuilder();

        AddStraightSection(10);

        Debug.Log("[TrackManager] Straight test track created!");
    }

    [ContextMenu("Create S-Curve Test Track")]
    public void CreateSCurveTestTrack()
    {
        ClearTrack();
        ResetTrackBuilder();

        AddStraightSection(2);
        AddCurveSection(45f, 6f);
        AddStraightSection(1);
        AddCurveSection(-45f, 6f);
        AddCurveSection(-45f, 6f);
        AddStraightSection(1);
        AddCurveSection(45f, 6f);
        AddStraightSection(2);

        Debug.Log("[TrackManager] S-Curve test track created!");
    }

    [ContextMenu("Clear Track")]
    public void ClearTrack()
    {
        while (transform.childCount > 0)
        {
            if (Application.isPlaying)
                Destroy(transform.GetChild(0).gameObject);
            else
                DestroyImmediate(transform.GetChild(0).gameObject);
        }

        segmentCount = 0;
        Debug.Log("[TrackManager] Track cleared.");
    }

    public void AddStraightSection(int numSegments)
    {
        for (int i = 0; i < numSegments; i++)
        {
            CreateTrackSegment(currentPosition, currentRotation);
            currentPosition += currentRotation * Vector3.forward * segmentLength;
            segmentCount++;
        }
    }

    public void AddCurveSection(float totalAngle, float radius)
    {
        int numSegments = Mathf.Max(3, Mathf.RoundToInt(Mathf.Abs(totalAngle) / 15f));
        float anglePerSegment = totalAngle / numSegments;
        float segmentArcLength = (2f * Mathf.PI * radius) * (Mathf.Abs(anglePerSegment) / 360f);

        for (int i = 0; i < numSegments; i++)
        {
            CreateTrackSegment(currentPosition, currentRotation, segmentArcLength);
            currentRotation = currentRotation * Quaternion.Euler(0f, anglePerSegment, 0f);
            currentPosition += currentRotation * Vector3.forward * segmentArcLength;
            segmentCount++;
        }
    }

    private void CreateTrackSegment(Vector3 position, Quaternion rotation, float length = -1f)
    {
        if (length < 0f)
            length = segmentLength;

        GameObject segment = new GameObject("TrackSegment_" + segmentCount);
        segment.transform.SetParent(transform);
        segment.transform.position = position;
        segment.transform.rotation = rotation;

        GameObject road = GameObject.CreatePrimitive(PrimitiveType.Cube);
        road.name = "RoadSurface";
        road.transform.SetParent(segment.transform);
        road.transform.localPosition = new Vector3(0f, -0.05f, length / 2f);
        road.transform.localRotation = Quaternion.identity;
        road.transform.localScale = new Vector3(roadWidth, 0.1f, length);

        Renderer roadRenderer = road.GetComponent<Renderer>();
        if (roadMaterial != null)
        {
            roadRenderer.material = roadMaterial;
        }
        else
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            Material mat = new Material(shader);
            mat.color = roadColor;
            roadRenderer.material = mat;
        }

        road.layer = 0;

        CreateLaneLine(segment.transform, "LeftLaneLine", -laneHalfWidth, Color.white, leftLaneLayerName, length);
        CreateLaneLine(segment.transform, "RightLaneLine", laneHalfWidth, Color.yellow, rightLaneLayerName, length);
    }

    private void CreateLaneLine(Transform parent, string name, float lateralOffset, Color color, string layerName, float length)
    {
        GameObject line = GameObject.CreatePrimitive(PrimitiveType.Cube);
        line.name = name;
        line.transform.SetParent(parent);
        line.transform.localPosition = new Vector3(lateralOffset, 0.01f, length / 2f);
        line.transform.localRotation = Quaternion.identity;
        line.transform.localScale = new Vector3(lineWidth, 0.02f, length);

        int layer = LayerMask.NameToLayer(layerName);
        if (layer == -1)
        {
            Debug.LogWarning("[TrackManager] Layer " + layerName + " not found! Please create it.");
        }
        else
        {
            line.layer = layer;
        }

        Renderer lineRenderer = line.GetComponent<Renderer>();
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        Material mat = new Material(shader);
        mat.color = color;
        lineRenderer.material = mat;
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(currentPosition, 0.5f);

        Gizmos.color = Color.blue;
        Gizmos.DrawRay(currentPosition, currentRotation * Vector3.forward * 2f);

        Gizmos.color = Color.yellow;
        Vector3 right = currentRotation * Vector3.right;
        Gizmos.DrawLine(currentPosition - right * laneHalfWidth, currentPosition + right * laneHalfWidth);
    }
}