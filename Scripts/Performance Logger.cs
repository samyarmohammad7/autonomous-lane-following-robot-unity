using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;

public class PerformanceLogger : MonoBehaviour
{
    [Header("Run")]
    public string runLabel = "TestRun";
    public string trackName = "Unknown";
    [TextArea]
    public string notes = "";

    [Header("Logging")]
    public int logEveryNFrames = 1;
    public string outputFolder = "LogData";

    private MonoBehaviour controller;
    private Rigidbody rb;
    private Type controllerType;

    private StreamWriter frameWriter;
    private string frameFilePath;
    private string summaryFilePath;
    private bool summaryWritten = false;

    private float runStartTime = 0f;
    private int frameCount = 0;
    private int loggedFrameCount = 0;

    private float sumAbsLateralError = 0f;
    private float sumSquaredLateralError = 0f;
    private float maxAbsLateralError = 0f;
    private int errorSampleCount = 0;

    private float sumSpeed = 0f;
    private float maxSpeed = 0f;

    private int offTrackCount = 0;
    private bool wasOffTrack = false;

    private int lapCount = 0;
    private float lastLapTime = 0f;
    private List<float> lapTimes = new List<float>();

    private float totalTimeInCurve = 0f;
    private float totalTimeRecovering = 0f;

    void Start()
    {
        controller = GetComponent("RobotController") as MonoBehaviour;
        rb = GetComponent<Rigidbody>();

        if (controller == null)
        {
            Debug.LogError("[PerformanceLogger] RobotController not found!");
            enabled = false;
            return;
        }

        if (rb == null)
        {
            Debug.LogError("[PerformanceLogger] Rigidbody not found!");
            enabled = false;
            return;
        }

        controllerType = controller.GetType();
        logEveryNFrames = Mathf.Max(1, logEveryNFrames);

        SetupOutputFiles();
        ResetRunStats();

        Debug.Log("[PerformanceLogger] Started logging to: " + frameFilePath);
    }

    void FixedUpdate()
    {
        if (controller == null || rb == null || frameWriter == null) return;

        frameCount++;

        // Use reflection so the logger still works even if the controller version changes.
        float lateralError = ReadFirstFloat("smoothedError", "currentLateralError", "currentError");
        float headingError = ReadFirstFloat("smoothedHeadingError", "currentHeadingError");
        float turnRate = ReadFirstFloat("appliedTurnRate", "currentTurnRate", "rateLimitedTurnRate");
        float targetSpeed = ReadFirstFloat("smoothedTargetVelocity", "targetVelocity");

        bool isInCurve = ReadBool("isInCurve");
        bool isRecovering = ReadBool("isRecovering");
        bool leftDetected = ReadBool("leftLineDetected");
        bool rightDetected = ReadBool("rightLineDetected");

        float pTerm = ReadFirstFloat("debugPTerm");
        float iTerm = ReadFirstFloat("debugITerm");
        float dTerm = ReadFirstFloat("debugDTerm");
        float hTerm = ReadFirstFloat("debugHTerm");
        float ffTerm = ReadFirstFloat("debugFFTerm");

        string avoidState = ReadFieldAsString("avoidState");
        if (string.IsNullOrEmpty(avoidState))
            avoidState = "None";

        float speed = rb.linearVelocity.magnitude;
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
        float absError = Mathf.Abs(lateralError);

        sumAbsLateralError += absError;
        sumSquaredLateralError += lateralError * lateralError;
        if (absError > maxAbsLateralError) maxAbsLateralError = absError;
        errorSampleCount++;

        sumSpeed += speed;
        if (speed > maxSpeed) maxSpeed = speed;

        bool currentlyOffTrack = !leftDetected && !rightDetected;
        if (currentlyOffTrack && !wasOffTrack)
            offTrackCount++;
        wasOffTrack = currentlyOffTrack;

        if (isInCurve) totalTimeInCurve += Time.fixedDeltaTime;
        if (isRecovering) totalTimeRecovering += Time.fixedDeltaTime;

        if (frameCount % logEveryNFrames != 0) return;

        loggedFrameCount++;
        float elapsed = Time.time - runStartTime;

        // Older controller versions may not expose turn rate directly, so use rigidbody yaw as a fallback.
        if (Mathf.Abs(turnRate) < 0.0001f && Mathf.Abs(rb.angularVelocity.y) > 0.001f)
            turnRate = rb.angularVelocity.y;

        frameWriter.WriteLine(
            $"{elapsed:F4}," +
            $"{transform.position.x:F4},{transform.position.z:F4}," +
            $"{speed:F4}," +
            $"{forwardSpeed:F4}," +
            $"{lateralError:F6}," +
            $"{headingError:F6}," +
            $"{turnRate:F6}," +
            $"{pTerm:F6},{iTerm:F6},{dTerm:F6},{hTerm:F6},{ffTerm:F6}," +
            $"{(isInCurve ? 1 : 0)},{(isRecovering ? 1 : 0)}," +
            $"{ToCsvCell(avoidState)}," +
            $"{(leftDetected ? 1 : 0)},{(rightDetected ? 1 : 0)}," +
            $"{targetSpeed:F4}," +
            $"{lapCount}"
        );
    }

    void Update()
    {
        if (Keyboard.current == null) return;

        // Lap marks are manual so runs can be tested on different track layouts.
        if (Keyboard.current.lKey.wasPressedThisFrame)
        {
            lapCount++;

            float now = Time.time;
            float lapTime = now - lastLapTime;
            lapTimes.Add(lapTime);
            lastLapTime = now;

            Debug.Log($"[PerformanceLogger] Lap {lapCount} completed. Time: {lapTime:F2}s");
        }
    }

    void OnDestroy()
    {
        WriteSummary();
        CloseWriter();
    }

    void OnApplicationQuit()
    {
        WriteSummary();
        CloseWriter();
    }

    private void SetupOutputFiles()
    {
        string basePath = Path.Combine(Application.dataPath, "..", outputFolder);
        Directory.CreateDirectory(basePath);

        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string safeLabel = MakeSafeFileLabel(runLabel);

        frameFilePath = Path.Combine(basePath, safeLabel + "_" + timestamp + ".csv");
        summaryFilePath = Path.Combine(basePath, "RunSummary.csv");

        frameWriter = new StreamWriter(frameFilePath, false, Encoding.UTF8);
        frameWriter.WriteLine(
            "Time," +
            "PosX,PosZ," +
            "Speed," +
            "ForwardSpeed," +
            "LateralError," +
            "HeadingError," +
            "TurnRate," +
            "PTerm,ITerm,DTerm,HTerm,FFTerm," +
            "IsInCurve,IsRecovering," +
            "AvoidState," +
            "LeftDetected,RightDetected," +
            "TargetSpeed," +
            "LapCount"
        );
    }

    private void ResetRunStats()
    {
        runStartTime = Time.time;
        frameCount = 0;
        loggedFrameCount = 0;

        sumAbsLateralError = 0f;
        sumSquaredLateralError = 0f;
        maxAbsLateralError = 0f;
        errorSampleCount = 0;

        sumSpeed = 0f;
        maxSpeed = 0f;

        offTrackCount = 0;
        wasOffTrack = false;

        lapCount = 0;
        lapTimes.Clear();
        lastLapTime = Time.time;

        totalTimeInCurve = 0f;
        totalTimeRecovering = 0f;
        summaryWritten = false;
    }

    private void CloseWriter()
    {
        if (frameWriter != null)
        {
            frameWriter.Flush();
            frameWriter.Close();
            frameWriter = null;
        }

        Debug.Log($"[PerformanceLogger] Stopped. {loggedFrameCount} frames written.");
    }

    private void WriteSummary()
    {
        if (summaryWritten || errorSampleCount == 0) return;
        summaryWritten = true;

        float totalTime = Time.time - runStartTime;
        float meanAbsError = sumAbsLateralError / errorSampleCount;
        float rmsError = Mathf.Sqrt(sumSquaredLateralError / errorSampleCount);
        float meanSpeed = sumSpeed / errorSampleCount;

        float kp = ReadFirstFloat(true, "Kp");
        float ki = ReadFirstFloat(true, "Ki");
        float kd = ReadFirstFloat(true, "Kd");
        float kh = ReadFirstFloat(true, "Kh");
        float kff = ReadFirstFloat(true, "Kff");

        string lapTimesStr = lapTimes.Count > 0
            ? string.Join(";", lapTimes.ConvertAll(t => t.ToString("F2")))
            : "N/A";

        bool writeHeader = !File.Exists(summaryFilePath);

        // The summary file is append-only so multiple runs can be compared later.
        using (StreamWriter sw = new StreamWriter(summaryFilePath, true, Encoding.UTF8))
        {
            if (writeHeader)
            {
                sw.WriteLine(
                    "RunLabel,TrackName,DateTime," +
                    "Kp,Ki,Kd,Kh,Kff," +
                    "TotalTime_s,Laps,LapTimes_s," +
                    "MeanAbsError,RMS_Error,MaxAbsError," +
                    "MeanSpeed,MaxSpeed," +
                    "OffTrackEvents," +
                    "TimeInCurves_s,TimeRecovering_s," +
                    "TotalFrames,Notes"
                );
            }

            sw.WriteLine(
                $"{ToCsvCell(runLabel)},{ToCsvCell(trackName)},{DateTime.Now:yyyy-MM-dd HH:mm:ss}," +
                $"{kp:F3},{ki:F4},{kd:F3},{kh:F3},{kff:F3}," +
                $"{totalTime:F2},{lapCount},{ToCsvCell(lapTimesStr)}," +
                $"{meanAbsError:F6},{rmsError:F6},{maxAbsLateralError:F6}," +
                $"{meanSpeed:F4},{maxSpeed:F4}," +
                $"{offTrackCount}," +
                $"{totalTimeInCurve:F2},{totalTimeRecovering:F2}," +
                $"{loggedFrameCount},{ToCsvCell(notes)}"
            );
        }

        Debug.Log(
            $"[PerformanceLogger] Summary saved. MAE:{meanAbsError:F4} RMS:{rmsError:F4} " +
            $"Max:{maxAbsLateralError:F4} OffTrack:{offTrackCount}"
        );
    }

    private float ReadFirstFloat(params string[] fieldNames)
    {
        return ReadFirstFloat(false, fieldNames);
    }

    private float ReadFirstFloat(bool publicOnly, params string[] fieldNames)
    {
        for (int i = 0; i < fieldNames.Length; i++)
        {
            if (TryReadFloat(fieldNames[i], out float value, publicOnly))
                return value;
        }

        return 0f;
    }

    private bool TryReadFloat(string fieldName, out float value, bool publicOnly = false)
    {
        value = 0f;

        BindingFlags flags = publicOnly
            ? BindingFlags.Public | BindingFlags.Instance
            : BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;

        FieldInfo field = controllerType.GetField(fieldName, flags);
        if (field == null || field.FieldType != typeof(float))
            return false;

        value = (float)field.GetValue(controller);
        return true;
    }

    private bool ReadBool(string fieldName)
    {
        BindingFlags flags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
        FieldInfo field = controllerType.GetField(fieldName, flags);

        if (field != null && field.FieldType == typeof(bool))
            return (bool)field.GetValue(controller);

        return false;
    }

    private string ReadFieldAsString(string fieldName)
    {
        BindingFlags flags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
        FieldInfo field = controllerType.GetField(fieldName, flags);

        if (field == null) return "";

        object value = field.GetValue(controller);
        return value != null ? value.ToString() : "";
    }

    private string MakeSafeFileLabel(string value)
    {
        if (string.IsNullOrWhiteSpace(value))
            return "Run";

        string safe = value.Trim().Replace(" ", "_").Replace("/", "-").Replace("\\", "-");
        char[] invalidChars = Path.GetInvalidFileNameChars();

        for (int i = 0; i < invalidChars.Length; i++)
        {
            safe = safe.Replace(invalidChars[i].ToString(), "");
        }

        return string.IsNullOrWhiteSpace(safe) ? "Run" : safe;
    }

    private string ToCsvCell(string value)
    {
        if (string.IsNullOrEmpty(value))
            return "\"\"";

        string escaped = value.Replace("\"", "\"\"");
        return "\"" + escaped + "\"";
    }
}
