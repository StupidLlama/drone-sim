// Simple multirotor model. It behaves like a flight controller in velocity
// (offboard) mode: it takes body-frame velocity + yaw-rate setpoints and
// tracks them with a first-order response, while gravity, quadratic air drag
// and wind push back. Python never sees the true disturbance, only the result.
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    [Header("Limits")]
    public float maxSpeed = 6f;          // m/s horizontal
    public float maxClimb = 2f;          // m/s
    public float maxYawRate = 1.5f;      // rad/s
    public float maxAccel = 8f;          // m/s^2 the motors can produce (beyond hover)

    [Header("Dynamics")]
    [Tooltip("Velocity response time constant (s). Higher = sluggish drone")]
    public float velocityTau = 0.35f;
    [Tooltip("Quadratic drag, (m/s^2) per (m/s)^2 of air-relative speed")]
    public float dragCoeff = 0.15f;

    [Header("Visual")]
    public Transform visual;             // tilted to look like a real quad; camera is NOT under it (gimbal)
    public float maxTiltDeg = 25f;

    [Header("Manual override (M to toggle)")]
    public bool manualOverride = false;
    public float manualSpeed = 3f;

    [Header("Battery")]
    [Tooltip("Minutes of flight on a full battery (0 = never drains). B drops it to 15% to test the failsafe.")]
    public float flightTimeMinutes = 15f;
    [Range(0f, 1f)] public float battery = 1f;

    [Header("Range sensors")]
    public float rangeMax = 20f;
    public LayerMask obstacleMask = ~0;

    public Vector4 CurrentCommand { get; private set; }   // fwd, right, up, yawRate
    public string CommandSource { get; private set; } = "hover";
    public readonly Dictionary<string, float> Ranges = new Dictionary<string, float>();

    Rigidbody _rb;
    Vector3 _spawnPos;
    Quaternion _spawnRot;
    Vector3 _lastAccelCmd;

    public Vector3 Velocity
    {
        get
        {
#if UNITY_6000_0_OR_NEWER
            return _rb.linearVelocity;
#else
            return _rb.velocity;
#endif
        }
        private set
        {
#if UNITY_6000_0_OR_NEWER
            _rb.linearVelocity = value;
#else
            _rb.velocity = value;
#endif
        }
    }

    public float Altitude => Ranges.TryGetValue("down", out var d) ? d : transform.position.y;

    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        _rb.useGravity = true;
        _rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        _rb.interpolation = RigidbodyInterpolation.Interpolate;
        _rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        _spawnPos = transform.position;
        _spawnRot = transform.rotation;
    }

    void Update()
    {
        if (InputCompat.Down(KeyCode.M)) manualOverride = !manualOverride;
        if (InputCompat.Down(KeyCode.R)) ResetDrone();
        if (InputCompat.Down(KeyCode.B)) battery = battery > 0.15f ? 0.15f : 1f;
        if (flightTimeMinutes > 0f) battery = Mathf.Max(0f, battery - Time.deltaTime / (flightTimeMinutes * 60f));
        var bridge = PythonBridge.Instance;
        if (bridge != null && bridge.ConsumeResetRequest()) ResetDrone();
    }

    public void ResetDrone()
    {
        transform.SetPositionAndRotation(_spawnPos, _spawnRot);
        _rb.position = _spawnPos;
        _rb.rotation = _spawnRot;
        Velocity = Vector3.zero;
        _rb.angularVelocity = Vector3.zero;
        battery = 1f;
    }

    Vector4 ManualCommand()
    {
        // I/K forward-back, J/L strafe, U/O yaw, Y/H up-down
        return new Vector4(
            InputCompat.Axis(KeyCode.K, KeyCode.I) * manualSpeed,
            InputCompat.Axis(KeyCode.J, KeyCode.L) * manualSpeed,
            InputCompat.Axis(KeyCode.H, KeyCode.Y) * manualSpeed * 0.5f,
            InputCompat.Axis(KeyCode.U, KeyCode.O) * 1.0f);
    }

    void FixedUpdate()
    {
        Vector4 cmd = Vector4.zero;
        CommandSource = "hover";
        if (manualOverride) { cmd = ManualCommand(); CommandSource = "manual"; }
        else if (PythonBridge.Instance != null && PythonBridge.Instance.TryGetCommand(out var c))
        { cmd = c; CommandSource = "python"; }

        // clamp like a real FC would
        var h = Vector2.ClampMagnitude(new Vector2(cmd.x, cmd.y), maxSpeed);
        cmd = new Vector4(h.x, h.y, Mathf.Clamp(cmd.z, -maxClimb, maxClimb),
                          Mathf.Clamp(cmd.w, -maxYawRate, maxYawRate));
        CurrentCommand = cmd;

        // Body is yaw-only (roll/pitch are frozen), so forward/right are level.
        Vector3 vSp = transform.forward * cmd.x + transform.right * cmd.y + Vector3.up * cmd.z;
        Vector3 v = Velocity;
        Vector3 accCmd = Vector3.ClampMagnitude((vSp - v) / Mathf.Max(0.05f, velocityTau), maxAccel);
        _lastAccelCmd = accCmd;

        // "Flight controller": thrust = m * (desired accel - gravity). Ideal hover.
        _rb.AddForce(_rb.mass * (accCmd - Physics.gravity), ForceMode.Force);

        // Disturbances the controller doesn't know about.
        Vector3 wind = WindField.Instance != null ? WindField.Instance.Current : Vector3.zero;
        Vector3 vRel = v - wind;
        _rb.AddForce(-_rb.mass * dragCoeff * vRel.magnitude * vRel, ForceMode.Force);

        // Yaw: + = clockwise from above = turn right (Unity +Y rotation).
        _rb.angularVelocity = new Vector3(0f, cmd.w, 0f);

        UpdateRanges();
    }

    void LateUpdate()
    {
        if (visual == null) return;
        // tilt the model into the acceleration like a quad does
        Vector3 local = transform.InverseTransformDirection(_lastAccelCmd);
        float pitch = Mathf.Clamp(Mathf.Atan2(local.z, 9.81f) * Mathf.Rad2Deg, -maxTiltDeg, maxTiltDeg);
        float roll = Mathf.Clamp(Mathf.Atan2(local.x, 9.81f) * Mathf.Rad2Deg, -maxTiltDeg, maxTiltDeg);
        visual.localRotation = Quaternion.Slerp(visual.localRotation, Quaternion.Euler(pitch, 0f, -roll), 10f * Time.deltaTime);
    }

    void UpdateRanges()
    {
        Ranges.Clear();
        Cast("front", transform.forward);
        Cast("back", -transform.forward);
        Cast("left", -transform.right);
        Cast("right", transform.right);
        Cast("front_left", (transform.forward - transform.right).normalized);
        Cast("front_right", (transform.forward + transform.right).normalized);
        Cast("down", Vector3.down);
    }

    void Cast(string key, Vector3 dir)
    {
        // Rays starting inside our own collider don't hit it.
        if (Physics.Raycast(transform.position, dir, out var hit, rangeMax, obstacleMask, QueryTriggerInteraction.Ignore))
            Ranges[key] = hit.distance;
    }

    /// <summary>Telemetry JSON header sent with every camera frame.</summary>
    public string TelemetryJson(int frameId, double t)
    {
        var ci = CultureInfo.InvariantCulture;
        string F(float x) => x.ToString("0.####", ci);
        Vector3 p = transform.position;
        Vector3 v = Velocity;
        var sb = new StringBuilder(256);
        sb.Append("{\"frame_id\":").Append(frameId)
          .Append(",\"t\":").Append(t.ToString("0.####", ci))
          .Append(",\"pos\":[").Append(F(p.x)).Append(',').Append(F(p.y)).Append(',').Append(F(p.z)).Append(']')
          .Append(",\"yaw_deg\":").Append(F(transform.eulerAngles.y))
          .Append(",\"alt\":").Append(F(Altitude))
          .Append(",\"battery\":").Append(F(battery))
          .Append(",\"vel_body\":[").Append(F(Vector3.Dot(v, transform.forward))).Append(',')
                                     .Append(F(Vector3.Dot(v, transform.right))).Append(',')
                                     .Append(F(v.y)).Append(']')
          .Append(",\"ranges\":{");
        bool first = true;
        foreach (var kv in Ranges)
        {
            if (!first) sb.Append(',');
            sb.Append('"').Append(kv.Key).Append("\":").Append(F(kv.Value));
            first = false;
        }
        sb.Append('}');
        var sonar = GetComponent<SonarArray>();
        if (sonar != null && sonar.enabled) sb.Append(",\"sonar\":").Append(sonar.Json());
        sb.Append('}');
        return sb.ToString();
    }
}
