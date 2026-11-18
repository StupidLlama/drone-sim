// Ultrasonic ("sonar") array: 18 cone-shaped range sensors around the drone,
// the same layout as drone_core.domain.models.sonar_layout():
//   8 level (every 45 deg), 4 tilted up 35 deg, 4 tilted down 35 deg, 1 up, 1 down.
// Each sensor reports the distance of the first echo inside its cone, like a
// real ultrasonic transducer. Python turns these cones into a 3-D voxel map
// (usecases/voxel_map.py) so the drone can climb over / drop under branches.
//
// Echo model: each cone is sampled with ~61 rays. The nearest surface answers
// only if it fills enough of the beam (minEchoFraction). A ray that hits a
// thin collider (a branch) counts only for the slice of its patch the branch
// covers, so twigs of a few millimetres are missed and thick branches are
// heard, as with real sensors. Only capsule/sphere colliders (the branches and
// trunks SceneBootstrap builds) get this thin-object model; boxes, the ground
// and imported meshes count as broad surfaces. Range noise grows with distance.
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

public class SonarArray : MonoBehaviour
{
    [Header("Sensor")]
    public float fovDeg = 30f;             // full cone angle
    public float minRange = 0.2f;
    public float maxRange = 7f;
    public float rateHz = 15f;
    [Range(0f, 0.3f)] public float minEchoFraction = 0.03f;
    public float noiseBase = 0.01f;        // m
    public float noisePerMetre = 0.01f;    // m per m of range
    public LayerMask mask = ~0;

    [Header("Debug")]
    [Tooltip("Draw each beam's axis in the Game view (N toggles). Yellow = echo, blue = clear.")]
    public bool showBeams = true;

    static readonly float[] RingFrac = { 0.25f, 0.5f, 0.75f, 1f };
    static readonly int[] RingCount = { 6, 12, 18, 24 };

    readonly List<Vector2> _layout = new List<Vector2>();     // (yaw, pitch) deg
    readonly List<Vector3[]> _cones = new List<Vector3[]>();  // local unit dirs per beam
    float[] _ranges;                                          // < 0 = no echo
    float _next;
    LineRenderer[] _lines;
    float[] _t; float[] _cover;

    public int BeamCount => _layout.Count;

    /// <summary>Layer for debug drawing. CameraStreamer hides it from the drone
    /// cameras, otherwise the colour tracker can lock onto the beams.
    /// (2 = Unity's built-in "Ignore Raycast" layer.)</summary>
    public const int DebugLayer = 2;

    void Awake()
    {
        float[] level = { 0, 45, 90, 135, 180, -135, -90, -45 };
        foreach (var a in level) _layout.Add(new Vector2(a, 0));
        foreach (var a in new float[] { 0, 90, 180, -90 }) _layout.Add(new Vector2(a, 35));
        foreach (var a in new float[] { 0, 90, 180, -90 }) _layout.Add(new Vector2(a, -35));
        _layout.Add(new Vector2(0, 90));
        _layout.Add(new Vector2(0, -90));
        foreach (var b in _layout) _cones.Add(ConeDirs(b.x, b.y));
        _ranges = new float[_layout.Count];
        for (int i = 0; i < _ranges.Length; i++) _ranges[i] = -1f;
        _t = new float[_cones[0].Length];
        _cover = new float[_cones[0].Length];
        BuildLines();
    }

    /// <summary>Unit vectors in drone-local space (x right, y up, z forward).</summary>
    Vector3[] ConeDirs(float yawDeg, float pitchDeg)
    {
        var rot = Quaternion.Euler(-pitchDeg, yawDeg, 0f);   // Unity: -x rotation pitches up
        var dirs = new List<Vector3> { rot * Vector3.forward };
        float half = fovDeg * 0.5f;
        for (int r = 0; r < RingFrac.Length; r++)
        {
            float th = half * RingFrac[r];
            for (int k = 0; k < RingCount[r]; k++)
            {
                float phi = 360f * k / RingCount[r];
                // tilt off the axis by th, spun around it by phi
                dirs.Add(rot * (Quaternion.AngleAxis(phi, Vector3.forward) * Quaternion.AngleAxis(th, Vector3.right) * Vector3.forward));
            }
        }
        return dirs.ToArray();
    }

    void Update()
    {
        if (InputCompat.Down(KeyCode.N)) showBeams = !showBeams;
        if (Time.time >= _next)
        {
            _next = Time.time + 1f / rateHz;
            Scan();
        }
        UpdateLines();
    }

    void Scan()
    {
        Vector3 o = transform.position;
        float cell = fovDeg * 0.5f * Mathf.Deg2Rad / RingFrac.Length;   // angle between rings
        for (int b = 0; b < _cones.Count; b++)
        {
            var dirs = _cones[b];
            int n = dirs.Length;
            for (int i = 0; i < n; i++)
            {
                _t[i] = float.PositiveInfinity;
                _cover[i] = 1f;
                // Drone root only yaws (tilt is visual), so this is a level sensor ring.
                if (Physics.Raycast(o, transform.rotation * dirs[i], out var hit, maxRange, mask, QueryTriggerInteraction.Ignore))
                {
                    _t[i] = hit.distance;
                    _cover[i] = Mathf.Min(1f, Thickness(hit.collider) / Mathf.Max(hit.distance * cell, 1e-4f));
                }
            }
            // First echo strong enough to hear; weak ones are skipped and the
            // next surface behind answers instead.
            float range = -1f;
            float floor = 0f;
            for (int guard = 0; guard < 8; guard++)
            {
                float t0 = float.PositiveInfinity;
                for (int i = 0; i < n; i++) if (_t[i] >= floor && _t[i] < t0) t0 = _t[i];
                if (float.IsPositiveInfinity(t0)) break;
                float strength = 0f;
                for (int i = 0; i < n; i++) if (_t[i] >= t0 && _t[i] < t0 + 0.3f) strength += _cover[i];
                if (strength / n >= minEchoFraction)
                {
                    range = Mathf.Max(minRange, t0 + Gaussian() * (noiseBase + noisePerMetre * t0));
                    break;
                }
                floor = t0 + 0.3f;
            }
            _ranges[b] = range;
        }
    }

    /// <summary>How thick the object is across the beam, in metres.</summary>
    static float Thickness(Collider c)
    {
        var s = c.transform.lossyScale;
        if (c is CapsuleCollider cap)
        {
            // radius scales with the two axes that aren't the capsule's own
            float a = cap.direction == 0 ? Mathf.Max(Mathf.Abs(s.y), Mathf.Abs(s.z))
                    : cap.direction == 1 ? Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.z))
                    : Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.y));
            return 2f * cap.radius * a;
        }
        if (c is SphereCollider sph) return 2f * sph.radius * Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.y), Mathf.Abs(s.z));
        return float.PositiveInfinity;   // boxes, planes, meshes: treat as broad surfaces
    }

    static float Gaussian()
    {
        float u1 = 1f - Random.value, u2 = Random.value;
        return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
    }

    /// <summary>"sonar" object for the FRAME header (see protocol.py).</summary>
    public string Json()
    {
        var ci = CultureInfo.InvariantCulture;
        var sb = new StringBuilder(64 + 24 * _layout.Count);
        sb.Append("{\"fov\":").Append(fovDeg.ToString("0.##", ci))
          .Append(",\"min\":").Append(minRange.ToString("0.###", ci))
          .Append(",\"max\":").Append(maxRange.ToString("0.###", ci))
          .Append(",\"beams\":[");
        for (int i = 0; i < _layout.Count; i++)
        {
            if (i > 0) sb.Append(',');
            sb.Append('[').Append(_layout[i].x.ToString("0.##", ci)).Append(',')
              .Append(_layout[i].y.ToString("0.##", ci)).Append(',')
              .Append(_ranges[i].ToString("0.###", ci)).Append(']');
        }
        sb.Append("]}");
        return sb.ToString();
    }

    // ------------------------------------------------------------- debug view
    void BuildLines()
    {
        var shader = Shader.Find("Sprites/Default");
        _lines = new LineRenderer[_layout.Count];
        for (int i = 0; i < _lines.Length; i++)
        {
            var go = new GameObject("SonarBeam" + i);
            go.transform.SetParent(transform, false);
            go.layer = DebugLayer;      // the drone's own cameras don't render this layer
            var lr = go.AddComponent<LineRenderer>();
            lr.positionCount = 2;
            lr.useWorldSpace = true;
            lr.startWidth = lr.endWidth = 0.02f;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            if (shader != null) lr.material = new Material(shader);
            _lines[i] = lr;
        }
    }

    void UpdateLines()
    {
        if (_lines == null) return;
        for (int i = 0; i < _lines.Length; i++)
        {
            var lr = _lines[i];
            lr.enabled = showBeams;
            if (!showBeams) continue;
            var dir = transform.rotation * _cones[i][0];
            bool echo = _ranges[i] > 0f;
            float len = echo ? _ranges[i] : maxRange * 0.3f;
            // never red: red is what --tracker color looks for
            var col = echo ? new Color(1f, 0.85f, 0.1f, 0.9f) : new Color(0.2f, 0.8f, 1f, 0.35f);
            lr.startColor = lr.endColor = col;
            lr.SetPosition(0, transform.position);
            lr.SetPosition(1, transform.position + dir * len);
        }
    }

    void OnDrawGizmosSelected()
    {
        if (_cones.Count == 0) return;
        Gizmos.color = new Color(1f, 0.6f, 0f, 0.5f);
        foreach (var cone in _cones)
            for (int i = 1; i < cone.Length; i += 3)
                Gizmos.DrawRay(transform.position, transform.rotation * cone[i] * maxRange);
    }
}
