// "Ghost" of the target: where the Python core THINKS the person is.
// Green while it sees them (should sit on the real person; the gap is the
// distance error), orange and walking on along the predicted path while they
// are hidden, with a ring on the ground = the area it would accept a re-lock in.
// V toggles. Lives on the debug layer so the drone's own cameras never film it
// (the colour tracker would lock onto it, like it did with red debug lines).
using UnityEngine;

public class GhostTarget : MonoBehaviour
{
    public bool show = true;
    [Range(0f, 1f)] public float alpha = 0.35f;
    public int ringSegments = 64;

    Transform _body;
    Renderer[] _renderers;
    // v2: fainter ghosts for the 2nd and 3rd guess of the belief (e.g. "stopped
    // behind the wall" vs "walked on"), faded by how likely they are.
    Transform[] _alts = new Transform[2];
    Material[] _altMats = new Material[2];
    LineRenderer _ring;
    Material _mat;
    Vector3 _shown;
    GUIStyle _style;
    string _state = "";
    bool _visible;

    void Start()
    {
        var shader = Shader.Find("Sprites/Default");   // unlit, supports alpha in every pipeline
        _mat = new Material(shader);
        _body = new GameObject("GhostBody").transform;
        _body.SetParent(transform, false);
        var torso = Part(PrimitiveType.Capsule, new Vector3(0, 0.9f, 0), new Vector3(0.55f, 0.9f, 0.55f));
        var head = Part(PrimitiveType.Sphere, new Vector3(0, 1.95f, 0), Vector3.one * 0.3f);
        _renderers = new[] { torso, head };

        for (int i = 0; i < _alts.Length; i++)
        {
            _altMats[i] = new Material(shader);
            var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            var col = go.GetComponent<Collider>();
            if (col != null) Destroy(col);
            go.name = "GhostAlt" + (i + 2);
            go.layer = SonarArray.DebugLayer;
            go.transform.SetParent(transform, false);
            go.transform.localScale = new Vector3(0.45f, 0.85f, 0.45f);
            var r = go.GetComponent<Renderer>();
            r.sharedMaterial = _altMats[i];
            r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            r.receiveShadows = false;
            go.SetActive(false);
            _alts[i] = go.transform;
        }

        var ringGo = new GameObject("GhostRing");
        ringGo.transform.SetParent(transform, false);
        ringGo.layer = SonarArray.DebugLayer;
        _ring = ringGo.AddComponent<LineRenderer>();
        _ring.loop = true;
        _ring.positionCount = ringSegments;
        _ring.useWorldSpace = true;
        _ring.startWidth = _ring.endWidth = 0.06f;
        _ring.material = _mat;
        _ring.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        _ring.receiveShadows = false;
        gameObject.layer = SonarArray.DebugLayer;
    }

    Renderer Part(PrimitiveType type, Vector3 pos, Vector3 scale)
    {
        var go = GameObject.CreatePrimitive(type);
        var col = go.GetComponent<Collider>();
        if (col != null) Destroy(col);                  // no physics: not an obstacle, invisible to rays
        go.layer = SonarArray.DebugLayer;
        go.transform.SetParent(_body, false);
        go.transform.localPosition = pos;
        go.transform.localScale = scale;
        var r = go.GetComponent<Renderer>();
        r.sharedMaterial = _mat;
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        r.receiveShadows = false;
        return r;
    }

    static Color StateColor(string s)
    {
        switch (s)
        {
            case "TRACKING": case "COASTING": return new Color(0.2f, 1f, 0.3f);
            case "TURNING": return new Color(0.2f, 0.9f, 1f);
            case "CONFIRMING": return new Color(0.6f, 1f, 0.9f);
            case "GAVE_UP": case "IDLE": return new Color(0.6f, 0.6f, 0.6f);
            default: return new Color(1f, 0.55f, 0.1f);  // REACQUIRING, SEARCHING, LOITER: a guess
        }
    }

    void Update()
    {
        if (InputCompat.Down(KeyCode.V)) show = !show;
        var bridge = PythonBridge.Instance;
        _visible = show && bridge != null && bridge.TryGetEstimate(out var est, out _state);
        _body.gameObject.SetActive(_visible);
        _ring.enabled = _visible;
        var modes = _visible ? bridge.GetEstimateModes() : null;
        for (int i = 0; i < _alts.Length; i++)
        {
            bool on = modes != null && modes.Length > i + 1 && modes[0].z > 0f;
            _alts[i].gameObject.SetActive(on);
            if (!on) continue;
            var m = modes[i + 1];
            _alts[i].position = new Vector3(m.x, 0.85f, m.y);
            var sc = StateColor(_state);
            _altMats[i].color = new Color(sc.r, sc.g, sc.b, alpha * Mathf.Clamp01(m.z / modes[0].z) * 0.8f);
        }
        if (!_visible) return;
        bridge.TryGetEstimate(out est, out _state);
        var target = new Vector3(est.x, 0f, est.y);
        float k = 1f - Mathf.Exp(-12f * Time.deltaTime);     // smooth the 15 Hz updates
        _shown = (_shown - target).sqrMagnitude > 25f ? target : Vector3.Lerp(_shown, target, k);
        _body.position = _shown;
        float h = Mathf.Max(1.6f, est.z);
        _body.localScale = new Vector3(1f, h / 1.95f * 0.95f, 1f);
        var c = StateColor(_state);
        _mat.color = new Color(c.r, c.g, c.b, alpha);
        _ring.startColor = _ring.endColor = new Color(c.r, c.g, c.b, 0.8f);
        float r = Mathf.Max(0.3f, est.w);
        for (int i = 0; i < ringSegments; i++)
        {
            float a = 2f * Mathf.PI * i / ringSegments;
            _ring.SetPosition(i, _shown + new Vector3(Mathf.Cos(a) * r, 0.05f, Mathf.Sin(a) * r));
        }
    }

    void OnGUI()
    {
        if (!_visible) return;
        var cam = Camera.main;
        if (cam == null) return;
        var sp = cam.WorldToScreenPoint(_shown + Vector3.up * 2.4f);
        if (sp.z <= 0) return;
        if (_style == null)
            _style = new GUIStyle(GUI.skin.label) { fontSize = 12, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter };
        _style.normal.textColor = StateColor(_state);
        GUI.Label(new Rect(sp.x - 80, Screen.height - sp.y - 10, 160, 20), "drone thinks: " + _state, _style);
    }
}
