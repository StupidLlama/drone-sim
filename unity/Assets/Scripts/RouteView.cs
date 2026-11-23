// v3: the scan route the drone flies while the target is hidden (Evan's idea A).
// A line through the waypoints (dim = done, bright = still to fly), a ball on
// the next one, and a label with what the drone thinks the target vanished
// behind (WALL / HALL / ROOM) and how sure it is. Before it is sure (under
// 80%) only the label shows, grey, with its current best guess.
// Z toggles. On the debug layer, so the drone's own cameras never film it.
using UnityEngine;

public class RouteView : MonoBehaviour
{
    public bool show = true;
    public float width = 0.08f;

    LineRenderer _done, _todo;
    Transform _next;
    Material _mat, _ballMat;
    GUIStyle _style;
    PythonBridge.RouteInfo _info;
    bool _visible;

    static Color KindColor(string k)
    {
        switch (k)
        {
            case "wall": return new Color(1f, 0.85f, 0.2f);
            case "hall": return new Color(0.3f, 0.8f, 1f);
            case "room": return new Color(1f, 0.4f, 0.8f);
            default: return new Color(0.7f, 0.7f, 0.7f);
        }
    }

    LineRenderer Line(string name)
    {
        var go = new GameObject(name);
        go.transform.SetParent(transform, false);
        go.layer = SonarArray.DebugLayer;
        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;
        lr.startWidth = lr.endWidth = width;
        lr.material = _mat;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
        lr.positionCount = 0;
        return lr;
    }

    void Start()
    {
        var shader = Shader.Find("Sprites/Default");
        _mat = new Material(shader);
        _ballMat = new Material(shader);
        gameObject.layer = SonarArray.DebugLayer;
        _done = Line("RouteDone");
        _todo = Line("RouteTodo");
        var ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        var col = ball.GetComponent<Collider>();
        if (col != null) Destroy(col);
        ball.name = "RouteNext";
        ball.layer = SonarArray.DebugLayer;
        ball.transform.SetParent(transform, false);
        ball.transform.localScale = Vector3.one * 0.35f;
        var r = ball.GetComponent<Renderer>();
        r.sharedMaterial = _ballMat;
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        r.receiveShadows = false;
        _next = ball.transform;
    }

    void Update()
    {
        if (InputCompat.Down(KeyCode.Z)) show = !show;
        var bridge = PythonBridge.Instance;
        _visible = show && bridge != null && bridge.TryGetEstimate(out _, out _);
        _info = _visible ? bridge.GetRoute() : default;
        bool hasRoute = _visible && _info.active && _info.pts != null && _info.pts.Length > 0;
        _visible = _visible && !string.IsNullOrEmpty(_info.kind);
        _done.enabled = _todo.enabled = hasRoute;
        _next.gameObject.SetActive(hasRoute);
        if (!hasRoute) return;

        var c = KindColor(_info.kind);
        var pts = _info.pts;
        int next = Mathf.Clamp(_info.next, 0, pts.Length - 1);
        // done part: from the first waypoint up to the next one
        _done.positionCount = next + 1;
        for (int i = 0; i <= next; i++) _done.SetPosition(i, pts[i]);
        _done.startColor = _done.endColor = new Color(c.r, c.g, c.b, 0.25f);
        // still to fly: from the drone to the next waypoint, then the rest
        var d = FindDrone();
        int n = pts.Length - next + (d != null ? 1 : 0);
        _todo.positionCount = n;
        int j = 0;
        if (d != null) _todo.SetPosition(j++, d.position);
        for (int i = next; i < pts.Length; i++) _todo.SetPosition(j++, pts[i]);
        _todo.startColor = _todo.endColor = new Color(c.r, c.g, c.b, 0.9f);
        _next.position = pts[next];
        _ballMat.color = new Color(c.r, c.g, c.b, 0.8f);
    }

    [Tooltip("Set by SceneBootstrap: the line starts at the drone")]
    public Transform drone;
    Transform FindDrone() => drone;

    void OnGUI()
    {
        if (!_visible) return;
        var cam = Camera.main;
        if (cam == null) return;
        Vector3 anchor;
        if (_info.pts != null && _info.pts.Length > 0 && _info.active)
            anchor = _info.pts[Mathf.Clamp(_info.next, 0, _info.pts.Length - 1)] + Vector3.up * 0.6f;
        else if (FindDrone() != null)
            anchor = drone.position + Vector3.up * 1.2f;
        else return;
        var sp = cam.WorldToScreenPoint(anchor);
        if (sp.z <= 0) return;
        if (_style == null)
            _style = new GUIStyle(GUI.skin.label) { fontSize = 12, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter };
        _style.normal.textColor = _info.active ? KindColor(_info.kind) : new Color(0.75f, 0.75f, 0.75f);
        string txt = (_info.active ? "scanning " : "maybe ") + _info.kind.ToUpper() + " " + Mathf.RoundToInt(_info.cert * 100f) + "%";
        GUI.Label(new Rect(sp.x - 90, Screen.height - sp.y - 10, 180, 20), txt, _style);
    }
}
