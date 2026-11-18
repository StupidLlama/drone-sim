// Builds the whole test world when you press Play: ground, obstacles, the
// drone (with streaming camera), the controllable target, wind, bridge, HUD
// and operator camera. Put this on one empty GameObject in an empty scene.
// Everything is regenerated each Play, so tweak it through the fields below.
using UnityEngine;
using UnityEngine.Rendering;

public class SceneBootstrap : MonoBehaviour
{
    [Header("World")]
    public int obstacleCount = 25;
    public int seed = 42;
    public float worldSize = 200f;

    [Header("Branches (sonar test)")]
    [Tooltip("Trees with thin side branches at flight height. The 32x24 camera depth mostly misses them; the sonar hears them.")]
    public int branchyTreeCount = 10;
    [Tooltip("A branch across the target's starting path at drone height, so the follow run has to deal with it.")]
    public bool branchAcrossPath = true;
    public bool sonar = true;

    [Header("Drone")]
    public Vector3 droneStart = new Vector3(0, 3, 0);
    public float cameraPitchDeg = 20f;            // down-tilt, matches the mock sim
    public float cameraVerticalFov = 73.7f;       // = 90 deg horizontal at 4:3
    public int streamWidth = 640, streamHeight = 480;
    public float streamFps = 15f;
    [Tooltip("Front/right/back/left cameras like the real drone. Off = front camera only.")]
    public bool fourCameras = true;

    [Header("Target")]
    public Vector3 targetStart = new Vector3(0, 0, 10);
    [Tooltip("Optional: a rigged humanoid (e.g. Mixamo / Starter Assets) for YOLO. " +
             "Leave empty for the red blocky person (works with --tracker color).")]
    public GameObject targetVisualPrefab;
    [Tooltip("Extra people wandering around, to test that the drone keeps its lock on the right one")]
    public int decoyCount = 2;

    [Header("Bridge")]
    public int port = 5555;
    public bool windOnAtStart = false;

    Shader _shader;

    void Awake()
    {
        _shader = PickShader();
        Random.InitState(seed);

        CreateBridge(port);

        var wind = new GameObject("Wind").AddComponent<WindField>();
        wind.windEnabled = windOnAtStart;

        EnsureLight();
        BuildGround();
        var target = BuildPerson("Target", targetStart, new Color(0.85f, 0.08f, 0.08f));
        var shirts = new[] { new Color(0.1f, 0.3f, 0.8f), new Color(0.1f, 0.55f, 0.2f), new Color(0.9f, 0.8f, 0.1f), Color.white };
        for (int i = 0; i < decoyCount; i++)
        {
            var pos = targetStart + new Vector3(i % 2 == 0 ? -6f - i : 6f + i, 0, 4f + 2f * i);
            var d = BuildPerson("Decoy " + (i + 1), pos, shirts[i % shirts.Length]);
            d.acceptInput = false;
            d.mode = TargetController.Mode.Wander;
            d.wanderRadius = 12f;
        }
        BuildObstacles();
        BuildBranches();
        var drone = BuildDrone();

        var cam = Camera.main;
        if (cam == null)
        {
            var go = new GameObject("Main Camera") { tag = "MainCamera" };
            cam = go.AddComponent<Camera>();
            go.AddComponent<AudioListener>();
        }
        cam.transform.position = targetStart + new Vector3(0, 4, -7);
        var chase = cam.gameObject.AddComponent<ChaseCamera>();
        chase.target = target.transform;
        chase.drone = drone.transform;

        new GameObject("GhostTarget").AddComponent<GhostTarget>();   // where Python thinks the target is
        new GameObject("MapView").AddComponent<MapView>();           // Python's 3-D obstacle map
        new GameObject("RouteView").AddComponent<RouteView>().drone = drone.transform;   // v3: scene + scan route while hidden

        var hud = new GameObject("HUD").AddComponent<SimHUD>();
        hud.drone = drone;
        hud.target = target;
    }

    static PythonBridge CreateBridge(int port)
    {
        // Add disabled so we can set the port before Awake opens the socket.
        var go = new GameObject("PythonBridge");
        go.SetActive(false);
        var b = go.AddComponent<PythonBridge>();
        b.port = port;
        go.SetActive(true);
        return b;
    }

    // ---------------------------------------------------------------- builders
    DroneController BuildDrone()
    {
        var root = new GameObject("Drone");
        root.transform.position = droneStart;
        var col = root.AddComponent<BoxCollider>();
        col.size = new Vector3(0.5f, 0.15f, 0.5f);

        var visual = new GameObject("Visual").transform;
        visual.SetParent(root.transform, false);
        Prim(PrimitiveType.Cube, visual, new Vector3(0, 0, 0), new Vector3(0.22f, 0.08f, 0.3f), new Color(0.15f, 0.15f, 0.17f));
        for (int i = 0; i < 4; i++)
        {
            float a = (45 + 90 * i) * Mathf.Deg2Rad;
            var p = new Vector3(Mathf.Sin(a), 0, Mathf.Cos(a)) * 0.25f;
            var arm = Prim(PrimitiveType.Cube, visual, p * 0.5f, new Vector3(0.03f, 0.02f, 0.25f), new Color(0.2f, 0.2f, 0.2f));
            arm.transform.localRotation = Quaternion.Euler(0, 45 + 90 * i, 0);
            Prim(PrimitiveType.Cylinder, visual, p + Vector3.up * 0.03f, new Vector3(0.18f, 0.005f, 0.18f),
                 i < 2 ? new Color(0.9f, 0.9f, 0.9f) : new Color(0.1f, 0.6f, 1f));
        }

        // Rigidbody comes from [RequireComponent]; Awake records spawn pose.
        var drone = root.AddComponent<DroneController>();
        drone.visual = visual;
        root.GetComponent<Rigidbody>().mass = 1.2f;

        // Four cameras on the root (not on Visual) = ideal gimbal: they yaw with
        // the drone but don't shake with its tilt. Front, right, back, left.
        var streamer = root.AddComponent<CameraStreamer>();
        streamer.width = streamWidth;
        streamer.height = streamHeight;
        streamer.fps = streamFps;
        streamer.drone = drone;
        string[] names = { "front", "right", "back", "left" };
        float[] yaws = { 0f, 90f, 180f, -90f };
        for (int i = 0; i < names.Length; i++)
        {
            if (!fourCameras && i > 0) break;
            var camGo = new GameObject("Cam_" + names[i]);
            camGo.transform.SetParent(root.transform, false);
            var dir = Quaternion.Euler(0, yaws[i], 0);
            camGo.transform.localPosition = dir * new Vector3(0, -0.06f, 0.3f);
            camGo.transform.localRotation = dir * Quaternion.Euler(cameraPitchDeg, 0, 0);
            var cam = camGo.AddComponent<Camera>();
            cam.fieldOfView = cameraVerticalFov;
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = 300f;
            streamer.cameras.Add(new CameraStreamer.RigCamera { name = names[i], camera = cam, yawDeg = yaws[i] });
        }
        if (sonar) root.AddComponent<SonarArray>();
        return drone;
    }

    void BuildBranches()
    {
        var parent = new GameObject("Branches").transform;
        var bark = new Color(0.33f, 0.29f, 0.25f);   // grey-brown: far from the target's red
        if (branchAcrossPath)
        {
            // 6 cm branch, 3 m up, across the path straight ahead of the target
            var mid = targetStart + new Vector3(0, 3f, 10f);
            Branch(parent, mid + Vector3.left * 6f, mid + Vector3.right * 6f, 0.03f, bark);
            // posts holding it up, so it reads as a fallen branch / bar
            Prim(PrimitiveType.Cylinder, parent, mid + Vector3.left * 6f + Vector3.down * 1.5f, new Vector3(0.2f, 1.5f, 0.2f), bark, true);
            Prim(PrimitiveType.Cylinder, parent, mid + Vector3.right * 6f + Vector3.down * 1.5f, new Vector3(0.2f, 1.5f, 0.2f), bark, true);
        }
        Vector3 keepClear = (droneStart + targetStart) * 0.5f;
        int placed = 0, tries = 0;
        while (placed < branchyTreeCount && tries++ < branchyTreeCount * 20)
        {
            var p = new Vector3(Random.Range(-35f, 35f), 0, Random.Range(-25f, 45f));
            if (Vector3.Distance(new Vector3(p.x, 0, p.z), new Vector3(keepClear.x, 0, keepClear.z)) < 8f) continue;
            float h = Random.Range(5f, 7f);
            Prim(PrimitiveType.Cylinder, parent, p + Vector3.up * h / 2, new Vector3(0.3f, h / 2, 0.3f), bark, true);
            int n = Random.Range(4, 8);
            for (int i = 0; i < n; i++)
            {
                // side branches 2-5 m up, 2-4 m long, 2-8 cm thick, angled slightly up
                float y = Random.Range(2f, 5f), len = Random.Range(2f, 4f), yaw = Random.Range(0f, 360f);
                var dir = Quaternion.Euler(-Random.Range(5f, 25f), yaw, 0) * Vector3.forward;
                var a = p + Vector3.up * y;
                Branch(parent, a, a + dir * len, Random.Range(0.01f, 0.04f), bark);
                // a few twigs off the end: too thin for the sonar to hear
                var tip = a + dir * len;
                Branch(parent, tip, tip + Quaternion.Euler(Random.Range(-40f, 40f), Random.Range(-60f, 60f), 0) * dir * 0.8f, 0.004f, bark);
            }
            placed++;
        }
    }

    /// <summary>A thin cylinder from a to b (CapsuleCollider, so the sonar knows how thick it is).</summary>
    void Branch(Transform parent, Vector3 a, Vector3 b, float radius, Color c)
    {
        var len = Vector3.Distance(a, b);
        var go = Prim(PrimitiveType.Cylinder, parent, (a + b) * 0.5f, new Vector3(radius * 2f, len / 2f, radius * 2f), c, true);
        go.transform.up = (b - a).normalized;
        go.name = "Branch";
    }

    TargetController BuildPerson(string name, Vector3 position, Color shirt)
    {
        var root = new GameObject(name);
        root.transform.position = position;
        var tc = root.AddComponent<TargetController>();   // adds CharacterController
        var cc = root.GetComponent<CharacterController>();
        cc.height = 1.8f;
        cc.radius = 0.3f;
        cc.center = new Vector3(0, 0.9f, 0);

        if (targetVisualPrefab != null)
        {
            Instantiate(targetVisualPrefab, root.transform, false).name = "Visual";
            return tc;
        }

        var body = new GameObject("Visual").transform;
        body.SetParent(root.transform, false);
        var red = shirt;   // the real target is pure red so --tracker color finds only it
        var jeans = new Color(0.12f, 0.15f, 0.35f);
        var skin = new Color(0.9f, 0.72f, 0.58f);
        Prim(PrimitiveType.Cube, body, new Vector3(0, 1.22f, 0), new Vector3(0.45f, 0.62f, 0.25f), red);
        Prim(PrimitiveType.Sphere, body, new Vector3(0, 1.67f, 0), Vector3.one * 0.25f, skin);
        var walker = body.gameObject.AddComponent<BlockyWalker>();
        walker.target = tc;
        walker.leftLeg = Limb(body, new Vector3(-0.11f, 0.9f, 0), new Vector3(0.18f, 0.9f, 0.2f), jeans);
        walker.rightLeg = Limb(body, new Vector3(0.11f, 0.9f, 0), new Vector3(0.18f, 0.9f, 0.2f), jeans);
        walker.leftArm = Limb(body, new Vector3(-0.3f, 1.5f, 0), new Vector3(0.13f, 0.62f, 0.15f), red);
        walker.rightArm = Limb(body, new Vector3(0.3f, 1.5f, 0), new Vector3(0.13f, 0.62f, 0.15f), red);
        return tc;
    }

    Transform Limb(Transform parent, Vector3 pivot, Vector3 size, Color c)
    {
        var p = new GameObject("limb").transform;
        p.SetParent(parent, false);
        p.localPosition = pivot;
        Prim(PrimitiveType.Cube, p, new Vector3(0, -size.y / 2, 0), size, c);
        return p;
    }

    void BuildGround()
    {
        var g = GameObject.CreatePrimitive(PrimitiveType.Plane);
        g.name = "Ground";
        g.transform.localScale = new Vector3(worldSize / 10f, 1, worldSize / 10f);
        var tex = new Texture2D(256, 256) { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Repeat };
        for (int y = 0; y < 256; y++)
            for (int x = 0; x < 256; x++)
            {
                bool check = ((x / 32) + (y / 32)) % 2 == 0;
                float n = Random.Range(-0.03f, 0.03f);
                tex.SetPixel(x, y, check ? new Color(0.33f + n, 0.5f + n, 0.3f + n) : new Color(0.28f + n, 0.44f + n, 0.26f + n));
            }
        tex.Apply();
        var m = Mat(Color.white);
        m.mainTexture = tex;
        m.mainTextureScale = Vector2.one * worldSize / 8f;   // 4 m checker squares: gives optical-flow texture
        g.GetComponent<Renderer>().sharedMaterial = m;
    }

    void BuildObstacles()
    {
        var parent = new GameObject("Obstacles").transform;
        Vector3 keepClear = (droneStart + targetStart) * 0.5f;
        int placed = 0, tries = 0;
        while (placed < obstacleCount && tries++ < obstacleCount * 20)
        {
            var p = new Vector3(Random.Range(-40f, 40f), 0, Random.Range(-30f, 50f));
            if (Vector3.Distance(new Vector3(p.x, 0, p.z), new Vector3(keepClear.x, 0, keepClear.z)) < 10f) continue;
            int kind = Random.Range(0, 3);
            if (kind == 0) // pillar / pole
            {
                float h = Random.Range(3f, 9f), r = Random.Range(0.3f, 1f);
                Prim(PrimitiveType.Cylinder, parent, p + Vector3.up * h / 2, new Vector3(r * 2, h / 2, r * 2), new Color(0.55f, 0.55f, 0.58f), true);
            }
            else if (kind == 1) // building block
            {
                var s = new Vector3(Random.Range(2f, 8f), Random.Range(3f, 10f), Random.Range(2f, 8f));
                var b = Prim(PrimitiveType.Cube, parent, p + Vector3.up * s.y / 2, s, new Color(0.7f, 0.65f, 0.55f), true);
                b.transform.rotation = Quaternion.Euler(0, Random.Range(0, 90f), 0);
            }
            else // tree
            {
                float h = Random.Range(2f, 4f);
                Prim(PrimitiveType.Cylinder, parent, p + Vector3.up * h / 2, new Vector3(0.35f, h / 2, 0.35f), new Color(0.4f, 0.27f, 0.15f), true);
                float c = Random.Range(2f, 4f);
                Prim(PrimitiveType.Sphere, parent, p + Vector3.up * (h + c * 0.35f), Vector3.one * c, new Color(0.15f, 0.4f, 0.15f), true);
            }
            placed++;
        }
    }

    static void EnsureLight()
    {
        foreach (var l in FindObjectsByType<Light>(FindObjectsSortMode.None))
            if (l.type == LightType.Directional) return;
        var go = new GameObject("Sun");
        var light = go.AddComponent<Light>();
        light.type = LightType.Directional;
        light.shadows = LightShadows.Soft;
        light.intensity = 1.1f;
        go.transform.rotation = Quaternion.Euler(50, -30, 0);
    }

    // ---------------------------------------------------------------- helpers
    GameObject Prim(PrimitiveType type, Transform parent, Vector3 localPos, Vector3 scale, Color c, bool keepCollider = false)
    {
        var go = GameObject.CreatePrimitive(type);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localPos;
        go.transform.localScale = scale;
        go.GetComponent<Renderer>().sharedMaterial = Mat(c);
        if (!keepCollider) DestroyImmediate(go.GetComponent<Collider>());
        return go;
    }

    Material Mat(Color c)
    {
        var m = new Material(_shader) { color = c };
        return m;
    }

    static Shader PickShader()
    {
        var rp = GraphicsSettings.currentRenderPipeline;
        Shader s = null;
        if (rp != null)
            s = Shader.Find(rp.GetType().Name.Contains("HD") ? "HDRP/Lit" : "Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Unlit/Color");
        return s;
    }
}
