// Streams the drone's camera rig to Python: one JPEG per camera
// (front/right/back/left), a low-res ground-truth depth image per camera made
// with raycasts, and the telemetry header, all in one FRAME message.
// The real drone gets depth from a depth network instead; the Python side
// doesn't care where it comes from.
// Performance: the rig cameras render only on capture frames, pixels come back
// with AsyncGPUReadback (no GPU stall) and JPEG encoding runs on a worker thread.
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;

public class CameraStreamer : MonoBehaviour
{
    [System.Serializable]
    public class RigCamera
    {
        public string name = "front";
        public Camera camera;
        public float yawDeg;          // relative to the drone's nose, + = right
    }

    public List<RigCamera> cameras = new List<RigCamera>();
    public int width = 640;
    public int height = 480;
    public float fps = 15f;
    [Range(10, 100)] public int jpegQuality = 75;
    public DroneController drone;

    [Header("Depth (ground truth via raycasts)")]
    public bool sendDepth = true;
    public int depthWidth = 32;
    public int depthHeight = 24;
    public float depthMax = 30f;
    public LayerMask depthMask = ~0;

    [Tooltip("Show all cameras picture-in-picture (top-right). P toggles.")]
    public bool showPreview = true;

    readonly List<RenderTexture> _rts = new List<RenderTexture>();
    int _frameId;
    float _next;
    volatile bool _busy;          // a capture is being read back / encoded

    void Start()
    {
        if (drone == null) drone = GetComponentInParent<DroneController>();
        foreach (var rc in cameras)
        {
            var rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
            rt.Create();
            rc.camera.targetTexture = rt;
            // Off by default: a camera with a target texture otherwise renders
            // every editor frame. We switch them on only for frames we send.
            rc.camera.enabled = false;
            rc.camera.cullingMask &= ~(1 << SonarArray.DebugLayer);   // don't film debug lines
            _rts.Add(rt);
        }
        StartCoroutine(CaptureLoop());
    }

    void Update()
    {
        if (InputCompat.Down(KeyCode.P)) showPreview = !showPreview;
    }

    // Each capture: turn the rig cameras on for one frame, then at the end of
    // that frame ask the GPU for the pixels asynchronously (no stall), and
    // encode the JPEGs on a worker thread. The main thread only does the depth
    // raycasts and the telemetry snapshot.
    IEnumerator CaptureLoop()
    {
        var eof = new WaitForEndOfFrame();
        while (true)
        {
            yield return null;
            var bridge = PythonBridge.Instance;
            bool live = bridge != null && bridge.IsConnected;
            if (!showPreview && !live) continue;
            if (_busy || Time.unscaledTime < _next) continue;
            _next = Time.unscaledTime + 1f / fps;

            foreach (var rc in cameras) rc.camera.enabled = true;
            yield return eof;
            foreach (var rc in cameras) rc.camera.enabled = false;
            if (!live) continue;              // preview only

            _busy = true;
            _frameId++;
            int frameId = _frameId;
            string telem = drone != null ? drone.TelemetryJson(frameId, Time.timeAsDouble)
                                         : $"{{\"frame_id\":{frameId}}}";
            var depthBlobs = new List<byte[]>();
            if (sendDepth)
                for (int i = 0; i < cameras.Count; i++) depthBlobs.Add(DepthImage(cameras[i].camera));

            int n = cameras.Count;
            var raw = new byte[n][];
            int pending = n;
            bool failed = false;
            for (int i = 0; i < n; i++)
            {
                int idx = i;
                AsyncGPUReadback.Request(_rts[i], 0, TextureFormat.RGB24, req =>
                {
                    if (req.hasError) failed = true;
                    else raw[idx] = req.GetData<byte>().ToArray();
                    if (--pending == 0)
                    {
                        if (failed) { _busy = false; return; }
                        ThreadPool.QueueUserWorkItem(_ => Encode(raw, depthBlobs, telem, frameId));
                    }
                });
            }
        }
    }

    void Encode(byte[][] raw, List<byte[]> depthBlobs, string telem, int frameId)
    {
        try
        {
            var bridge = PythonBridge.Instance;
            if (bridge == null) return;
            var ci = CultureInfo.InvariantCulture;
            var blobs = new List<byte[]>();
            var cams = new StringBuilder("[");
            for (int i = 0; i < raw.Length; i++)
            {
                // Same row order as Texture2D, so the JPEG comes out upright.
                byte[] jpg = ImageConversion.EncodeArrayToJPG(raw[i], GraphicsFormat.R8G8B8_UNorm,
                                                              (uint)width, (uint)height, 0, jpegQuality);
                blobs.Add(jpg);
                if (i > 0) cams.Append(',');
                cams.Append("{\"name\":\"").Append(cameras[i].name)
                    .Append("\",\"yaw_deg\":").Append(cameras[i].yawDeg.ToString("0.###", ci))
                    .Append(",\"len\":").Append(jpg.Length).Append('}');
            }
            cams.Append(']');
            var depth = new StringBuilder("[");
            for (int i = 0; i < depthBlobs.Count; i++)
            {
                blobs.Add(depthBlobs[i]);
                if (i > 0) depth.Append(',');
                depth.Append("{\"name\":\"").Append(cameras[i].name)
                     .Append("\",\"w\":").Append(depthWidth).Append(",\"h\":").Append(depthHeight)
                     .Append(",\"len\":").Append(depthBlobs[i].Length).Append('}');
            }
            depth.Append(']');
            // splice the camera/depth tables into the telemetry object
            string header = telem.Substring(0, telem.Length - 1) + ",\"cams\":" + cams +
                            (depthBlobs.Count > 0 ? ",\"depth\":" + depth : "") + "}";
            bridge.SendFrame(header, blobs);
        }
        catch (System.Exception e) { Debug.LogWarning($"[CameraStreamer] encode failed: {e.Message}"); }
        finally { _busy = false; }
    }

    /// <summary>Distance along each pixel's ray, float16 little-endian, row 0 = top.</summary>
    byte[] DepthImage(Camera cam)
    {
        var bytes = new byte[depthWidth * depthHeight * 2];
        int k = 0;
        for (int j = 0; j < depthHeight; j++)
        {
            float vy = 1f - (j + 0.5f) / depthHeight;           // viewport y=0 is the bottom
            for (int i = 0; i < depthWidth; i++)
            {
                var ray = cam.ViewportPointToRay(new Vector3((i + 0.5f) / depthWidth, vy, 0f));
                ushort h = 0x7C00;                              // +inf = nothing hit
                if (Physics.Raycast(ray, out var hit, depthMax, depthMask, QueryTriggerInteraction.Ignore))
                    h = Mathf.FloatToHalf(hit.distance + cam.nearClipPlane);
                bytes[k++] = (byte)(h & 0xFF);
                bytes[k++] = (byte)(h >> 8);
            }
        }
        return bytes;
    }

    void OnGUI()
    {
        if (!showPreview || _rts.Count == 0) return;
        // 2 x 2 grid, top-right
        float w = Mathf.Min(Screen.width * 0.18f, width / 2f);
        float h = w * height / width;
        int cols = 2;
        for (int i = 0; i < _rts.Count; i++)
        {
            int c = i % cols, r = i / cols;
            var rect = new Rect(Screen.width - (cols - c) * (w + 6) - 4, 10 + r * (h + 6), w, h);
            GUI.DrawTexture(rect, _rts[i], ScaleMode.ScaleToFit, false);
            DrawCamLabel(rect, cameras[i]);
        }
    }

    GUIStyle _labelStyle;
    static readonly Color LabelBg = new Color(0f, 0f, 0f, 0.7f);

    /// <summary>Badge in the preview's top-left corner: arrow + name, e.g. "^ FRONT".</summary>
    void DrawCamLabel(Rect rect, RigCamera rc)
    {
        if (_labelStyle == null)
            _labelStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 14, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white }
            };
        string n = rc.name.ToLowerInvariant();
        string arrow = n == "front" ? "^" : n == "right" ? ">" : n == "back" ? "v" : n == "left" ? "<" : "";
        string text = (arrow + " " + rc.name.ToUpperInvariant()).Trim();
        var size = _labelStyle.CalcSize(new GUIContent(text));
        var bg = new Rect(rect.x, rect.y, size.x + 12, size.y + 4);
        var old = GUI.color;
        GUI.color = LabelBg;
        GUI.DrawTexture(bg, Texture2D.whiteTexture);
        GUI.color = old;
        GUI.Label(new Rect(bg.x + 6, bg.y + 2, size.x + 4, size.y), text, _labelStyle);
    }

    void OnDestroy()
    {
        foreach (var rc in cameras) if (rc.camera != null) rc.camera.targetTexture = null;
        foreach (var rt in _rts) if (rt != null) rt.Release();
    }
}
