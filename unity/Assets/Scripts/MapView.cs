// Draws the Python core's 3-D obstacle map (sonar + depth voxels) in the scene:
// one small block per occupied voxel, coloured by height (blue at the ground ->
// red 6 m up). Python sends it twice a second (MSG_MAP). X toggles.
// Debug layer: the drone's own cameras never see it, and it has no colliders,
// so it can't fool the tracker, the depth rays or the sonar.
using UnityEngine;
using UnityEngine.Rendering;

public class MapView : MonoBehaviour
{
    public bool show = true;
    [Range(0.3f, 1f)] public float blockScale = 0.8f;   // fraction of the voxel size, so blocks don't merge
    [Range(0f, 1f)] public float alpha = 0.75f;
    public float colorTopM = 6f;

    Mesh _mesh;
    MeshRenderer _renderer;
    int _count;

    static readonly Vector3[] CubeCorners =
    {
        new Vector3(-1,-1,-1), new Vector3(1,-1,-1), new Vector3(1,1,-1), new Vector3(-1,1,-1),
        new Vector3(-1,-1,1), new Vector3(1,-1,1), new Vector3(1,1,1), new Vector3(-1,1,1),
    };
    static readonly int[] CubeTris =
    {
        0,2,1, 0,3,2,  4,5,6, 4,6,7,  0,1,5, 0,5,4,  3,7,6, 3,6,2,  0,4,7, 0,7,3,  1,2,6, 1,6,5,
    };

    void Start()
    {
        gameObject.layer = SonarArray.DebugLayer;
        _mesh = new Mesh { indexFormat = IndexFormat.UInt32, name = "VoxelMap" };
        _mesh.MarkDynamic();
        gameObject.AddComponent<MeshFilter>().sharedMesh = _mesh;
        _renderer = gameObject.AddComponent<MeshRenderer>();
        _renderer.sharedMaterial = new Material(Shader.Find("Sprites/Default"));   // unlit, uses vertex colours
        _renderer.shadowCastingMode = ShadowCastingMode.Off;
        _renderer.receiveShadows = false;
    }

    void Update()
    {
        if (InputCompat.Down(KeyCode.X)) show = !show;
        _renderer.enabled = show;
        var bridge = PythonBridge.Instance;
        if (bridge == null) return;
        var data = bridge.TakeMap();
        if (data == null || data.Length < 8) return;
        Rebuild(data);
    }

    void Rebuild(byte[] d)
    {
        int n = (int)System.BitConverter.ToUInt32(d, 0);
        float res = System.BitConverter.ToSingle(d, 4);
        n = Mathf.Min(n, (d.Length - 8) / 6);
        if (!System.BitConverter.IsLittleEndian) { _mesh.Clear(); return; }   // all target PCs are LE
        float h = 0.5f * res * blockScale;
        var verts = new Vector3[n * 8];
        var cols = new Color32[n * 8];
        var tris = new int[n * 36];
        for (int i = 0; i < n; i++)
        {
            int o = 8 + 6 * i;
            float x = System.BitConverter.ToInt16(d, o) * 0.01f;
            float y = System.BitConverter.ToInt16(d, o + 2) * 0.01f;     // core north -> Unity z
            float z = System.BitConverter.ToInt16(d, o + 4) * 0.01f;     // core up    -> Unity y
            var c = new Vector3(x, z, y);
            Color col = Color.HSVToRGB(0.66f * (1f - Mathf.Clamp01(z / colorTopM)), 0.85f, 1f);
            col.a = alpha;
            Color32 c32 = col;
            int v0 = i * 8;
            for (int k = 0; k < 8; k++)
            {
                verts[v0 + k] = c + CubeCorners[k] * h;
                cols[v0 + k] = c32;
            }
            int t0 = i * 36;
            for (int k = 0; k < 36; k++) tris[t0 + k] = v0 + CubeTris[k];
        }
        _mesh.Clear();
        _mesh.vertices = verts;
        _mesh.colors32 = cols;
        _mesh.triangles = tris;
        _mesh.RecalculateBounds();
        _count = n;
    }

    public int BlockCount => _count;
}
