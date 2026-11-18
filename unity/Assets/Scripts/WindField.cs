// Global wind: steady component + smooth Perlin gusts. The drone feels it
// through aerodynamic drag on its *air-relative* velocity, so the Python
// controller has to compensate (integral terms) just like outdoors.
using UnityEngine;

public class WindField : MonoBehaviour
{
    public static WindField Instance { get; private set; }

    public bool windEnabled = false;
    [Tooltip("Steady wind, world m/s (x = east/right, z = north/forward)")]
    public Vector3 baseWind = new Vector3(3f, 0f, 1f);
    [Tooltip("Peak gust deviation, m/s")]
    public float gustStrength = 2.5f;
    [Tooltip("How fast gusts change, Hz-ish")]
    public float gustFrequency = 0.25f;

    public Vector3 Current { get; private set; }

    void Awake() => Instance = this;

    void Update()
    {
        if (InputCompat.Down(KeyCode.G)) windEnabled = !windEnabled;
        if (!windEnabled) { Current = Vector3.zero; return; }
        float t = Time.time * gustFrequency;
        var gust = new Vector3(
            Mathf.PerlinNoise(t, 0.0f) - 0.5f,
            (Mathf.PerlinNoise(t, 10f) - 0.5f) * 0.3f,
            Mathf.PerlinNoise(t, 20f) - 0.5f) * 2f * gustStrength;
        Current = baseWind + gust;
    }
}
