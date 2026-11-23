// On-screen status + key help.
using UnityEngine;

public class SimHUD : MonoBehaviour
{
    public DroneController drone;
    public TargetController target;
    public bool showHelp = true;

    void Update()
    {
        if (InputCompat.Down(KeyCode.F1)) showHelp = !showHelp;
    }

    void OnGUI()
    {
        var bridge = PythonBridge.Instance;
        var wind = WindField.Instance;
        string s =
            $"Python: {(bridge != null && bridge.IsConnected ? "<color=lime>connected</color>" : "<color=orange>waiting on port " + (bridge != null ? bridge.port : 0) + "</color>")}" +
            (bridge != null ? $"   frames sent: {bridge.FramesSent}" : "") + "\n";
        if (drone != null)
        {
            var c = drone.CurrentCommand;
            s += $"Drone cmd ({drone.CommandSource}): fwd {c.x:+0.00;-0.00} right {c.y:+0.00;-0.00} up {c.z:+0.00;-0.00} yaw {c.w:+0.00;-0.00}\n";
            s += $"Alt {drone.Altitude:0.0} m   speed {drone.Velocity.magnitude:0.0} m/s   battery {drone.battery * 100f:0}%";
            if (target != null)
            {
                var d = target.transform.position - drone.transform.position; d.y = 0;
                s += $"   dist to target {d.magnitude:0.0} m";
            }
            s += "\n";
        }
        if (target != null) s += $"Target mode: {target.mode}   speed {target.CurrentSpeed:0.0} m/s\n";
        if (wind != null) s += $"Wind: {(wind.windEnabled ? wind.Current.ToString("0.0") + " m/s" : "off")}\n";
        if (showHelp)
            s += "\nTarget: WASD move, Shift run, T cycle Manual/Waypoints/Wander\n" +
                 "Drone: M manual override (I/K fwd, J/L strafe, U/O yaw, Y/H up/down)\n" +
                 "Camera: Q/E or right-drag to orbit, C change view\n" +
                 "G wind on/off   B low battery   P camera previews   N sonar beams   V ghost   Z scan route   X 3D map   R reset   F1 hide help\n" +
                 "Python window: F follow, H return home, Space hold";
        var style = new GUIStyle(GUI.skin.box) { alignment = TextAnchor.UpperLeft, richText = true, fontSize = 13 };
        GUI.Box(new Rect(10, 10, 520, showHelp ? 212 : 100), s, style);
    }
}
