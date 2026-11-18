// Operator view. C cycles: behind target -> behind drone -> overhead.
// The camera keeps its OWN heading instead of copying the target's rotation,
// so turning the person with A/D no longer swings the whole view around.
// Orbit with Q/E or by holding the right mouse button and dragging.
using UnityEngine;

public class ChaseCamera : MonoBehaviour
{
    public Transform target;
    public Transform drone;
    public enum View { BehindTarget, BehindDrone, Overhead }
    public View view = View.BehindTarget;

    [Header("Behind-target view")]
    public float distance = 8f;
    public float height = 4f;
    public float orbitSpeedDeg = 90f;          // Q/E, degrees per second
    public float mouseOrbitSensitivity = 0.25f; // degrees per pixel of right-drag
    [Tooltip("0 = never auto-rotate. >0 slowly swings behind the target while it walks forward")]
    public float autoAlignDegPerSec = 0f;

    public float positionSmooth = 6f;

    float _yaw;            // camera heading (deg), independent of the target's facing
    bool _init;

    void Update()
    {
        if (InputCompat.Down(KeyCode.C)) view = (View)(((int)view + 1) % 3);
        _yaw += InputCompat.Axis(KeyCode.Q, KeyCode.E) * orbitSpeedDeg * Time.unscaledDeltaTime;
        if (InputCompat.RightMouseHeld()) _yaw += InputCompat.MouseDeltaX() * mouseOrbitSensitivity;
    }

    void LateUpdate()
    {
        if (target == null) return;
        if (!_init) { _yaw = target.eulerAngles.y; _init = true; }

        Vector3 desired, look;
        switch (view)
        {
            case View.BehindDrone:
                if (drone == null) return;
                // the drone turns smoothly, so following its heading is fine here
                desired = drone.position - drone.forward * 4f + Vector3.up * 1.5f;
                look = drone.position + drone.forward * 5f;
                break;
            case View.Overhead:
            {
                Vector3 mid = drone != null ? (drone.position + target.position) * 0.5f : target.position;
                desired = mid + new Vector3(0, 25f, -8f);
                look = mid;
                break;
            }
            default:
            {
                if (autoAlignDegPerSec > 0f)
                {
                    var tc = target.GetComponent<TargetController>();
                    if (tc != null && tc.CurrentSpeed > 0.5f)
                        _yaw = Mathf.MoveTowardsAngle(_yaw, target.eulerAngles.y, autoAlignDegPerSec * Time.deltaTime);
                }
                Vector3 back = Quaternion.Euler(0f, _yaw, 0f) * Vector3.back;
                desired = target.position + back * distance + Vector3.up * height;
                look = target.position + Vector3.up * 1f;
                break;
            }
        }
        float k = 1f - Mathf.Exp(-positionSmooth * Time.deltaTime);
        transform.position = Vector3.Lerp(transform.position, desired, k);
        // look straight at the subject (no rotation smoothing = no extra swing)
        transform.rotation = Quaternion.LookRotation(look - transform.position, Vector3.up);
    }
}
