// The person the drone follows. T cycles the mode:
//   Manual    WASD (relative to the view camera), Shift = run
//   Waypoints scripted figure-8 / your own waypoint transforms
//   Wander    random goals, random speeds, random pauses (good for soak tests)
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class TargetController : MonoBehaviour
{
    public enum Mode { Manual, Waypoints, Wander }

    public Mode mode = Mode.Manual;
    [Tooltip("Off for decoys: they ignore the keyboard and just keep their mode")]
    public bool acceptInput = true;
    public float walkSpeed = 1.5f;
    public float runSpeed = 4.5f;
    public float turnSpeedDeg = 540f;
    [Tooltip("Movement is relative to this camera (defaults to Camera.main)")]
    public Transform viewCamera;

    [Header("Waypoints mode")]
    [Tooltip("Leave empty to auto-generate a figure-8 around the spawn point")]
    public Transform[] waypoints;
    public float scriptedSpeed = 2f;
    public float figure8Size = 12f;

    [Header("Wander mode")]
    public float wanderRadius = 25f;

    public float CurrentSpeed { get; private set; }
    public Vector3 SpawnPoint => _spawn;

    CharacterController _cc;
    readonly List<Vector3> _path = new List<Vector3>();
    int _wp;
    Vector3 _spawn, _wanderGoal;
    float _vy, _wanderSpeed, _pauseUntil;

    void Awake()
    {
        _cc = GetComponent<CharacterController>();
        _spawn = transform.position;
        if (waypoints != null && waypoints.Length > 0)
            foreach (var w in waypoints) _path.Add(w.position);
        else
            for (int i = 0; i < 24; i++)
            {
                float a = i / 24f * Mathf.PI * 2f;          // lemniscate of Gerono
                _path.Add(_spawn + new Vector3(Mathf.Sin(a) * figure8Size, 0, Mathf.Sin(a) * Mathf.Cos(a) * figure8Size));
            }
        NewWanderGoal();
    }

    void Update()
    {
        if (acceptInput && InputCompat.Down(KeyCode.T)) mode = (Mode)(((int)mode + 1) % 3);
        if (InputCompat.Down(KeyCode.R)) { _cc.enabled = false; transform.position = _spawn; _cc.enabled = true; }

        Vector3 dir = Vector3.zero;
        float speed = 0f;
        switch (mode)
        {
            case Mode.Manual:
            {
                var cam = viewCamera != null ? viewCamera : (Camera.main != null ? Camera.main.transform : null);
                Vector3 fwd = cam != null ? Vector3.ProjectOnPlane(cam.forward, Vector3.up).normalized : Vector3.forward;
                Vector3 right = Vector3.Cross(Vector3.up, fwd);
                dir = fwd * InputCompat.Axis(KeyCode.S, KeyCode.W) + right * InputCompat.Axis(KeyCode.A, KeyCode.D);
                dir = Vector3.ClampMagnitude(dir, 1f);
                speed = InputCompat.Held(KeyCode.LeftShift) ? runSpeed : walkSpeed;
                break;
            }
            case Mode.Waypoints:
            {
                Vector3 to = _path[_wp] - transform.position; to.y = 0;
                if (to.magnitude < 0.6f) _wp = (_wp + 1) % _path.Count;
                dir = to.normalized;
                speed = scriptedSpeed;
                break;
            }
            case Mode.Wander:
            {
                if (Time.time < _pauseUntil) break;
                Vector3 to = _wanderGoal - transform.position; to.y = 0;
                if (to.magnitude < 0.8f)
                {
                    if (Random.value < 0.3f) _pauseUntil = Time.time + Random.Range(1f, 4f);
                    NewWanderGoal();
                }
                dir = to.normalized;
                speed = _wanderSpeed;
                break;
            }
        }

        Vector3 horizontal = dir * speed;
        CurrentSpeed = horizontal.magnitude;
        if (dir.sqrMagnitude > 0.01f)
        {
            var targetRot = Quaternion.LookRotation(dir, Vector3.up);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, turnSpeedDeg * Time.deltaTime);
        }
        _vy = _cc.isGrounded ? -1f : _vy + Physics.gravity.y * Time.deltaTime;
        var flags = _cc.Move((horizontal + Vector3.up * _vy) * Time.deltaTime);
        // stuck on an obstacle in wander mode -> pick another goal
        if (mode == Mode.Wander && (flags & CollisionFlags.Sides) != 0) NewWanderGoal();
    }

    void NewWanderGoal()
    {
        var r = Random.insideUnitCircle * wanderRadius;
        _wanderGoal = _spawn + new Vector3(r.x, 0, r.y);
        _wanderSpeed = Random.value < 0.25f ? runSpeed : Random.Range(walkSpeed * 0.7f, walkSpeed * 1.5f);
    }
}
