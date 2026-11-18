// Procedural arm/leg swing for the placeholder "blocky person". Not needed if
// you swap in a rigged character with its own Animator.
using UnityEngine;

public class BlockyWalker : MonoBehaviour
{
    public Transform leftArm, rightArm, leftLeg, rightLeg;
    public TargetController target;
    public float stepFrequency = 1.6f;   // steps/s per m/s
    public float swingDeg = 35f;
    float _phase;

    void Update()
    {
        float speed = target != null ? target.CurrentSpeed : 0f;
        _phase += speed * stepFrequency * Time.deltaTime * Mathf.PI;
        float amount = Mathf.Clamp01(speed / 1.5f);
        float a = Mathf.Sin(_phase) * swingDeg * amount;
        Set(leftArm, a); Set(rightArm, -a); Set(leftLeg, -a); Set(rightLeg, a);
    }

    static void Set(Transform t, float deg)
    {
        if (t != null) t.localRotation = Quaternion.Euler(deg, 0, 0);
    }
}
