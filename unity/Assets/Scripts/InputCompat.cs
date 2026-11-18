// Keyboard helper that works with either the legacy Input Manager or the new
// Input System package, so the scripts compile whatever "Active Input
// Handling" your project uses.
using System;
using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public static class InputCompat
{
#if ENABLE_INPUT_SYSTEM
    static readonly Dictionary<KeyCode, Key> _map = new Dictionary<KeyCode, Key>();

    static Key ToKey(KeyCode kc)
    {
        if (_map.TryGetValue(kc, out var k)) return k;
        string n = kc.ToString();
        if (n.StartsWith("Alpha")) n = "Digit" + n.Substring(5);
        if (n == "Return") n = "Enter";
        if (!Enum.TryParse(n, out k)) k = Key.None;
        _map[kc] = k;
        return k;
    }

    public static bool Held(KeyCode kc)
    {
        var kb = Keyboard.current; var k = ToKey(kc);
        return kb != null && k != Key.None && kb[k].isPressed;
    }

    public static bool Down(KeyCode kc)
    {
        var kb = Keyboard.current; var k = ToKey(kc);
        return kb != null && k != Key.None && kb[k].wasPressedThisFrame;
    }
#else
    public static bool Held(KeyCode kc) => Input.GetKey(kc);
    public static bool Down(KeyCode kc) => Input.GetKeyDown(kc);
#endif

#if ENABLE_INPUT_SYSTEM
    public static bool RightMouseHeld() => Mouse.current != null && Mouse.current.rightButton.isPressed;
    public static float MouseDeltaX() => Mouse.current != null ? Mouse.current.delta.ReadValue().x : 0f;
#else
    public static bool RightMouseHeld() => Input.GetMouseButton(1);
    // legacy "Mouse X" is roughly pixels/10; scale so sensitivity matches the new Input System
    public static float MouseDeltaX() => Input.GetAxis("Mouse X") * 10f;
#endif

    public static float Axis(KeyCode negative, KeyCode positive) =>
        (Held(positive) ? 1f : 0f) - (Held(negative) ? 1f : 0f);
}
