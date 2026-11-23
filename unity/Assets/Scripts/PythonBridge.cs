// TCP server that the Python core connects to. Protocol must match
// core/drone_core/adapters/unity/protocol.py:
//   every message: uint32 BE length(type+payload) | uint8 type | payload
//   Unity->Py  FRAME(1):   uint32 BE header_len | header JSON | JPEG per camera | depth per camera
//   Py->Unity  COMMAND(2): JSON {forward,right,up (m/s), yaw_rate (rad/s, + = right),
//                               optional est:{has,x,y,h,r,state} = where Python thinks the target is,
//                               est.modes:[{x,y,w}] = v2 top guesses while hidden,
//                               est.route:{kind,cert,active,i,pts:[{x,y,z}]} = v3 scene + scan route}
//              RESET(5):   empty
//              MAP(6):     uint32 LE n | float32 LE voxel size | n x int16 LE (x,y,z cm, core frame) -> MapView
using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class PythonBridge : MonoBehaviour
{
    public const byte MSG_FRAME = 1, MSG_COMMAND = 2, MSG_RESET = 5, MSG_MAP = 6;

    public static PythonBridge Instance { get; private set; }

    [Tooltip("Port the Python core connects to (main.py --port)")]
    public int port = 5555;
    [Tooltip("Hover if no command arrives for this long (s)")]
    public float commandTimeout = 0.5f;

    public bool IsConnected => _client != null && _client.Connected;
    public int FramesSent => _framesSent;
    public bool ConsumeResetRequest() => Interlocked.Exchange(ref _resetRequested, 0) == 1;

#pragma warning disable 0649 // assigned by JsonUtility
    [Serializable] class ModeMsg { public float x, y, w; }
    [Serializable] class PtMsg { public float x, y, z; }
    [Serializable] class RouteMsg { public string kind; public float cert; public int active, i; public PtMsg[] pts; }
    [Serializable] class EstMsg { public int has; public float x, y, h, r; public string state; public ModeMsg[] modes; public RouteMsg route; }
    [Serializable] class CommandMsg { public float forward, right, up, yaw_rate; public EstMsg est; }
#pragma warning restore 0649

    TcpListener _listener;
    volatile TcpClient _client;
    volatile NetworkStream _stream;
    Thread _acceptThread, _writeThread;
    volatile bool _running;
    int _resetRequested;
    int _framesSent;

    readonly object _cmdLock = new object();
    Vector4 _cmd;                 // x=forward y=right z=up w=yawRate
    double _cmdTime = -1;
    Vector4 _est;                 // x, y (= Unity x, z), h, r of where Python thinks the target is
    string _estState = "";
    double _estTime = -1;
    // v3: the scene + scan route (route points in Unity world: x, height, z)
    public struct RouteInfo { public string kind; public float cert; public bool active; public int next; public Vector3[] pts; }
    RouteInfo _route;
    Vector3[] _modes = new Vector3[0];   // v2: other guesses (x, y = Unity x, z; z = weight), heaviest first
    readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();
    // Capacity 2: if Python is slow we drop frames instead of building latency.
    BlockingCollection<byte[]> _sendQueue = new BlockingCollection<byte[]>(2);

    void Awake()
    {
        Instance = this;
        _running = true;
        _listener = new TcpListener(IPAddress.Loopback, port);
        _listener.Start();
        _acceptThread = new Thread(AcceptLoop) { IsBackground = true, Name = "bridge-accept" };
        _acceptThread.Start();
        _writeThread = new Thread(WriteLoop) { IsBackground = true, Name = "bridge-write" };
        _writeThread.Start();
        Debug.Log($"[PythonBridge] listening on 127.0.0.1:{port}");
    }

    /// <summary>Latest command from Python. False if none / stale (caller should hover).</summary>
    public bool TryGetCommand(out Vector4 cmd)
    {
        lock (_cmdLock)
        {
            cmd = _cmd;
            return _cmdTime >= 0 && _clock.Elapsed.TotalSeconds - _cmdTime < commandTimeout;
        }
    }

    byte[] _map;                  // latest MSG_MAP payload, taken by MapView
    public byte[] TakeMap() => Interlocked.Exchange(ref _map, null);

    /// <summary>Where Python thinks the target is (for GhostTarget). False if it sent none recently.</summary>
    /// <summary>v2: the belief's top guesses while the target is hidden (x, y, weight), heaviest first. Empty otherwise.</summary>
    public Vector3[] GetEstimateModes()
    {
        lock (_cmdLock) return _modes;
    }

    /// <summary>v3: what the drone thinks the target vanished behind and the scan route it flies. pts empty = none.</summary>
    public RouteInfo GetRoute()
    {
        lock (_cmdLock) return _route;
    }

    public bool TryGetEstimate(out Vector4 est, out string state)
    {
        lock (_cmdLock)
        {
            est = _est;
            state = _estState;
            return _estTime >= 0 && _clock.Elapsed.TotalSeconds - _estTime < 1.0;
        }
    }

    /// <summary>Header JSON followed by the blobs it describes (JPEGs, then depth).</summary>
    public void SendFrame(string headerJson, System.Collections.Generic.List<byte[]> blobs)
    {
        if (!IsConnected) return;
        byte[] header = Encoding.UTF8.GetBytes(headerJson);
        int blobLen = 0;
        foreach (var b in blobs) blobLen += b.Length;
        int payloadLen = 4 + header.Length + blobLen;
        var msg = new byte[4 + 1 + payloadLen];
        WriteU32(msg, 0, (uint)(payloadLen + 1));
        msg[4] = MSG_FRAME;
        WriteU32(msg, 5, (uint)header.Length);
        Buffer.BlockCopy(header, 0, msg, 9, header.Length);
        int off = 9 + header.Length;
        foreach (var b in blobs) { Buffer.BlockCopy(b, 0, msg, off, b.Length); off += b.Length; }
        if (_sendQueue.TryAdd(msg)) Interlocked.Increment(ref _framesSent);
    }

    void AcceptLoop()
    {
        while (_running)
        {
            try
            {
                var c = _listener.AcceptTcpClient();
                c.NoDelay = true;
                _client = c;
                _stream = c.GetStream();
                Debug.Log("[PythonBridge] Python connected");
                ReadLoop(_stream);
            }
            catch (Exception e)
            {
                if (_running) Debug.Log($"[PythonBridge] connection ended: {e.Message}");
            }
            finally
            {
                lock (_cmdLock) { _cmdTime = -1; _cmd = Vector4.zero; }
                try { _client?.Close(); } catch { }
                _client = null;
                _stream = null;
            }
        }
    }

    void ReadLoop(NetworkStream s)
    {
        var hdr = new byte[5];
        while (_running)
        {
            ReadExact(s, hdr, 5);
            uint len = ReadU32(hdr, 0);
            byte type = hdr[4];
            if (len < 1 || len > 1 << 20) throw new Exception($"bad length {len}");
            var payload = new byte[len - 1];
            ReadExact(s, payload, payload.Length);

            if (type == MSG_COMMAND)
            {
                // JsonUtility is safe to call from background threads.
                var m = JsonUtility.FromJson<CommandMsg>(Encoding.UTF8.GetString(payload));
                lock (_cmdLock)
                {
                    _cmd = new Vector4(m.forward, m.right, m.up, m.yaw_rate);
                    _cmdTime = _clock.Elapsed.TotalSeconds;
                    if (m.est != null && m.est.has == 1)
                    {
                        _est = new Vector4(m.est.x, m.est.y, m.est.h, m.est.r);
                        _estState = m.est.state ?? "";
                        _estTime = _cmdTime;
                        var n = m.est.modes == null ? 0 : m.est.modes.Length;
                        var modes = new Vector3[n];
                        for (int i = 0; i < n; i++) modes[i] = new Vector3(m.est.modes[i].x, m.est.modes[i].y, m.est.modes[i].w);
                        _modes = modes;
                        var r = m.est.route;
                        var info = new RouteInfo { kind = "", pts = new Vector3[0] };
                        if (r != null && !string.IsNullOrEmpty(r.kind))
                        {
                            int np = r.pts == null ? 0 : r.pts.Length;
                            var pts = new Vector3[np];
                            for (int k = 0; k < np; k++) pts[k] = new Vector3(r.pts[k].x, r.pts[k].z, r.pts[k].y);  // core (x, y, z up) -> Unity (x, y up, z)
                            info = new RouteInfo { kind = r.kind, cert = r.cert, active = r.active == 1, next = r.i, pts = pts };
                        }
                        _route = info;
                    }
                }
            }
            else if (type == MSG_RESET)
            {
                Interlocked.Exchange(ref _resetRequested, 1);
            }
            else if (type == MSG_MAP)
            {
                Interlocked.Exchange(ref _map, payload);
            }
        }
    }

    void WriteLoop()
    {
        try
        {
            foreach (var msg in _sendQueue.GetConsumingEnumerable())
            {
                var s = _stream;
                if (s == null) continue;
                try { s.Write(msg, 0, msg.Length); }
                catch (Exception e) { Debug.Log($"[PythonBridge] write failed: {e.Message}"); }
            }
        }
        catch (InvalidOperationException) { } // queue completed on shutdown
    }

    static void ReadExact(NetworkStream s, byte[] buf, int n)
    {
        int off = 0;
        while (off < n)
        {
            int r = s.Read(buf, off, n - off);
            if (r <= 0) throw new Exception("socket closed");
            off += r;
        }
    }

    static void WriteU32(byte[] b, int o, uint v)
    {
        b[o] = (byte)(v >> 24); b[o + 1] = (byte)(v >> 16); b[o + 2] = (byte)(v >> 8); b[o + 3] = (byte)v;
    }

    static uint ReadU32(byte[] b, int o) =>
        ((uint)b[o] << 24) | ((uint)b[o + 1] << 16) | ((uint)b[o + 2] << 8) | b[o + 3];

    void OnDestroy()
    {
        _running = false;
        try { _listener?.Stop(); } catch { }
        try { _client?.Close(); } catch { }
        _sendQueue.CompleteAdding();
        if (Instance == this) Instance = null;
    }
}
