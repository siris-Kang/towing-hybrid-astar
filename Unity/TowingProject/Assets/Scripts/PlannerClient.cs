using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Networking;

public class PlannerClient : MonoBehaviour
{
    [Header("Server")]
    public string baseUrl = "http://127.0.0.1:8080";
    public float timeoutSec = 120f;

    [Header("World Mapping")]
    public float worldScale = 1f;
    public float yLift = 0.05f;

    [Header("Obstacle Sampling")]
    public string obstacleTag = "Obstacle";
    public float sampleStep = 0.5f;
    public bool includeAllIfNoTag = true;

    [Header("Planning Grid")]
    public float xyreso = 2.0f;
    public float yawresoDeg = 15.0f;

    [Serializable]
    public class PlanReq
    {
        public float sx, sy, syaw, styaw;
        public float gx, gy, gyaw, gtyaw;
        public float xyreso, yawreso;
        public float[] ox;
        public float[] oy;
    }

    [Serializable]
    public class PlanResp
    {
        public bool ok;
        public string error;
        public int length;
        public float[] x;
        public float[] y;
        public float[] yaw;
        public float[] yaw1;
        public int[] direction;
        public float cost;
    }
    public bool Busy { get; private set; }
    public void RequestPlan(
        Transform startTowing,
        Transform startPlane,
        Vector3 goalWorld,
        float goalYawUnityDeg,
        float goalPlaneYawUnityDeg,
        Action<PlanResp> onDone)
    {
        if (Busy) return;
        StartCoroutine(CoPlan(startTowing, startPlane, goalWorld, goalYawUnityDeg, goalPlaneYawUnityDeg, onDone));
    }

    IEnumerator CoPlan(
        Transform startTowing,
        Transform startPlane,
        Vector3 goalWorld,
        float goalYawUnityDeg,
        float goalPlaneYawUnityDeg,
        Action<PlanResp> onDone)
    {
        Busy = true;

        // start pose
        Vector3 sW = startTowing.position;
        float syaw = UnityYawDegToPlannerRad(startTowing.eulerAngles.y);
        float styaw = (startPlane != null) ? UnityYawDegToPlannerRad(startPlane.eulerAngles.y) : syaw;

        // goal pose
        float gyaw = UnityYawDegToPlannerRad(goalYawUnityDeg);
        float gtyaw = UnityYawDegToPlannerRad(goalPlaneYawUnityDeg);

        float sx = sW.x / worldScale;
        float sy = sW.z / worldScale;

        float gx = goalWorld.x / worldScale;
        float gy = goalWorld.z / worldScale;

        // obstacles
        var oxList = new List<float>();
        var oyList = new List<float>();
        CollectObstaclePoints(oxList, oyList);

        var reqObj = new PlanReq
        {
            sx = sx,
            sy = sy,
            syaw = syaw,
            styaw = styaw,
            gx = gx,
            gy = gy,
            gyaw = gyaw,
            gtyaw = gtyaw,
            xyreso = xyreso,
            yawreso = Mathf.Deg2Rad * yawresoDeg,
            ox = oxList.ToArray(),
            oy = oyList.ToArray(),
        };
        string url = $"{baseUrl}/plan";
        string json = JsonUtility.ToJson(reqObj);
        byte[] body = Encoding.UTF8.GetBytes(json);

        using (UnityWebRequest req = new UnityWebRequest(url, UnityWebRequest.kHttpVerbPOST))
        {
            req.timeout = Mathf.CeilToInt(timeoutSec);
            req.uploadHandler = new UploadHandlerRaw(body);
            req.downloadHandler = new DownloadHandlerBuffer();
            req.SetRequestHeader("Content-Type", "application/json");

            float t0 = Time.realtimeSinceStartup;
            yield return req.SendWebRequest();
            float dt = Time.realtimeSinceStartup - t0;

            if (req.result != UnityWebRequest.Result.Success)
            {
                string respText = "";
                try { respText = req.downloadHandler != null ? req.downloadHandler.text : ""; } catch { }

                Debug.LogError(
                    $"[PLAN] failed: result={req.result} err={req.error} code={req.responseCode} dt={dt:F2}s " +
                    $"url={url} upBytes={body.Length} dlBytes={req.downloadedBytes}\n{respText}"
                );

                Busy = false;
                onDone?.Invoke(new PlanResp { ok = false, error = req.error });
                yield break;
            }

            PlanResp resp = null;
            string text = req.downloadHandler.text;
            try
            {
                resp = JsonUtility.FromJson<PlanResp>(text);
            }
            catch (Exception e)
            {
                Debug.LogError($"[PLAN] parse fail: {e}\n{text}");
                resp = new PlanResp { ok = false, error = "parse fail" };
            }

            Busy = false;
            onDone?.Invoke(resp);
        }

    }

    // Planner yaw: 0 -> +X, +pi/2 -> +Y
    // Unity yaw: 0 -> +Z, +90 -> +X
    // match heading: plannerYaw = pi/2 - unityYaw
    static float UnityYawDegToPlannerRad(float unityYawDeg)
    {
        float u = unityYawDeg * Mathf.Deg2Rad;
        float p = (Mathf.PI / 2f) - u;
        return WrapPi(p);
    }

    static float WrapPi(float a)
    {
        while (a > Mathf.PI) a -= 2f * Mathf.PI;
        while (a < -Mathf.PI) a += 2f * Mathf.PI;
        return a;
    }

    void CollectObstaclePoints(List<float> ox, List<float> oy)
    {
        GameObject[] obs = GameObject.FindGameObjectsWithTag(obstacleTag);

        if ((obs == null || obs.Length == 0) && includeAllIfNoTag)
        {
            var all = GameObject.FindObjectsOfType<BoxCollider>();
            var tmp = new List<GameObject>();
            foreach (var bc in all)
            {
                if (!bc.enabled) continue;
                if (bc.isTrigger) continue;
                if (bc.gameObject.name.ToLower().Contains("plane")) continue;
                tmp.Add(bc.gameObject);
            }
            obs = tmp.ToArray();
        }

        foreach (var go in obs)
        {
            if (go == null) continue;
            var bc = go.GetComponent<BoxCollider>();
            if (bc == null || !bc.enabled || bc.isTrigger) continue;

            SampleBoxColliderEdgesWorld(bc, sampleStep, ox, oy);
        }
    }

    static void SampleBoxColliderEdgesWorld(BoxCollider bc, float step, List<float> ox, List<float> oy)
    {
        Transform t = bc.transform;

        Vector3 c = bc.center;
        Vector3 s = bc.size * 0.5f;

        Vector3[] local = new Vector3[4]
        {
            c + new Vector3(-s.x, 0, -s.z),
            c + new Vector3(-s.x, 0, +s.z),
            c + new Vector3(+s.x, 0, +s.z),
            c + new Vector3(+s.x, 0, -s.z),
        };

        Vector3[] world = new Vector3[4];
        for (int i = 0; i < 4; i++) world[i] = t.TransformPoint(local[i]);

        for (int e = 0; e < 4; e++)
        {
            Vector3 a = world[e];
            Vector3 b = world[(e + 1) % 4];
            float len = Vector3.Distance(a, b);
            int n = Mathf.Max(2, Mathf.CeilToInt(len / Mathf.Max(0.01f, step)));

            for (int i = 0; i < n; i++)
            {
                float tt = i / (float)(n - 1);
                Vector3 p = Vector3.Lerp(a, b, tt);
                ox.Add(p.x);
                oy.Add(p.z);
            }
        }
    }
}
