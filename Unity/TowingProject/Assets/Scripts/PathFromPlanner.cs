using System.Collections;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class PathFromPlanner : MonoBehaviour
{
    [Header("Follow Targets")]
    public Transform towing;
    public Transform plane;

    [Header("World Mapping")]
    public float worldScale = 1f;
    public float yLift = 0.05f;

    [Header("Animation")]
    public float speed = 5f;
    Coroutine followCo;

    [Header("Plane follow")]
    public float planeOffset = 3.0f; // LT와 맞추기


    public PlannerClient planner;

    LineRenderer lr;

    void Awake()
    {
        lr = GetComponent<LineRenderer>();
    }

    public void Clear()
    {
        if (!lr) lr = GetComponent<LineRenderer>();
        lr.positionCount = 0;
    }

    public void Apply(PlannerClient.PlanResp resp)
    {
        if (!lr) lr = GetComponent<LineRenderer>();

        if (resp == null)
        {
            Debug.LogError("[PATH] resp=null");
            return;
        }

        if (!resp.ok || resp.x == null || resp.y == null || resp.x.Length == 0)
        {
            Debug.LogError($"[PATH] plan failed ok={resp.ok} err={resp.error}");
            Clear();
            return;
        }

        float lift = (planner != null) ? planner.yLift : yLift;
        float scale = (planner != null) ? planner.worldScale : worldScale;

        int n = Mathf.Min(resp.x.Length, resp.y.Length);
        lr.positionCount = n;

        for (int i = 0; i < n; i++)
        {
            float px = resp.x[i] * scale;
            float py = resp.y[i] * scale;
            lr.SetPosition(i, new Vector3(px, lift, py));
        }

        Debug.Log($"[PATH] ok n={n} cost={resp.cost}");

        if (followCo != null) StopCoroutine(followCo);
        followCo = StartCoroutine(CoFollow(resp));
    }

    IEnumerator CoFollow(PlannerClient.PlanResp resp)
    {
        var x = resp.x;
        var y = resp.y;
        var yaw = resp.yaw;
        var yaw1 = resp.yaw1;

        int n = x.Length;
        if (n < 2 || towing == null || plane == null) yield break;

        for (int i = 0; i < n - 1; i++)
        {
            Vector3 p0 = PlannerXYToUnity(x[i], y[i]);
            Vector3 p1 = PlannerXYToUnity(x[i + 1], y[i + 1]);

            float t0 = PlannerYawRadToUnityYawDeg((float)yaw[i]);
            float t1 = PlannerYawRadToUnityYawDeg((float)yaw[i + 1]);

            float r0 = PlannerYawRadToUnityYawDeg((float)yaw1[i]);
            float r1 = PlannerYawRadToUnityYawDeg((float)yaw1[i + 1]);

            float segLen = Vector3.Distance(p0, p1);
            float segTime = Mathf.Max(0.001f, segLen / Mathf.Max(0.01f, speed));

            float t = 0f;
            while (t < 1f)
            {
                t += Time.deltaTime / segTime;

                Vector3 pos = Vector3.Lerp(p0, p1, t);
                float towingYaw = Mathf.LerpAngle(t0, t1, t);
                float planeYaw = Mathf.LerpAngle(r0, r1, t);

                // towing position - yaw
                towing.position = pos;
                towing.rotation = Quaternion.Euler(0f, towingYaw, 0f);

                // plane rotation - yaw1
                Quaternion planeRot = Quaternion.Euler(0f, planeYaw, 0f);
                plane.rotation = planeRot;

                // plane position
                Vector3 planeForward = planeRot * Vector3.forward;
                plane.position = pos - planeForward * planeOffset;

                yield return null;
            }
        }
    }

    Vector3 PlannerXYToUnity(double px, double py)
        => new Vector3((float)px * worldScale, yLift, (float)py * worldScale);

    static float PlannerYawRadToUnityYawDeg(float yawRad)
        => 90f - yawRad * Mathf.Rad2Deg;
}
