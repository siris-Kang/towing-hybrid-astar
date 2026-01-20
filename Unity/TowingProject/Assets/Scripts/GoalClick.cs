using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.EventSystems;

public class GoalClick : MonoBehaviour
{
    public Camera cam;
    public LayerMask groundMask = ~0;
    public Transform goalMarker;

    [Header("Planning")]
    public PlannerClient planner;
    public Transform startTowing;
    public Transform startPlane;
    public float goalYawUnityDeg = 0f;
    public float goalPlaneYawUnityDeg = 0f;

    [Header("Path Draw")]
    public PathFromPlanner pathDrawer;

    public Vector3 LastGoalWorld { get; private set; }
    public bool HasGoal { get; private set; }

    public void ResetGoal() => HasGoal = false;

    void Awake()
    {
        if (cam == null) cam = Camera.main;
    }

    void Update()
    {
        if (EventSystem.current != null && EventSystem.current.IsPointerOverGameObject())
            return;

        bool pressed = false;
        if (Mouse.current != null && Mouse.current.leftButton.wasPressedThisFrame) pressed = true;
        if (!pressed && Touchscreen.current != null && Touchscreen.current.primaryTouch.press.wasPressedThisFrame) pressed = true;
        if (!pressed) return;

        Vector2 screenPos = Vector2.zero;
        if (Mouse.current != null) screenPos = Mouse.current.position.ReadValue();
        else if (Touchscreen.current != null) screenPos = Touchscreen.current.primaryTouch.position.ReadValue();

        Ray ray = cam.ScreenPointToRay(screenPos);
        if (!Physics.Raycast(ray, out RaycastHit hit, 1000f, groundMask))
            return;

        Vector3 p = hit.point;
        LastGoalWorld = p;
        HasGoal = true;

        if (goalMarker) // 마커 이동
        {
            goalMarker.position = new Vector3(p.x, goalMarker.position.y, p.z);
        }

        if (planner && startTowing)
        {
            if (pathDrawer) pathDrawer.Clear();

            planner.RequestPlan( // planner에게 plan 만들어주세요
                startTowing,
                startPlane,
                LastGoalWorld,
                goalYawUnityDeg,
                goalPlaneYawUnityDeg,
                (resp) =>
                {
                    if (pathDrawer) pathDrawer.Apply(resp);
                }
            );
        }
        else
        {
            Debug.LogWarning("[GoalClick] planner/startTractor 연결 안 됨");
        }
    }
}
