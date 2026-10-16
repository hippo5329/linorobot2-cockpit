"""A Map Viewer goal reports how it ended, and can be cancelled.

The viewer published /goal_pose and said "Goal sent: x= y= yaw=" -- and then
nothing, whether the robot arrived, Nav2 aborted, or the robot sat spinning in
place near the goal (2026-10-16: that one had to be cancelled from a shell). A
topic publish has no answer, so the goal now goes through the NavigateToPose
action (rosbridge's send_action_goal op), whose feedback and result the viewer
shows, and Cancel calls the action's cancel_goal service with an empty request,
which cancels every goal, whoever sent it.
"""
import json
import pathlib
import re
import shutil
import subprocess

import pytest

FRONTEND = pathlib.Path(__file__).resolve().parents[1] / "web" / "frontend"


def _rosviz():
    return (FRONTEND / "rosviz.js").read_text()


def _function(js, name):
    start = js.index(f"function {name}(")
    end = js.index("\n  }\n", start) + len("\n  }\n")
    return js[start:end]


def _block(js, start_marker):
    start = js.index(start_marker)
    end = js.index("\n  }\n", start) + len("\n  }\n")
    return js[start:end]


def _helpers(js):
    consts = js[js.index("  const GOAL_STATUS = {"):js.index("  function secondsOf(")]
    return consts + "\n".join(_function(js, n) for n in (
        "secondsOf", "describeNavFeedback", "describeNavResult", "activeGoals"))


def _node(src):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    out = subprocess.run([node, "-e", src], capture_output=True, text=True, check=True)
    return json.loads(out.stdout)


def _eval(expr):
    return _node(_helpers(_rosviz()) + f"\nconsole.log(JSON.stringify({expr}));")


# ------------------------------------------------------------- the outcome
def test_success_is_reported():
    r = _eval("describeNavResult({op: 'action_result', status: 4, result: true,"
              " values: {error_code: 0, error_msg: ''}})")
    assert r["level"] == "ok" and "succeeded" in r["text"]


def test_an_abort_carries_nav2s_error():
    r = _eval("describeNavResult({status: 6, result: true,"
              " values: {error_code: 9003, error_msg: 'Navigation timed out'}})")
    assert r["level"] == "err"
    assert "aborted" in r["text"] and "9003" in r["text"] and "Navigation timed out" in r["text"]


def test_a_cancel_is_reported_as_one():
    r = _eval("describeNavResult({status: 5, result: true, values: {error_code: 0}})")
    assert r["level"] == "warn" and "canceled" in r["text"]


def test_no_action_server_points_at_nav2():
    r = _eval("describeNavResult({status: 0, result: false, values: 'No action server available'})")
    assert r["level"] == "err"
    assert "No action server available" in r["text"] and "Nav2" in r["text"]


def test_feedback_says_how_far_and_how_many_recoveries():
    t = _eval("describeNavFeedback({distance_remaining: 1.234, navigation_time: {sec: 12, nanosec: 0},"
              " estimated_time_remaining: {sec: 0, nanosec: 0}, number_of_recoveries: 2})")
    assert "1.23 m to go" in t and "12 s" in t and "2 recoveries" in t


def test_only_live_goals_count():
    n = _eval("activeGoals({status_list: [{status: 2}, {status: 4}, {status: 6}, {status: 3},"
              " {status: 1}, {status: 5}]}).length")
    assert n == 3
    assert _eval("activeGoals(null).length") == 0


# ------------------------------------------------- the protocol, end to end
_FAKE_SOCKET = """
const sent = [];
class WebSocket {
  static OPEN = 1;
  constructor(url) { this.url = url; this.readyState = 1; globalThis.sock = this; }
  send(text) { sent.push(JSON.parse(text)); }
  close() { this.readyState = 3; }
}
"""


def _bridge_run(body):
    js = _rosviz()
    src = (_FAKE_SOCKET + _helpers(js)
           + 'const ADVERTISED_TYPES = {};\n'
           + _block(js, "  class RosBridge {")
           + "\nconst bridge = new RosBridge();\nbridge.connect('ws://x:9090');\n"
           + body)
    return _node(src)


def test_the_goal_is_sent_as_a_navigate_to_pose_action_with_feedback():
    out = _bridge_run("""
const seen = [];
const id = bridge.sendActionGoal('/navigate_to_pose', 'nav2_msgs/action/NavigateToPose',
  {pose: {header: {frame_id: 'map'}, pose: {}}, behavior_tree: ''},
  {onFeedback: (v) => seen.push(['fb', v.distance_remaining]),
   onResult: (m) => seen.push(['res', m.status])});
const msg = (o) => sock.onmessage({data: JSON.stringify(o)});
msg({op: 'action_feedback', id, action: '/navigate_to_pose', values: {distance_remaining: 2.5}});
msg({op: 'action_feedback', id: 'someone_else', values: {distance_remaining: 9}});
msg({op: 'action_result', id, status: 4, result: true, values: {}});
msg({op: 'action_result', id, status: 6, result: true, values: {}});
console.log(JSON.stringify({sent, seen, pending: bridge.pending.size}));
""")
    goal = out["sent"][0]
    assert goal["op"] == "send_action_goal"
    assert goal["action"] == "/navigate_to_pose"
    assert goal["action_type"] == "nav2_msgs/action/NavigateToPose"
    assert goal["feedback"] is True and goal["id"]
    # one feedback for this goal, one result, and nothing after the result
    assert out["seen"] == [["fb", 2.5], ["res", 4]]
    assert out["pending"] == 0


def test_a_lost_socket_ends_the_wait():
    out = _bridge_run("""
const seen = [];
bridge.sendActionGoal('/navigate_to_pose', 'nav2_msgs/action/NavigateToPose', {},
  {onResult: (m) => seen.push(m)});
bridge.shouldReconnect = false;
sock.onclose();
console.log(JSON.stringify(seen));
""")
    assert len(out) == 1 and out[0]["result"] is False


def test_cancel_calls_the_cancel_goal_service_with_an_empty_request():
    out = _bridge_run("""
const seen = [];
bridge.callService('/navigate_to_pose/_action/cancel_goal', 'action_msgs/srv/CancelGoal', {},
  (m) => seen.push(m.values.goals_canceling.length));
sock.onmessage({data: JSON.stringify({op: 'service_response', id: sent[0].id, result: true,
  values: {return_code: 0, goals_canceling: [{}]}})});
console.log(JSON.stringify({sent, seen}));
""")
    call = out["sent"][0]
    assert call["op"] == "call_service"
    assert call["service"] == "/navigate_to_pose/_action/cancel_goal"
    assert call["type"] == "action_msgs/srv/CancelGoal"
    assert call["args"] == {}, "an empty request (zero id, zero stamp) cancels every goal"
    assert out["seen"] == [1]


# ----------------------------------------------------------------- wiring
def test_the_page_has_a_cancel_button_and_a_navigation_line():
    html = (FRONTEND / "index.html").read_text()
    assert 'id="btn-rosviz-cancel"' in html
    assert 'id="rosviz-nav"' in html


def test_the_goal_no_longer_goes_to_goal_pose():
    """Both at once would send Nav2 the goal twice."""
    js = _rosviz()
    assert 'publish("/goal_pose"' not in js
    assert "sendActionGoal(NAV_ACTION" in js


def test_goals_from_elsewhere_are_watched():
    js = _rosviz()
    assert re.search(r'bridge\.subscribe\(`\$\{NAV_ACTION\}/_action/status`, "action_msgs/msg/GoalStatusArray"', js)


def test_the_bridge_runs_actions_off_its_socket_thread():
    """In the bridge's own thread a goal blocks every other op until Nav2 answers."""
    launch = (FRONTEND.parents[1] / "launchers" / "bringup.launch.py").read_text()
    assert '"send_action_goals_in_new_thread": True' in launch
    assert '"call_services_in_new_thread": True' in launch
