import json
import os

from flask import Flask, jsonify, request

from astar import astar
from map_data import graph, nodes
from trajectory import generate_trajectory

try:
    import rospy
    from std_msgs.msg import String as RosString
except Exception:
    rospy = None
    RosString = None


app = Flask(__name__)
ros_pub = None
ros_enabled = False


def init_ros():
    global ros_pub, ros_enabled

    if os.getenv("ENABLE_ROS", "0") != "1":
        return
    if rospy is None or RosString is None:
        return

    if not rospy.core.is_initialized():
        rospy.init_node("trajectory_server", anonymous=True, disable_signals=True)

    ros_pub = rospy.Publisher("/limo/trajectory", RosString, queue_size=1)
    ros_enabled = True


def render_index_page():
    options = "\n".join(
        f'<option value="{name}">{name}</option>' for name in sorted(nodes.keys())
    )

    return f"""
<!doctype html>
<html>
<head>
  <meta charset=\"utf-8\" />
  <title>Trajectory Server</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 24px; }}
    label {{ display: block; margin-top: 12px; }}
    select, input {{ margin-top: 4px; }}
    button {{ margin-top: 16px; }}
    pre {{ background: #f4f4f4; padding: 12px; }}
    #path-result {{ margin: 12px 0; font-weight: bold; }}
  </style>
</head>
<body>
  <h2>Trajectory Planner</h2>
  <div id=\"path-result\"></div>
  <form id=\"plan-form\">
    <label>Start node
      <select name=\"start\">{options}</select>
    </label>
    <label>Goal node
      <select name=\"goal\">{options}</select>
    </label>
    <label>v_max
      <input name=\"v_max\" value=\"1.0\" />
    </label>
    <label>a_max
      <input name=\"a_max\" value=\"0.5\" />
    </label>
    <label>dt
      <input name=\"dt\" value=\"0.1\" />
    </label>
    <button type=\"submit\">Plan</button>
  </form>
  <h3>Result</h3>
  <pre id=\"result\"></pre>

  <script>
    const form = document.getElementById('plan-form');
    const result = document.getElementById('result');
    const pathResult = document.getElementById('path-result');
    form.addEventListener('submit', async (e) => {{
      e.preventDefault();
      const data = new FormData(form);
      const body = new URLSearchParams(data);
      result.textContent = 'Planning...';
      pathResult.textContent = '';
      const response = await fetch('/plan', {{ method: 'POST', body }});
      const json = await response.json();
      if (json.path) {{
        pathResult.textContent = `Plus court chemin trouve : ${{json.path.join(' -> ')}}`;
      }} else if (json.error) {{
        pathResult.textContent = `Erreur : ${{json.error}}`;
      }}
      result.textContent = JSON.stringify(json, null, 2);
    }});
  </script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_index_page()


@app.route("/plan", methods=["POST"])
def plan():
    start = request.form.get("start")
    goal = request.form.get("goal")

    if start not in nodes or goal not in nodes:
        return jsonify({"error": "invalid node"}), 400

    if start == goal:
        return jsonify({"error": "start equals goal"}), 400

    v_max = float(request.form.get("v_max", 1.0))
    a_max = float(request.form.get("a_max", 0.5))
    dt = float(request.form.get("dt", 0.1))

    path = astar(graph, nodes, start, goal)
    if not path:
        return jsonify({"error": "no path"}), 400

    trajectory = generate_trajectory(
        path,
        nodes,
        v_max=v_max,
        a_max=a_max,
        dt=dt,
        samples_per_segment=20,
    )

    payload = {
        "start": start,
        "goal": goal,
        "path": path,
        "trajectory": trajectory,
    }

    if ros_enabled and ros_pub is not None:
        ros_pub.publish(json.dumps(payload))

    return jsonify(payload)


if __name__ == "__main__":
    init_ros()
    debug = os.getenv("FLASK_DEBUG", "1") == "1"
    # Flask's debug reloader starts the app twice; in ROS mode this can
    # re-initialize the node and publisher unexpectedly.
    use_reloader = False if ros_enabled else debug
    app.run(host="0.0.0.0", port=5000, debug=debug, use_reloader=use_reloader)
