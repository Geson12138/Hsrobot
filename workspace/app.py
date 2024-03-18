from flask import Flask, request, jsonify
import numpy as np
# import gym

app = Flask(__name__)

#初始化操作
# env_id = 'ReachArm11Env'
# entry_point = 'muscle_arm11:ReachArm11Env'
# max_episode_steps = 10000
# gym.envs.registration.register(id=env_id,
#                                entry_point=entry_point,
#                                max_episode_steps=max_episode_steps)
# assert (env_id in gym.envs.registry.keys()), \
#     f"ERROR: {env_id} not found in env registry"
# print(f"RS:> Registering {env_id} Successfully!")
# env = gym.make(env_id, render_mode="rgb_array")
# env.reset()
# print("RS:> Creating Environment Instance Successfully!")


# @app.route('/forward_step', methods=["POST"])
# def forward_step():
#     try:
#         data = request.get_json()
#         ctrl = data["activation"]
#         if data["disturbance"]:
#             disturb = data["disturbance"]["intensity"]
#             id = data["disturbance"]["id"]
#             activation = ctrl[id-1] + disturb
#             ctrl[id-1] = activation
#         # target_pos = request.form["target_position"]
#         # obs_pos = request.form["obstacle_position"]
#         _, _, _, _, feedback_info = env.step(ctrl)
#         # feedback_info["target_position"] = target_pos
#         # feedback_info["obstacle_position"] = obs_pos
#         result = json.dumps(feedback_info)
#         return result
#     except Exception as e:
#         return jsonify({'code': 0, 'error': 'Invalid JSON data', 'message': str(e)})


# @app.route('/test', methods=["POST"])
# def test():
#     try:
#         data = request.get_json()
#         if data:
#             calc = Calculator()
#             result = calc.add(data['x'], data['y'])
#             return jsonify({'code': 1, 'message': 'Success', 'data': result})
#         else:
#             return jsonify({'code': 0, 'message': 'No JSON data received'})
#     except Exception as e:
#         return jsonify({'code': 0, 'error': 'Invalid JSON data', 'message': str(e)})


@app.route('/env_view', methods=["GET"])
def system_view():
    return jsonify({'code': 0, 'message': ''})


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=8001, debug=True)