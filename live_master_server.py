import socket
import json
import time
import concurrent.futures
import torch
import numpy as np
import threading

mpc_lock = threading.Lock()

from model import highwayNet
from closed_loop_orchestrator import VehicleControlUnit
from omnet_to_cslstm_bridge import build_lane_lateral_map, run_inference

# Safety tuning (Python-side pre-filter before Veins safety shield)
HARD_BRAKE_GAP_M = 10.0
HARD_BRAKE_STOPPED_LEADER_MS = 0.5
HARD_BRAKE_TTC_S = 2.0
CRITICAL_GAP_M = 5.0


def get_model_arguments():
    args = {}
    args['use_cuda'] = False
    args['encoder_size'] = 64
    args['decoder_size'] = 128
    args['in_length'] = 16
    args['out_length'] = 25
    args['grid_size'] = (13, 3)
    args['soc_conv_depth'] = 64
    args['conv_3x1_depth'] = 16
    args['dyn_embedding_size'] = 32
    args['input_embedding_size'] = 32
    args['num_lat_classes'] = 3
    args['num_lon_classes'] = 2
    args['use_maneuvers'] = True
    args['train_flag'] = False
    return args


def load_pytorch_model(model_path):
    print("=" * 60)
    print(f"[*] INITIALIZING INTEL CPU RUNTIME RECONSTRUCTION: {model_path}")
    print("=" * 60)
    try:
        args = get_model_arguments()
        net = highwayNet(args)
        state_dict = torch.load(model_path, map_location=torch.device('cpu'))
        net.load_state_dict(state_dict)
        net.eval()
        print("[SUCCESS] highwayNet Architecture is armed and locked in Intel CPU memory!\n")
        return net
    except Exception as e:
        print(f"\n[FATAL ERROR INITIALIZING AI MODEL]: {e}")
        print("[Diagnostic] Verify that model.py is in the same directory as this script.")
        exit(1)


def compute_agent(ego_id, current_state, final_goal, ego_preds, nbrs_current, nbrs_preds, vcu_instance):
    try:
        with mpc_lock:
            verdict = vcu_instance.run_step(current_state, final_goal, ego_preds, nbrs_current, nbrs_preds)
        return ego_id, verdict, True
    except Exception as e:
        print(f"[Core Math Error - Vehicle {ego_id}]: {e}")
        return ego_id, {"u_safe": [0.0, -6.0], "mode": "Emergency Fallback"}, False


def apply_lane_escape_hint(ego_veh, u_safe):
    """Suggest lateral escape when Veins reports a stopped leader in range (CBF/NMPC hint)."""
    gap = ego_veh.get('Leader_Gap_m')
    if gap is None:
        return u_safe

    gap = float(gap)
    v_lead = float(ego_veh.get('Leader_Speed_ms', ego_veh.get('sigVel', 0.0)))
    closing = float(ego_veh.get('Closing_Speed_ms', 0.0))
    ax, ay = float(u_safe[0]), float(u_safe[1])

    if 8.0 < gap < 28.0 and v_lead < 0.5 and closing > 0.15 and abs(ax) < 1.0:
        return [2.0, min(ay, -0.5)]

    return [ax, ay]


def apply_leader_hard_brake(ego_veh, u_safe):
    """Pre-CBF override using TraCI leader fields from Veins."""
    gap = ego_veh.get('Leader_Gap_m')
    if gap is None:
        return u_safe

    gap = float(gap)
    v_lead = float(ego_veh.get('Leader_Speed_ms', ego_veh.get('sigVel', 0.0)))
    closing = float(ego_veh.get('Closing_Speed_ms', 0.0))
    ttc = ego_veh.get('TTC_s')

    ax, ay = float(u_safe[0]), float(u_safe[1])

    if gap < CRITICAL_GAP_M and closing > 0.2:
        return [0.0, -6.0]

    if gap < HARD_BRAKE_GAP_M and v_lead < HARD_BRAKE_STOPPED_LEADER_MS and closing > 0.2:
        return [0.0, -6.0]

    if ttc is not None and ttc != '' and float(ttc) < HARD_BRAKE_TTC_S:
        return [0.0, min(ay, -6.0)]

    return [ax, ay]


def is_relevant_neighbor(ego_veh, other_veh, max_planar_dist_m=80.0):
    ego_road = ego_veh.get('Road_ID')
    other_road = other_veh.get('Road_ID')
    if ego_road and other_road and ego_road != other_road:
        return False
    ex = float(ego_veh.get('sigGlobalX', 0.0))
    ey = float(ego_veh.get('sigGlobalY', 0.0))
    ox = float(other_veh.get('sigGlobalX', 0.0))
    oy = float(other_veh.get('sigGlobalY', 0.0))
    if ex or ey or ox or oy:
        return ((ox - ex) ** 2 + (oy - ey) ** 2) ** 0.5 <= max_planar_dist_m
    return abs(float(other_veh['sigLocalY']) - float(ego_veh['sigLocalY'])) <= max_planar_dist_m


def same_lane(ego_veh, other_veh):
    ego_lane = str(ego_veh.get('Lane_ID', ''))
    other_lane = str(other_veh.get('Lane_ID', ''))
    if ego_lane and other_lane and ego_lane != other_lane:
        return False
    return True


def build_neighbor_lists(ego_veh, all_vehicles, lane_map):
    """Leader-aware neighbor set for CBF/NMPC (same road, sorted by longitudinal offset)."""
    ego_id = ego_veh['Car_ID']
    ego_lat = lane_map.get(float(ego_veh['Lane_ID']), lane_map.get(str(int(float(ego_veh['Lane_ID']))), 0.0))
    ego_y = float(ego_veh['sigLocalY'])

    candidates = []
    leader_id = ego_veh.get('Leader_ID')

    for n_veh in all_vehicles:
        if n_veh['Car_ID'] == ego_id:
            continue
        if not is_relevant_neighbor(ego_veh, n_veh):
            continue

        n_lat = lane_map.get(float(n_veh['Lane_ID']), lane_map.get(str(int(float(n_veh['Lane_ID']))), 0.0))
        n_y = float(n_veh['sigLocalY'])
        dy = n_y - ego_y
        dx = n_lat - ego_lat
        same = same_lane(ego_veh, n_veh)
        is_leader = leader_id is not None and str(n_veh['Car_ID']) == str(leader_id)
        candidates.append((0 if is_leader else 1, 0 if same else 1, abs(dx), dy, n_lat, n_y, n_veh))

    candidates.sort(key=lambda t: (t[0], t[1], t[2], t[3]))

    nbrs_curr = []
    nbrs_pred = []
    dt_out = 0.2

    for _, _, _, dy, n_lat, n_y, n_veh in candidates:
        if abs(dy) > 120.0:
            continue
        n_v = float(n_veh['sigVel'])
        nbrs_curr.append([n_lat, n_y, 0.0, n_v])

        pred_y = [n_y + n_v * dt_out * i for i in range(25)]
        nbrs_pred.append({
            'Pred_Lateral_m': [n_lat] * 25,
            'Pred_LocalY_m': pred_y,
            'Sigma_Lat_m': [0.1] * 25,
            'Sigma_Long_m': [0.2] * 25,
        })

        if len(nbrs_curr) >= 6:
            break

    return nbrs_curr, nbrs_pred


def run_realtime_server():
    model_file_path = 'trained_models/cslstm_m.tar'
    net = load_pytorch_model(model_file_path)

    HOST = '0.0.0.0'
    PORT = 9999

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((HOST, PORT))
    print(f"[Network] UDP HIL Daemon Listening globally on {HOST}:{PORT}")
    print("[-] Standing by. Launch the OMNeT++/Veins simulation on the Linux VM...")
    print("=" * 60)

    active_agents = {}
    live_history_buffer = []
    lane_map = {"0": 0.0, "1": 3.5, "2": 7.0}

    def lane_offset(veh):
        lid = float(veh['Lane_ID'])
        return lane_map.get(lid, lane_map.get(str(int(lid)), 0.0))

    def refresh_lane_map():
        nonlocal lane_map
        if len(live_history_buffer) >= 10:
            built, _ = build_lane_lateral_map(live_history_buffer)
            if built:
                lane_map = built

    def ego_predictions_for_nmpc(pred_dict):
        return {
            'Pred_Lateral_m': pred_dict['lat_m'],
            'Pred_LocalY_m': pred_dict['long_m'],
            'Sigma_Lat_m': pred_dict['sigma_lat_m'],
            'Sigma_Long_m': pred_dict['sigma_long_m'],
        }

    with torch.no_grad(), concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
        while True:
            data, addr = server_socket.recvfrom(65536)
            traffic_state = json.loads(data.decode('utf-8'))
            current_time = traffic_state['Time_sec']

            for veh in traffic_state['Vehicles']:
                veh['Time_sec'] = current_time

            live_history_buffer.extend(traffic_state['Vehicles'])
            if len(live_history_buffer) > 2000:
                live_history_buffer = live_history_buffer[-1000:]

            print(f"\n[Tick t={current_time:.2f}s] Packet from {addr[0]} | Vehicles Tracked: {len(traffic_state['Vehicles'])}")

            response_payload = {
                "Commands": [],
                "Time_sec": current_time,
                "Tick_seq": traffic_state.get("Tick_seq", 0),
                "ai_ready": current_time >= 1.6,
            }
            futures = []

            if current_time >= 1.6:
                refresh_lane_map()
                for veh in traffic_state['Vehicles']:
                    ego_id = veh['Car_ID']

                    if ego_id not in active_agents:
                        active_agents[ego_id] = VehicleControlUnit()

                    try:
                        ego_preds, _ = run_inference(net, ego_id, live_history_buffer, current_time, lane_map)
                    except ValueError:
                        continue

                    has_nan_long = np.isnan(ego_preds['long_m']).any() or np.isinf(ego_preds['long_m']).any()
                    has_nan_lat = np.isnan(ego_preds['lat_m']).any() or np.isinf(ego_preds['lat_m']).any()

                    if has_nan_long or has_nan_lat:
                        print(f"[WARNING] Skipping Optimizer for Vehicle {ego_id}: Model returned NaN/Inf trajectory.")
                        continue

                    lat_pos = lane_offset(veh)
                    current_state = [lat_pos, veh['sigLocalY'], 0.0, veh['sigVel']]
                    final_goal = [current_state[0], current_state[1] + 100.0]

                    nbrs_curr, nbrs_pred = build_neighbor_lists(veh, traffic_state['Vehicles'], lane_map)
                    ego_nmpc = ego_predictions_for_nmpc(ego_preds)

                    future = executor.submit(
                        compute_agent, ego_id, current_state, final_goal,
                        ego_nmpc, nbrs_curr, nbrs_pred, active_agents[ego_id]
                    )
                    futures.append((future, veh))

                for future, veh in futures:
                    ego_id, verdict, success = future.result()
                    if success:
                        u_safe = apply_leader_hard_brake(veh, verdict['u_safe'])
                        u_safe = apply_lane_escape_hint(veh, u_safe)
                        response_payload["Commands"].append({
                            "Car_ID": int(ego_id),
                            "a_x": float(u_safe[0]),
                            "a_y": float(u_safe[1]),
                        })

            response_json = json.dumps(response_payload)
            server_socket.sendto(response_json.encode('utf-8'), addr)
            print(f"  └─► Echoed execution commands back to VM. Actuated Agents: {len(response_payload['Commands'])}")


if __name__ == "__main__":
    torch.set_num_threads(1)
    run_realtime_server()