import math
import pickle
import argparse
import pandas as pd
from metrics.collision import CollisionMetric
from call_ilqr import *

def read_json_file(json_file):
    map_data = None
    try:
        with open(json_file, 'r', encoding='utf-8') as file:
            map_data = json.load(file)
    except Exception as e:
        print(f"read map data failed")

    return map_data


def read_pkl_file(pkl_file):
    traj_data = None
    try:
        with open(pkl_file, 'rb') as file:
            traj_data = pickle.load(file)
    except Exception as e:
        print(f"read trajectory data failed")
    return traj_data


def inputs_preprocess(single_sample_dir, agent_file, map_file, pkl_file):
    map_file_path = os.path.join(single_sample_dir, map_file)
    agent_file_path = os.path.join(single_sample_dir, agent_file)
    pkl_file_path = os.path.join(single_sample_dir, pkl_file)
    map_data = read_json_file(map_file_path)
    agent_data = pd.read_parquet(agent_file_path)
    traj_data = read_pkl_file(pkl_file_path)
    if traj_data is not None:
        # calculate v/s/heading of prediction trajectory point via trajectory points position
        for obs_index, obs_trajs in traj_data['traj'].items():
            for each_traj in obs_trajs:
                sum_s = 0.0
                for point_index in range(len(each_traj)):
                    if point_index > 0:
                        sum_s += math.sqrt((each_traj[point_index]['x'] - each_traj[point_index - 1]['x']) ** 2 + (
                                each_traj[point_index]['y'] - each_traj[point_index - 1]['y']) ** 2)
                    each_traj[point_index]['s'] = round(sum_s, 2)

                    if point_index + 1 < len(each_traj):
                        dis_to_next_pt = math.sqrt(
                            (each_traj[point_index + 1]['x'] - each_traj[point_index]['x']) ** 2 + (
                                    each_traj[point_index + 1]['y'] - each_traj[point_index]['y']) ** 2)
                        each_traj[point_index]['v'] = round(
                            dis_to_next_pt / 0.1, 2)
                        each_traj[point_index]['heading'] = math.atan2(
                            each_traj[point_index + 1]['y'] - each_traj[point_index]['y'],
                            each_traj[point_index + 1]['x'] - each_traj[point_index]['x'])
                    elif len(each_traj) == 1:
                        each_traj[point_index]['v'] = 0.0
                        each_traj[point_index]['heading'] = 0.0
                    else:
                        each_traj[point_index]['v'] = round(
                            each_traj[point_index - 1]['v'], 2)
                        each_traj[point_index]['heading'] = round(
                            each_traj[point_index - 1]['heading'], 2)

    # get ego origin state
    ego_origin_state = PlanningPoint()
    if agent_data is not None:
        ego_data = agent_data[agent_data['track_id'] == 'AV']
        ego_origin_state.position.x = ego_data.iloc[49]['position_x']
        ego_origin_state.position.y = ego_data.iloc[49]['position_y']
        ego_origin_state.velocity = math.sqrt(ego_data.iloc[49]['velocity_x'] ** 2 +
                                              ego_data.iloc[49]['velocity_y'] ** 2)
        ego_origin_state.theta = ego_data.iloc[49]['heading']
        
    # get ego traj
    sum_error = []           
    for obs_index, trajs in traj_data['traj'].items():
        error_dis = 0.0
        for each_traj in trajs:
            first_pt = each_traj[0]
            error_dis += math.sqrt((first_pt['x'] - ego_origin_state.position.x) ** 2 +
                                   (first_pt['y'] - ego_origin_state.position.y) ** 2)
        sum_error.append(error_dis)
    ego_traj_index = sum_error.index(min(sum_error))
    ego_row = traj_data['pi'][ego_traj_index]
    max_probability_ego_traj_index = np.argmax(ego_row)
    ego_traj_data = traj_data['traj'][ego_traj_index][max_probability_ego_traj_index]
    
    return map_data, agent_data, ego_traj_data, ego_origin_state


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_samples_dir", default="", type=str, required=True,
                        help="input_samples file directory")
    parser.add_argument("--enable_closed_loop_flag", default="", type=bool,
                        help="enable closed loop model evaluation or not")
    parser.add_argument("--evaluation_results_dir", default="",
                        type=str, required=True, help="save dlp model evaluation results file directory")

    args = parser.parse_args()
    input_samples_file_paths_lists = os.listdir(args.input_samples_dir)
    for single_sample in input_samples_file_paths_lists:
        single_sample_dir = os.path.join(args.input_samples_dir, single_sample)
        input_files_list = os.listdir(single_sample_dir)
        agent_file = map_file = pkl_file = ""
        for index, input_raw_data_file in enumerate(input_files_list):
            file_extension = os.path.splitext(input_raw_data_file)[1]
            if file_extension == '.parquet':
                agent_file = input_raw_data_file
            elif file_extension == '.json':
                map_file = input_raw_data_file
            elif file_extension == '.pkl':
                pkl_file = input_raw_data_file

        # extract agent & map & dlp ego trajectory
        map_data, agent_data, \
                ego_traj_data, ego_origin_state= inputs_preprocess(
            single_sample_dir, agent_file, map_file, pkl_file)

        if args.enable_closed_loop_flag:
            # run ilqr to abtain processed trajectory
            call_ilqr = CallILQR(ego_traj_data, ego_origin_state)
            call_ilqr.process()
            processed_trajectory = call_ilqr.get_processed_trajs()
                
        # run metrics evaluation that we are interested in
        # collision_metric = CollisionMetric(collision_threshold=0.0)
        # collision_metric.evaluate(ego_traj_data, map_data, agent_data)

        # write evaluation result into statistical files
        print(f"sample {single_sample}: dlp model evaluation successful")
