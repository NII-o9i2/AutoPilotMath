from bokeh.plotting import figure, show, output_file, output_notebook
import argparse
from scipy.interpolate import interp1d
import numpy as np
from idm_case_manager import IDMCaseManager
from idm_case_ploter import IDMCasePloter
import sys
sys.path.append('/home/sensetime/ws/common_math/algorithm/idm/build')
import pybind_idm

def linear_interpolation(x, y, x_new):
    f = interp1d(x, y, kind='linear')
    return f(x_new)

class IDMDebug():
    def __init__(self, case_name,output):
        self.case_name_ = case_name
        self.output_ = output

    def idm_debug_func(self):
        idm_params = pybind_idm.IDMParam(
                0.2,  # delta_t
                4.0,  # idm_cosnt_jerk
                3.0,  # delta
                2.0,  # acc_delta
                3.5,  # desired_deacc
                2.5,  # max_acc 输出限幅
                1.0,  # time_headway
                1.5,  # follow_car_delta
                0.0,  # min_spacing
                4.15,  # min_dacc 输出限幅
                [9.49, 18.98, 28.48],  # v_std
                [0.88, 1.7, 2.7],     # vel_error_std
                [0.0, 2.78, 11.11, 22.22, 90.0],  # idm_delta_v
                [3.0, 4.0, 5.0, 6.0, 8.0],     # idm_min_spacing_list
                [-2, 0, 2, 10],  #delta_v
                [0.2, 0.3, 0.5, 0.7],  #s_change_rate
                0.0,  # desired_spd
                0.0)  # change_rate

        idm_output = []

        # get test case
        case_path = '/idm_case_file'
        case_manager = IDMCaseManager(case_path)
        picked_case = case_manager.get_case(self.case_name_)

        t_triger_std = picked_case['idm_info']['t_triger']
        # read leader info
        leadercarlength_std = picked_case['idm_info']['leader_length']
        s_leader_init = picked_case['idm_info']['s_leader_init']
        v_leader_init = picked_case['idm_info']['v_leader_init'] / 3.6
        a_leader_init = picked_case['idm_info']['a_leader_init']
        exist_leader = picked_case['idm_info']['exist_leader']
        # read v_refs
        v_refs = picked_case['idm_info']['v_refs'] / 3.6
        # read ego init info
        ego_s = picked_case['ego_init_info']['s']
        ego_v = picked_case['ego_init_info']['v'] / 3.6
        ego_a = picked_case['ego_init_info']['a']
        
        # linear_interpolation info to idm input can use
        leader_info = []

        t_triger_list = []
        leadercarlength_list = []
        exist_leader_list = []
        v_refs_list = []

        for i in np.arange(0, 6.1, 0.2):
                t_triger_list.append(i)
        for t in t_triger_list:
                exist_leader_list.append(exist_leader)
                leadercarlength_list.append(leadercarlength_std)
                v_refs_list.append(v_refs)
        # print("exist_leader_list:",len(exist_leader_list),exist_leader_list)
        # print("leadercarlength_list:",len(leadercarlength_list),leadercarlength_list)
        # print("v_refs_list:",len(v_refs_list),v_refs_list)

        s_leader_list = []
        v_leader_list = []
        a_leader_list = []
        
        tmp_leader_s = s_leader_init
        tmp_leader_v = v_leader_init
        const_a = a_leader_init
        
        for t in t_triger_list:
                s_leader_list.append(tmp_leader_s)
                v_leader_list.append(tmp_leader_v)
                a_leader_list.append(const_a)
                tmp_leader_v += const_a * 0.2
                tmp_leader_s += tmp_leader_v * 0.2 + 0.5 * const_a * 0.2 * 0.2
        print("s leadre info:",len(s_leader_list),s_leader_list)
        print("v leadre info:",len(v_leader_list),v_leader_list)
        
        for i in range(len(t_triger_list)):
            leader_info.append(pybind_idm.SpeedPlannerProfile(
                leadercarlength_list[i], s_leader_list[i], v_leader_list[i], exist_leader_list[i]))
        ego_info = pybind_idm.EgoInfo(ego_s,ego_v,ego_a)
        # run idm
        idm_output = pybind_idm.get_idm_output(
                ego_info, leader_info, idm_params, v_refs_list)

        # plot case result
        plot_v_leader_list = []
        for item in v_leader_list:
                plot_v_leader_list.append(item * 3.6)
        plot_data = dict()
        plot_data['t_triger'] = t_triger_list
        plot_data['s_leader'] = s_leader_list
        plot_data['v_leader'] = plot_v_leader_list
        plot_data['a_leader'] = a_leader_list
        plot_data['t_ref_out'] = []
        plot_data['s_ref_out'] = []
        plot_data['v_ref_out'] = []
        plot_data['a_ref_out'] = []
        for idm_output_object in idm_output:
                plot_data['t_ref_out'].append(idm_output_object.t)
                plot_data['s_ref_out'].append(idm_output_object.s_ref_out)
                plot_data['v_ref_out'].append(idm_output_object.v_ref_out * 3.6)
                plot_data['a_ref_out'].append(idm_output_object.a_ref_out)
                # print("t:", idm_output_object.t)
                # print("s_ref_out:", idm_output_object.s_ref_out)
                # print("v_ref_out:", idm_output_object.v_ref_out)
                # print("a_ref_out:", idm_output_object.a_ref_out)
        case_ploter = IDMCasePloter()
        output_file(self.output_)
        layout = case_ploter.plot_idm_case(plot_data)
        show(layout)
        # output_notebook()

# usage:python3 idm_debug.py  -o ./idm.html -s static_car_30m
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='parse rsclbag',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-s',
                        '--source',
                        dest='source',
                        help='specify case name',
                        required=True)
    parser.add_argument('-o',
                        '--output',
                        dest='output',
                        help='specify output html',
                        required=True)
    args = parser.parse_args()
    idm_debug = IDMDebug(args.source,args.output)
    idm_debug.idm_debug_func()
