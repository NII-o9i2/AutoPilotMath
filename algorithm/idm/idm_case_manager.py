import json
import os
import argparse

class IDMCaseManager:
    def __init__(self, case_path) -> None:
        self.case_path_ = case_path
        self.case_set_ = dict()
        self.__read_case__()

    def __read_case__(self):
        print("*" * 30 + " IDMCaseManager start read case " + "*" * 30)
        # 遍历路径下的所有文件
        path = '/home/sensetime/ws/common_math/algorithm/idm/'+self.case_path_
        for filename in os.listdir(path):
            if filename.endswith('.json'):  # 确保文件是以 .json 结尾的 JSON 文件
                file_path = os.path.join(path, filename)

                # 读取 JSON 文件
                with open(file_path, 'r') as file:
                    data = json.load(file)

                # 处理读取的 JSON 数据
                # 这里假设 JSON 文件的内容是一个包含键值对的字典
                # for key, value in data.items():
                #     print(key, value)

                # name_key = "follow_static_car_case"
                # if name_key in data and data[name_key] != "" and item_key in data and data[name_key] != None:
                #     self.case_set_[data[name_key]] = data[item_key]

                # for case_name, case_item in self.case_set_.items():
                #     print("read " + case_name)
                #     print(case_item)
                for key in data:
                    for case in data[key]:
                        self.case_set_[case['case_name']]=case['case_item']

                # for case_name, case_item in self.case_set_.items():
                #     print("read " + case_name)
                #     print(case_item)
    def get_case(self, case_name) -> dict():
        if case_name in self.case_set_ and self.case_set_[case_name] != None:
            return self.case_set_[case_name]
        else:
            print("no such case " + case_name)
        return None

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='longi case manager',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-s',
                        '--source',
                        dest='source',
                        help='specify source case path',
                        required=False,
                        default='./idm_case_file')
    args = parser.parse_args()

    case_manager = IDMCaseManager(args.source)

    print("get case_1")
    print(case_manager.get_case("case_1"))