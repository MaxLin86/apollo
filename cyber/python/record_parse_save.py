"""
function to parse data from *.record files, created using Apollo-Auto

current implementation illustrates sample record file parsing for
* radar (Continental ars-408)
* camera (Leopard Imaging 6mm)
* lidar (Velodyne vls-128)

* saves extracted images in separate folder using *.jpg format
* saves radar and lidar data in respective folders in *.txt format for each scan
* also saves timestamp in separate text files

"""

###########################################################
# import packages
import sys, time, os, yaml, stat
from importlib import import_module

print(os.environ.keys())
from cyber_py import cyber
from cyber_py import record

os.system('clear')

###########################################################
# def read_parameters(yaml_file):
def read_parameters(params):
    """
    function to read YAML parameter file and define output destinations
    """
    # record file params
    RECORD_FOLDER = params['records']['filepath']
    parse_type = params['parse']

    # define destinations
    dest_path = os.path.split(RECORD_FOLDER)
    if not dest_path[-1]:
        dest_path = os.path.split(dest_path[0])

    OUT_FOLDER = dest_path[0] + '/'
    temp_path = os.path.split(dest_path[0])
    # FOLDER_PREFIX = temp_path[1].replace("-", "")
    FOLDER_PREFIX = temp_path[1]

    parse_dict = {"params": params,
                    "parse_type": parse_type,
                    "out_folder": OUT_FOLDER,
                    "prefix": FOLDER_PREFIX,
                    "record_folder": RECORD_FOLDER }
    print(parse_dict)
    return parse_dict

###########################################################
def define_destinations(parse_dict):
    """
    define destination for extracted files
    """
    dest_dict = {
            "channel_name" : "",
            "timestamp_file": "",
            "destination_folder": ""
    }

    parse_type = parse_dict["parse_type"]
    params = parse_dict["params"]
    dest_folder = parse_dict["out_folder"]
    prefix = parse_dict["prefix"]

    parser_func = 'parse_' + parse_type

    vid_name_extract = params['records']['filepath'].split('/')
    if  vid_name_extract[-1] == '':
        vid_name =  vid_name_extract[-2]
    else:
        vid_name =  vid_name_extract[-1]
    print('vid name: {}'.format(vid_name))

    dest_dict['channel_name'] = params[parse_type]['channel_name']
    dest_dict['timestamp_file'] = dest_folder + prefix + params[parse_type]['timestamp_file_extn']
    dest_dict['destination_folder'] = dest_folder + prefix + params[parse_type]['out_folder_extn'] + '/' +  vid_name +  '/'

    exist_flag = False
    if not os.path.exists(dest_dict["destination_folder"]):
        # os.makedirs(dest_dict["destination_folder"])
        param_path = dest_dict["destination_folder"]
        print("mkdir: {}".format(param_path))
        os.system("mkdir -p %s" % (param_path))
    else:
        print("already exists: {}".format(dest_dict["destination_folder"]))
        exist_flag = True

    return dest_dict, parser_func, exist_flag

###########################################################
def parse_apollo_record(parse_dict, dest_dict, parser_func):
    """
    """
    record_folder_path = parse_dict["record_folder"]
    parse_type = parse_dict["parse_type"]
    print("record_folder: {}".format(parse_dict["record_folder"]))
    record_files = sorted(os.listdir(parse_dict["record_folder"]))
    parse_timestamp = []
    print("import_module: {}".format(parser_func))
    parse_mod = import_module(parser_func)

    print("=" *60)
    print('--------- Parsing data for: ' + parse_type + ' ---------')

    for index in range(len(record_files)):
        print('sub record index: {}'.format(index))
        rfile = record_files[index]
        frame_index = 0
        print("=" *60)
        print("parsing record file: %s" % rfile)
        freader = record.RecordReader(record_folder_path + rfile)
        # time.sleep(.025)
        time.sleep(.25)

        for channelname, msg, datatype, timestamp in freader.read_messages():
            # print('channel name: {}'.format(channelname))
            # print('dest_dict: {}'.format(dest_dict["channel_name"]))
            if channelname == dest_dict["channel_name"]:
                tstamp = parse_mod.parse_data(channelname, msg, dest_dict['destination_folder'], index, frame_index)
                frame_index += 1
                parse_timestamp.append(tstamp)

        # write radar-timestamp files
        # with open(dest_dict["timestamp_file"], 'w+') as f:
        #     for item in parse_timestamp:
        #         f.write("%s\n" % item)

    print("=" *60)
    print('DONE: records parsed and data saved to: \n  ' + dest_dict['destination_folder'])
    print("=" *60)

def process_main_call(params):
    parse_dict = read_parameters(params)    
    dest_dict, parser_func, exist_flag = define_destinations(parse_dict)
    if exist_flag is False:
        parse_apollo_record(parse_dict, dest_dict, parser_func)

###########################################################
if __name__ == '__main__':
    cyber.init()
    yaml_path = 'cyber/python/parser_params.yaml'

    with open(yaml_path, 'r') as f:
        params = yaml.load(f)
    if  'folderpath' in params['records']:
        folder_path = params['records']['folderpath']
        print('folderpath: {}'.format(folder_path))
        subfolders = os.listdir(folder_path)
        for subfolder in subfolders:
            new_file_path = os.path.join(folder_path, subfolder) + '/'
            record_num = 0
            for file in os.listdir(new_file_path):
                if  '.record.' in file:
                    record_num += 1
            if  record_num > 0:
                params['records']['filepath'] = new_file_path
                process_main_call(params)    
    else:
        process_main_call(params)

    cyber.shutdown()
