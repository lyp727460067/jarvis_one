import argparse
from ruamel.yaml import YAML
from pathlib import Path

def read_all_configs(yaml=None):
    """Read all configs from configs.yaml"""
    with open('configs.yaml', 'r') as config_file:
        if yaml is None:
            yaml = YAML()
        all_configs = yaml.load(config_file)
    return all_configs

def main():

    parser = argparse.ArgumentParser(
        description='Create configuration for MB-VIO in configs.yaml.')
    parser.add_argument('--mbvio_folder', type=str, help='Path to location where the source code of MB-VIO was cloned.',
                        required=True)
    parser.add_argument('--results_folder', type=str,
                        help='The results of running MB-VIO will be stored in this folder.', required=True)

    parser.add_argument('--data_folder', type=str,
                        help='The folder of euroc dataset.', required=True)
    
    parser.add_argument('--program_name', type=str,
                        help='Name of testing program.', required=True)
    

    args = parser.parse_args()

    
    yaml = YAML()
    all_configs = read_all_configs(yaml)
    data_set_typ = all_configs["data_set_typ"]
    config = all_configs[data_set_typ]
    config["mbvio_folder"] = args.mbvio_folder
    config["results_folder"] = args.results_folder
    config["data_folder"] = args.data_folder
    config["program_name"] = args.program_name
    config["ground_truth_folder"] = args.data_folder + "/ground_truth_pose"


    all_configs[data_set_typ] = config

    with open('configs.yaml', 'w') as config_file:
        yaml.dump(all_configs, config_file)

if __name__ == "__main__":
    main()