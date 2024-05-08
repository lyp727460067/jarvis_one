import argparse
import sys
import subprocess
from pathlib import Path
from datetime import datetime
from enum import Enum
from ruamel.yaml import YAML


class RunCommand:
    """Data for a command which should be run."""

    def __init__(self, command, working_dir, post_run_commands):
        """
        :param command: The main command which shall be run (MB-VIO execution).
        :param working_dir: The working directory to run it in.
        :param post_run_commands: Commands which should be run afterwards (e.g. moving the results to the correct
        places).
        """
        self.command = command
        self.working_dir = working_dir
        self.post_run_commands = post_run_commands

def save_code_info(repository_path, results_folder):
    # Get Git log
    git_hash = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repository_path).strip().decode('ascii')
    commit_message = subprocess.check_output(['git', 'log', '--format=%B', '-n', '1'],
                                             cwd=repository_path).strip().decode('ascii')
    # unfortunately the commit_time is in utc (and not in local time like the running time), but as we want to use it
    # for sorting mainly that should be okay.
    commit_time = datetime.utcfromtimestamp(
        int(subprocess.check_output(['git', 'log', '--format=%ct', '-n', '1'], cwd=repository_path).strip().decode(
            'ascii')))

    code_info={
        "commit_message": commit_message,
        "git_hash": git_hash,
        "commit_time": commit_time
    }

    result = {
        "code_info":code_info
    }

    yaml = YAML()
    with open(str(results_folder)+"/result.yaml", 'w') as result_file_handle:
        print("write---------------------------------------\n",code_info )
        yaml.dump(code_info, result_file_handle)


def read_all_configs(config_name=None, yaml=None):
    """Read all configs from configs.yaml"""
    name = 'configs.yaml'
    if config_name:
        name = config_name
    with open(name, 'r') as config_file:
        if yaml is None:
            yaml = YAML()
        all_configs = yaml.load(config_file)
    return all_configs

def main():
    # Read parameters (name, selected config, custom mbvio params.)
    parser = argparse.ArgumentParser(description='Run MB-VIO.')
    parser.add_argument('--config', type=str, default=None,
                        help="running config, specify which data sets to run")

    parser.add_argument('--results_folder', type=str,
                        help='The results of running MB-VIO will be stored in this folder.', required=True)

    args = parser.parse_args()
    print(args.config)
    # Read config.
    all_config = read_all_configs(args.config)
    if all_config is None:
        print('Error: config has to specified.')
        sys.exit(1)

    data_set_typ = all_config["data_set_typ"]
    config = all_config[data_set_typ]

    results_name, time_used_for_name = build_results_name(data_set_typ)

    mbvio_folder = config['mbvio_folder']
    mbvio_executable_path = mbvio_folder + '/build/bin/' + config['program_name']

    results_folder = Path(args.results_folder + "/" + results_name)
    all_config[data_set_typ]['results_folder']  = str(results_folder)
    yaml = YAML()
    with open(args.config, 'w') as config_file_handle:
        yaml.dump(all_config, config_file_handle)

    num_iter = config['iter']
    print('Num iterations: {}'.format(num_iter))
    only_seq = config['only_seq']

    # Create save folder
    if not results_folder.exists():
        results_folder.mkdir()
    else:
        print('WARNING: Results folder already exists.')
        sys.exit(1)

    # -> Create array of commands and working directories
    # -> For a normal script we can just run them one by one, for Slurm we need to write them to an sbatch file which
    # is then run.
    settings_file = config['program_config_file']
    if settings_file is None:
        print('do not have program file. ')
        sys.exit(1)

    commands = create_mbvio_commands(mbvio_executable_path, config['data_folder'], config['data_names'], results_folder, num_iter,
                                     only_seq, config['program_config_file'])

    #save code info
    save_code_info(mbvio_folder, results_folder)

    # ------------------------------ Run-Loop -> Run / create Slurm script. ------------------------------
    print("----------- STARTING EXECUTION! -----------")
    for command in commands:
        print(command)
        subprocess.run(command, shell=True)

    

def create_mbvio_commands(dmvio_executable, data_folder, data_names, results_folder, num_iter, only_seq, dmvio_settings_file,):

    commands = []
    for i, name in enumerate(data_names):
        for j in range(num_iter):
            if only_seq != -1:
                if i != only_seq:
                    continue
            #ex config datapath savename
            data_path = data_folder + '/' + name + '.bag'
            save_path = str(results_folder) + '/' + str(name) + "_" + str(j) + '.txt'
            command = "{} {} {} {}".format(dmvio_executable, dmvio_settings_file, data_path, save_path)
            commands.append(command)
    return commands

def build_results_name(dataset):
    # Build results name
    time = datetime.today()
    timestr = time.strftime('%Y-%m-%d--%H-%M-%S')
    results_name = ''
    if not dataset is None:
        results_name += dataset + '-'

    results_name += timestr
    print('Results-name: ', results_name)
    return results_name, time


if __name__ == "__main__":
    main()
