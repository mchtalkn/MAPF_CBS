from cbs import cbs
import time
import multiprocessing as mp
import glob
import argparse
from pathlib import Path
from datetime import datetime
from common_search_funcs import *


def foo(x=1):
    cnt = 1
    while True:
        time.sleep(1)
        print(x, cnt)
        cnt += 1


def timeout(func, timeout_args=(), kwds=None, timeout=1, default=None):
    if kwds is None:
        kwds = {}
    pool = mp.Pool(processes=1)
    result = pool.apply_async(func, args=timeout_args, kwds=kwds)
    try:
        val = result.get(timeout=timeout)
    except mp.TimeoutError:
        pool.terminate()
        return default
    else:
        pool.close()
        pool.join()
        return val


def import_bm_map(filename):
    file_path = Path(filename)
    if not file_path.is_file():
        raise BaseException(filename + " does not exist.")
    input_file = open(filename, 'r')
    # first line: #rows #columns
    read_line = input_file.readline()
    read_line = input_file.readline()
    rows = read_line.split()[1]
    rows = int(rows)
    columns = input_file.readline()
    columns = read_line.split()[1]
    columns = int(columns)
    read_line = input_file.readline()
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        read_line = input_file.readline()
        my_map.append([])
        for cell in read_line:
            if cell == '@':
                my_map[-1].append(True)
            elif not cell.isspace():
                my_map[-1].append(False)
            # else:
            #    my_map[-1].append(False)
    # #agents
    read_line = input_file.readline()
    input_file.close()
    # print(my_map)
    return my_map


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs MAPF benchmarks')
    parser.add_argument('--map_dir', type=str, default='benchmarks/maps',
                        help='path of directory that maps are located')
    parser.add_argument('--scens', type=str, default='benchmarks/scen-sample/*.scen',
                        help='list of scenarios to be tested')
    args = parser.parse_args()

    files = sorted(glob.glob(args.scens))
    print(files)
    now = datetime.now()
    dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
    result_file = open(f"excels/benchmark_{dt_string}.csv", 'w+', buffering=1)
    max_agents = ['']
    for sc in files:
        node_count = 0
        i = 0
        f = Path(sc)
        map_name = sc.split('/')[-1].split('-even')[0].split('-random')[0]
        mf = Path(args.map_dir + '/' + map_name + '.map')
        if not mf.is_file():
            raise BaseException(args.map_dir + map_name + '.map' + " does not exist.")
        bm_map = import_bm_map(mf)
        if not f.is_file():
            raise BaseException(sc + " does not exist.")
        f = open(sc, 'r')

        starts = []
        goals = []
        for line in f.readlines()[1:]:
            _, _, _, _, sy, sx, gy, gx, _ = line.split()
            starts.append((int(sx), int(sy)))
            goals.append((int(gx), int(gy)))
            print(f'map:{mf},agent_count:{len(starts)}')
            if bm_map[int(sx)][int(sy)] or bm_map[int(gx)][int(gy)]:
                print('unreachable start/end point')
                break
            start_time = time.time()
            res = timeout(cbs, timeout_args=(bm_map, starts, goals), timeout=30, default=None)
            end_time = time.time()
            time_diff = start_time - end_time
            if res:
                paths, node_count = res
                cost = get_sum_of_cost(paths)
                i += 1
            else:
                print(f'\n\n!!!map:{mf},max_agent_count:{len(starts)}!!!\n\n')
                result_file.write(f"{sc},{i},{node_count},!\n")
                max_agents.append(str(i))
                break
    result_file.write('!!!\n')
    result_file.write(','.join(max_agents))
    result_file.write('\n')
    # print(timeout(foo, kwds = {'x': 'Hi'}, timeout = 3, default = 'Bye'))
    # print(timeout(foo, args = (2,3), timeout = 2, default = 'Sayonara'))
