import os
import json
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('path', type=str, help='output path')
args = parser.parse_args()

repo = '<path to repo>'

cols = ['gap', 'pass', 'method', 'run', 'traj_length', 'collisions', 'goal',
        'run0', 'run1', 'run2', 'run3', 'run4', 'run5', 'run6', 'run7', 'run8', 'run9', 'n_plans',
        'bfs_state_size', 'bfs_n_states', 'bfs_adj_list_compile', 'bfs_stack_compile', 'bfs_set_compile', 'bfs_max_stack_capacity',  
        'bfs_max_set_size', 'dfs_state_size', 'dfs_n_states', 'dfs_adj_list_compile', 'dfs_stack_compile', 'dfs_set_compile', 
        'dfs_max_stack_capacity', 'dfs_max_set_size',
        'virt_min', 'virt_max', 'virt_sum', 'res_min', 'res_max', 'res_sum', 
        'shr_min', 'shr_max', 'shr_sum', 'mem_perc_min', 'mem_perc_max', 'mem_perc_mean', 'mem_perc_median',
        'mem_total_min', 'mem_total_max', 'mem_total_mean', 'mem_total_median', 'mem_free_min', 'mem_free_max', 'mem_free_mean', 'mem_free_median',
        'mem_used_min', 'mem_used_max', 'mem_used_mean', 'mem_used_median', 'cache_min', 'cache_max', 'cache_mean', 'cache_median', 
        'swap_total_min', 'swap_total_max', 'swap_total_mean', 'swap_total_median',
        'swap_free_min', 'swap_free_max', 'swap_free_mean', 'swap_free_median', 
        'swap_used_min', 'swap_used_max', 'swap_used_mean', 'swap_used_median', 'process_uptime']

data = []

for run in os.listdir(repo):
    row = []
    with open(repo+'/{}/{}.json'.format(run, run)) as infile:
        d = json.load(infile)
     
    for col in cols:
        print(run)
        row.append(d[col])
        
    data.append(row)
            
df = pd.DataFrame(data, columns=cols);
df.to_csv('DATASET.csv')


        
    
        
    
    