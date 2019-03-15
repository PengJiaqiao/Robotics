import pandas as pd

file = 'build/benchmark.csv'
header = ['Date', 'Time', 'Solved', 'Planner_Name', 'Vertices', 'Total_Queries', 'Free_Queries', 'Planner_Duration']
data = pd.read_csv(file,names=header)

avg_t = data[['Planner_Name', 'Planner_Duration']].groupby('Planner_Name').mean().rename(columns={'Planner_Duration':'avgT'}).T
std_t = data[['Planner_Name', 'Planner_Duration']].groupby('Planner_Name').std().rename(columns={'Planner_Duration':'stdT'}).T
avg_nodes = data[['Planner_Name', 'Vertices']].groupby('Planner_Name').mean().rename(columns={'Vertices':'avgNodes'}).T
avg_queries = data[['Planner_Name', 'Total_Queries']].groupby('Planner_Name').mean().rename(columns={'Total_Queries':'avgQueries'}).T
evaluation_table = pd.concat([avg_t,std_t,avg_nodes,avg_queries],).round(2)

print(evaluation_table)

evaluation_table.to_csv('build/evaluation_table.csv')