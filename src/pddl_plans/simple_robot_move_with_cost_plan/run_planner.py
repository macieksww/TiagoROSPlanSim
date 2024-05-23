# script to run pddl planner and save planning results to db
import pandas as pd

start_state_df = {'X': [], 'Y': [], 'Battery':[], 'Planner': []}
start_state_df = pd.DataFrame(data=start_state_df)

start_state_df.head()