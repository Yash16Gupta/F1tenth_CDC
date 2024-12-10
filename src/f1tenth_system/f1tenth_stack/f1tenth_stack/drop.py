import pandas as pd

# Step 1: Load the CSV file
df = pd.read_csv('/home/nvidia/new_rl.csv')

# Step 2: Drop the 3rd and 4th columns (indexing starts at 0, so columns 2 and 3)
df = df.drop(df.columns[[2, 3]], axis=1)

# Step 3: Save the modified DataFrame back to a new CSV file
df.to_csv('modified_file.csv', index=False)
