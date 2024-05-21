import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pdb

dfs = []
# for file in os.listdir('data'):
#     dfs.append(pd.read_csv(f'data/{file}'))

dfs.append(pd.read_csv('data/single_output.csv'))

fig, axs = plt.subplots(3, 1, sharex=True)
colors = cm.viridis(np.linspace(0, 1, len(dfs)))


for i, df in enumerate(dfs):
    axs[0].plot(df['t'], df['pos'], color=colors[i], alpha=0.7)
    axs[1].plot(df['t'], df['vel'], color=colors[i], alpha=0.7)
    axs[2].plot(df['t'], df['u'], color=colors[i], alpha=0.7)

axs[0].plot([dfs[0]['t'].min(), dfs[0]['t'].max()], [dfs[0]['pref'], dfs[0]['pref']], 'k--')
axs[1].plot([dfs[0]['t'].min(), dfs[0]['t'].max()], [0, 0], 'k--')

# axs[0].set_ylim(-1, 1)
axs[0].set_ylabel('Position (m)')
axs[1].set_ylabel('Velocity (m/s)')
axs[2].set_ylabel('Control (m/s^2)')
axs[2].set_xlabel('Time (s)')

plt.show()