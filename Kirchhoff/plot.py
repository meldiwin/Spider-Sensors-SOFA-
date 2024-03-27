import numpy as np 
import sys
import pandas as pd
from pandas import DataFrame
import matplotlib.pyplot as plt 
from scipy.signal import savgol_filter
  


dataset = pd.read_csv('/home/marwaeldiwiny/Documents/empa02024/onl spirals/w.csv')

time = dataset.iloc[:,0].values
R1 = dataset.iloc[:,1].values
R2 = dataset.iloc[:,2].values
R3 = dataset.iloc[:,3].values
R4 = dataset.iloc[:,4].values
R5 = dataset.iloc[:,5].values
R6 = dataset.iloc[:,6].values



# Create a new figure and axis

plt.xlabel("time (min)", fontsize="16")
plt.ylabel("Resistance ohm", fontsize="16")


# Plot the first set of values on the first y-axis (blue)

plt.plot(time, R1, label = "R1", color='orange', linestyle='solid', marker='*')
plt.plot(time, R2, label = "R2", color='red', linestyle='solid', marker='*' )
plt.plot(time, R3, label = "R3", color='green', linestyle='solid', marker='*')
plt.plot(time, R4, label = "R4", color='fuchsia', linestyle='solid', marker='*')
plt.plot(time, R5, label = "R5", color='cyan', linestyle='solid', marker='*')
plt.plot(time, R6, label = "R6", color='blue', linestyle='solid', marker='*')



# Add legends for each dataset
#ax1.legend(loc='upper left')
plt.legend(loc='middle left', ncol=5, fontsize="12")

# Show the plot

plt.show()


	
