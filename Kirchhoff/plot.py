import numpy as np 
import sys
import pandas as pd
from pandas import DataFrame
import matplotlib.pyplot as plt 
from scipy.signal import savgol_filter
  


dataset = pd.read_csv('/home/marwaeldiwiny/Documents/sofa plots/one_stage.csv')

strain = dataset.iloc[:,0].values
R14 = dataset.iloc[:,1].values
R12 = dataset.iloc[:,2].values
R25 = dataset.iloc[:,3].values
R36 = dataset.iloc[:,4].values
R34 = dataset.iloc[:,5].values
R32 = dataset.iloc[:,6].values
R16 = dataset.iloc[:,7].values
R56 = dataset.iloc[:,8].values
R54 = dataset.iloc[:,9].values



# Create a new figure and axis

plt.xlabel("strain (%)", fontsize="16")
plt.ylabel("Resistance ohm", fontsize="16")


# Plot the first set of values on the first y-axis (blue)

plt.plot(strain, R14, label = "R14", color='black', linestyle='solid')
plt.plot(strain, R12, label = "R12", color='blue', linestyle='solid' )
plt.plot(strain, R25, label = "R25", color='yellow', linestyle='solid')
plt.plot(strain, R36, label = "R36", color='black', linestyle='dashed')
plt.plot(strain, R34, label = "R34", color='red', linestyle='dashed')
plt.plot(strain, R32 , label = "R32", color='green', linestyle='solid')
plt.plot(strain, R16, label = "R16", color='red', linestyle='solid')
plt.plot(strain, R56, label = "R56", color='blue', linestyle='dashed')
plt.plot(strain, R54, label = "R54", color='green', linestyle='dashed')



# Add legends for each dataset
#ax1.legend(loc='upper left')
plt.legend(loc='middle left', ncol=5, fontsize="12")

# Show the plot

plt.show()


	
