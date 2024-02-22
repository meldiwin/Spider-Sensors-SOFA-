
import numpy as np 
import sys
import pandas as pd
from pandas import DataFrame
import matplotlib.pyplot as plt 
from scipy.signal import savgol_filter
  



dataset = pd.read_csv('/Users/marwaeldiwiny/Documents/DIC_GRAPHES_Sep/Nature1/Rs.csv')

#R_6= dataset.iloc[:,7:8].values

time = dataset.iloc[:,0].values
R_0 = dataset.iloc[:,1].values
R_1 = dataset.iloc[:,2].values
R_2 = dataset.iloc[:,3].values
R_3 = dataset.iloc[:,4].values
R_4 = dataset.iloc[:,5].values
R_5 = dataset.iloc[:,6].values
# Apply Savitzky-Golay smoothing
window_length = 11  # Adjust the window length as needed
polyorder = 2        # Adjust the polynomial order as needed

## smoothing chs
R_0_smooth = savgol_filter(R_0, window_length, polyorder)
R_1_smooth = savgol_filter(R_1, window_length, polyorder)
R_2_smooth = savgol_filter(R_2, window_length, polyorder)
R_3_smooth = savgol_filter(R_3, window_length, polyorder)
R_4_smooth = savgol_filter(R_4, window_length, polyorder)
R_5_smooth = savgol_filter(R_5, window_length, polyorder)

fig, ax1 = plt.subplots() 

  
ax1.set_xlabel('time ($s$)') 
ax1.set_ylabel('R_0 ($\%$)', color = 'navy') 
plot1, = ax1.plot(time, R_0_smooth, color = 'navy', label="strain",  linestyle='dashed', marker='^') 

ax1.tick_params(axis ='y', labelcolor = 'navy') 


#### Adding Twin Axes

ax2 = ax1.twinx() 
  
ax2.set_ylabel('R_1 ($\Omega $) ', color = 'blue') 
plot2, = ax2.plot(time,R_1_smooth, color = 'red', label="R1", linestyle='dashdot')

ax2.tick_params(axis ='y', labelcolor = 'blue') 



ax3 = ax1.twinx() 

rspine = ax3.spines['right']
rspine.set_position(('axes', 1.1))
ax3.set_frame_on(True)
ax3.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax3.set_ylabel('R_2 ($\Omega $)', color = 'green') 
plot3, = ax3.plot(time, R_2_smooth, color = 'green',  label="R2", linestyle='dashdot') 

ax3.tick_params(axis ='y', labelcolor = 'green') 


ax4 = ax1.twinx() 

rspine = ax4.spines['right']
rspine.set_position(('axes', 1.17))
ax4.set_frame_on(True)
ax4.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax4.set_ylabel('R_3  ($\Omega $)', color = 'orange') 
plot4, =ax4.plot(time,R_3_smooth, color = 'orange', label="R3", linestyle='dashdot') 

ax4.tick_params(axis ='y', labelcolor = 'orange') 


ax5 = ax1.twinx() 

rspine = ax5.spines['right']
rspine.set_position(('axes', 1.25))
ax5.set_frame_on(True)
ax5.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax5.set_ylabel('R_4  ($\Omega$)', color = 'red') 
plot5, = ax5.plot(time,R_4_smooth, color = 'red',  label="R4", linestyle='dashdot') 

ax5.tick_params(axis ='y', labelcolor = 'red') 


ax6 = ax1.twinx() 

rspine = ax6.spines['right']
rspine.set_position(('axes', 1.34))
ax6.set_frame_on(True)
ax6.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax6.set_ylabel('R_5  ($\Omega $)', color = 'purple') 
plot6, = ax6.plot(time,R_5_smooth, color = 'purple',  label="R5", linestyle='dashdot') 

ax6.tick_params(axis ='y', labelcolor = 'purple') 


#ax7 = ax1.twinx() 

#rspine = ax7.spines['right']
#rspine.set_position(('axes', 1.42))
#ax7.set_frame_on(True)
#ax7.patch.set_visible(False)

#fig.subplots_adjust(right=0.7)

#ax7.set_ylabel('R_6  ($\Omega$)', color = 'fuchsia') 
#plot7, = ax7.plot(time_r,R_6, color = 'fuchsia', label="R6", linestyle='dashdot') 

#ax7.tick_params(axis ='y', labelcolor = 'fuchsia') 




plt.title('NatureI - All Resistances', fontsize= 12, fontname= 'serif')
plt.legend([plot1,plot2,plot3,plot4,plot5,plot6],["R_1", "R_2","R_3", "R_4", "R_5", "R_6"], loc= 'lower right')
plt.show()
plt.savefig('Ch4.png')





#plt.title('Radial-measurement-In-Full-Spider-60x60mm-DragonSkin30-Horizontal-Direction', fontsize= 12, fontname= 'serif')
#plt.legend([plot1,plot2,plot3,plot4],["strain", "R_1", "R_2", "R_3" ], loc= 'lower right')
#plt.show()
#plt.savefig('result.png')


