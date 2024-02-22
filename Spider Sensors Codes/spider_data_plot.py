

import numpy as np 
import sys
import pandas as pd
from pandas import DataFrame
import matplotlib.pyplot as plt 
  




dataset = pd.read_csv('/home/marwaeldiwiny/python_plot/EMPA _June/July27/printed_spider_inside_DragonSkin30/Horizontal_Direction/50 mm separation after relaxation/exp2/printed_spider_horizontal_direction_50mm_after resting.csv')

R_6= dataset.iloc[:,7:8].values

R_5= dataset.iloc[:,6:7].values

R_4= dataset.iloc[:,5:6].values

R_3= dataset.iloc[:,4:5].values

R_2= dataset.iloc[:,3:4].values

R_1= dataset.iloc[:,2:3].values

time_r= dataset.iloc[:,1:2].values






time, strain = np.loadtxt('/home/marwaeldiwiny/python_plot/EMPA _June/July27/printed_spider_inside_DragonSkin30/Horizontal_Direction/50 mm separation after relaxation/exp2/strain.txt',delimiter=',',unpack=True)





fig, ax1 = plt.subplots() 

  
ax1.set_xlabel('time ($s$)') 
ax1.set_ylabel('strain ($\%$)', color = 'navy') 
plot1, = ax1.plot(time, strain, color = 'navy', label="strain",  linestyle='dashed', marker='^') 

ax1.tick_params(axis ='y', labelcolor = 'navy') 



  
#### Adding Twin Axes

ax2 = ax1.twinx() 
  
ax2.set_ylabel('R_1 ($\Omega $) ', color = 'blue') 
plot2, = ax2.plot(time_r,R_1, color = 'blue', label="R1", linestyle='dashdot')

ax2.tick_params(axis ='y', labelcolor = 'blue') 



ax3 = ax1.twinx() 

rspine = ax3.spines['right']
rspine.set_position(('axes', 1.1))
ax3.set_frame_on(True)
ax3.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax3.set_ylabel('R_2 ($\Omega $)', color = 'green') 
plot3, = ax3.plot(time_r, R_2, color = 'green',  label="R2", linestyle='dashdot') 

ax3.tick_params(axis ='y', labelcolor = 'green') 


ax4 = ax1.twinx() 

rspine = ax4.spines['right']
rspine.set_position(('axes', 1.17))
ax4.set_frame_on(True)
ax4.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax4.set_ylabel('R_3  ($\Omega $)', color = 'orange') 
plot4, =ax4.plot(time_r,R_3, color = 'orange', label="R3", linestyle='dashdot') 

ax4.tick_params(axis ='y', labelcolor = 'orange') 


ax5 = ax1.twinx() 

rspine = ax5.spines['right']
rspine.set_position(('axes', 1.25))
ax5.set_frame_on(True)
ax5.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax5.set_ylabel('R_4  ($\Omega$)', color = 'red') 
plot5, = ax5.plot(time_r,R_4, color = 'red',  label="R4", linestyle='dashdot') 

ax5.tick_params(axis ='y', labelcolor = 'red') 


ax6 = ax1.twinx() 

rspine = ax6.spines['right']
rspine.set_position(('axes', 1.34))
ax6.set_frame_on(True)
ax6.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax6.set_ylabel('R_5  ($\Omega $)', color = 'purple') 
plot6, = ax6.plot(time_r,R_5, color = 'purple',  label="R5", linestyle='dashdot') 

ax6.tick_params(axis ='y', labelcolor = 'purple') 


ax7 = ax1.twinx() 

rspine = ax7.spines['right']
rspine.set_position(('axes', 1.42))
ax7.set_frame_on(True)
ax7.patch.set_visible(False)

fig.subplots_adjust(right=0.7)

ax7.set_ylabel('R_6  ($\Omega$)', color = 'fuchsia') 
plot7, = ax7.plot(time_r,R_6, color = 'fuchsia', label="R6", linestyle='dashdot') 

ax7.tick_params(axis ='y', labelcolor = 'fuchsia') 




plt.title('Printed Full spider-60x60mm-DragonSkin30-Horizontal-Direction_50 mm separation grip', fontsize= 12, fontname= 'serif')
plt.legend([plot1,plot2,plot3,plot4,plot5,plot6,plot7],["strain", "R_1", "R_2", "R_3", "R_4", "R_5", "R_6" ], loc= 'lower right')
plt.show()
plt.savefig('result.png')





#plt.title('Radial-measurement-In-Full-Spider-60x60mm-DragonSkin30-Horizontal-Direction', fontsize= 12, fontname= 'serif')
#plt.legend([plot1,plot2,plot3,plot4],["strain", "R_1", "R_2", "R_3" ], loc= 'lower right')
#plt.show()
#plt.savefig('result.png')


