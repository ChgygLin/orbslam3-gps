import matplotlib.pyplot as plt
import numpy as np

import pandas as pd

err=pd.read_csv("./TW_err.txt", names=['id','res', 'lat', 'lon', 'alt'], delimiter=' ')


#fig = plt.figure()
x=err['id'].values-err['id'][0]
res=err['res'].values
lat=err['lat'].values
lon=err['lon'].values
alt=err['alt'].values


avg_res=err['res'].mean()
avg_lat=err['lat'].mean()
avg_lon=err['lon'].mean()
avg_alt=err['alt'].mean()


plt.plot(res[:1000], label='error: '+ "{:.2f}".format(avg_res) + 'm' )
plt.plot(lat[:1000], color='r', label='lat error: '+ "{:.2f}".format(avg_lat) + 'm')
plt.plot(lon[:1000], color='g', label='lon error: '+ "{:.2f}".format(avg_lon) + 'm')
plt.plot(alt[:1000], color='b', label='alt error: '+ "{:.2f}".format(avg_alt) + 'm')


plt.legend()
plt.title('camera postion error')
plt.xlabel('image')
plt.ylabel('error(m)')
plt.show()