import matplotlib.pyplot as plt
import numpy as np
import sys
import os

import pandas as pd

if os.path.exists('./png') is not True:
    os.mkdir('./png')

err_file=sys.argv[1]
file_name=err_file.split('/')[-1].split('.')[0]

err=pd.read_csv(err_file, names=['id','res', 'lat', 'lon', 'alt'], delimiter=' ')

iend=300

#fig = plt.figure()
x=err['id'].values-err['id'][0]
res=err['res'][:iend].values
lat=err['lat'][:iend].values
lon=err['lon'][:iend].values
alt=err['alt'][:iend].values


avg_res=res.mean()
avg_lat=lat.mean()
avg_lon=lon.mean()
avg_alt=alt.mean()

plt.plot(res[:iend], label='error: '+ "{:.2f}".format(avg_res) + 'm' )
plt.plot(lat[:iend], color='r', label='lat error: '+ "{:.2f}".format(avg_lat) + 'm')
plt.plot(lon[:iend], color='g', label='lon error: '+ "{:.2f}".format(avg_lon) + 'm')
plt.plot(alt[:iend], color='b', label='alt error: '+ "{:.2f}".format(avg_alt) + 'm')


plt.legend()
plt.title('camera postion error: ' + file_name)
plt.xlabel('image')
plt.ylabel('error(m)')

plt.savefig('png/'+file_name+'.png')
plt.show()
