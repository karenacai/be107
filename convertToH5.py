from __future__ import print_function
import sys
import h5py
import numpy as np
#import multi_tracker_analysis as mta
import pandas
import matplotlib.pyplot as plt
#import scipy.io

def load_data_as_pandas_dataframe_from_hdf5_file(filename, attributes=None):
    if '.pickle' in filename:
        pd = pandas.read_pickle(filename)
        return pd
    try:
        data = h5py.File(filename, 'r', swmr=True)['data']
    except ValueError:
        data = h5py.File(filename, 'r', swmr=False)['data']

    if attributes is None:
        attributes = {   'objid'                : 'objid',
                         'time_epoch_secs'      : 'header.stamp.secs',
                         'time_epoch_nsecs'     : 'header.stamp.nsecs',
                         'position_x'           : 'position.x',
                         'position_y'           : 'position.y',
                         'measurement_x'        : 'measurement.x',
                         'measurement_y'        : 'measurement.y',
                         'velocity_x'           : 'velocity.x',
                         'velocity_y'           : 'velocity.y',
                         'angle'                : 'angle',
                         'frames'               : 'header.frame_id',
                         'area'                 : 'size',
                         }
    index = data['header.frame_id'].flat
    d = {}
    for attribute, name in attributes.items():
        print(name)
        d.setdefault(attribute, data[name].flat)
    pd = pandas.DataFrame(d, index=index)
    return d, pd
    #pd = pd.drop(pd.index==[0]) # delete 0 frames (frames with no data)
    #pd = calc_additional_columns(pd)
    #print(pd)
    # pd_subset = pd[pd.objid==key]

filename = sys.argv[1]
outfile = sys.argv[2]
d, pd = load_data_as_pandas_dataframe_from_hdf5_file(filename)
#print("returned")
#print(pd)
#f = h5py.File(filename,"r")
#print(d)
pd = pd.values
pd = np.array(pd)
#print(pd.shape)
#m_nonzero_rows = pd[[i for i, x in enumerate(pd) if pd.any()]]
#print(pd.shape)
ind_nz = np.where(pd[:,9] != 0)[0]
pd_concat = pd[ind_nz,:]
print(pd_concat.shape)

# 0: angle, 6:x_pos, 7:y_pos, 9:time
hf = h5py.File(outfile, 'w')
hf.create_dataset('time', data = pd_concat[:,9])
hf.create_dataset('pos_x', data = pd_concat[:,6])
hf.create_dataset('pos_y', data = pd_concat[:,7])
hf.create_dataset('angle', data = pd_concat[:,0])
hf.close()
