import os
import pickle

f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","datas_30Hz.pkl"), 'r')
datas = pickle.load(f)  
f.close()
print(datas)