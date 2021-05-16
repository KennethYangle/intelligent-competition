import os
import pickle

f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","CEP_raw.pkl"), 'r')
CEP_raw = pickle.load(f)  
f.close()
print(CEP_raw)