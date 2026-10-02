import sys, c06_proto_sim as S
from multiprocessing import Pool
jobs=[(s,k,'pvt') for s in sys.argv[1].split(',') for k in range(int(sys.argv[2]))]
with Pool(12) as p:
    for r in p.map(S.run, jobs): print(r)
