from solve_arm import *
for y in [-.18,-.20,-.22,-.24,-.26,-.28]:
 m.body('base').pos[1]=y
 q=np.array([-1.57,.5,1.1,1.4,-1.52]);errors=[]
 for target in [[0,-.12,.065],[0,-.065,.065],[0,-.065,.075],[0,-.07,.075],[0,-.07,.054],[0,-.02,.054]]:
  q,e,a=solve(target,q);errors.append([round(e*1000,2),round(a*180/np.pi,2)])
 print(y,errors,flush=True)
