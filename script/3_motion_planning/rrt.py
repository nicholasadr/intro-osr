'''
Next:
1) Create new graph that connect one node to another
2) Search algorithm
'''
import numpy as np
import matplotlib.pyplot as pl
import sys
sys.path.append('../../')
import env_2D
import graph
import math

#q = []
graphdict = {}

def generate_qrand(g):
  x_rand = np.random.rand()*env.size_x
  y_rand = np.random.rand()*env.size_y
  #check if generated q_rand is already generated before
  if not any(val==[x_rand,y_rand] for val in g.graph_dict.values()):
    if not env.check_collision(x_rand,y_rand):
      return [x_rand,y_rand]
    else:
      #generated point is in collision
      return [0,0]
  else:
    #point has been generated previously
    return [0,0]

def check_path_collision(x1,y1,x2,y2,d):
  if x1<x2:
    s1=1;r1=0;
  else: s1=-1;r1=1
  if y1<y2:
    s2=1
  else: s2=-1
  dx=(s1*(x2-x1))/d; dy=(s2*(y2-y1))/d
  x=((1-r1)*x1)+((r1)*x2)
  y=((1-r1)*y1)+((r1)*y2)
  if r1==0:
    if s2==1: sign=1
    else: sign=-1
  else:
    if s2==1: sign=-1
    else: sign=1
  for i in range(d-1):
   x+=dx; y+=sign*dy
   if env.check_collision(x,y):
     return True
   else:
     continue
  return False

def extend(g,c,x_start,y_start,q_rand_x,q_rand_y,radius):
  global solved
  next=0	#increment c if next==1

  #find q_near
  smallest=env.size_x+env.size_y
  for coord in g.graph_dict.values():
    x=coord[0][0]
    y=coord[0][1]
    d=math.sqrt(math.pow(x-q_rand_x,2)+math.pow(y-q_rand_y,2))
    if d<smallest:
      smallest=d
      q_near_x=x
      q_near_y=y

  #find q_new
  if smallest<radius:
    q_new_x = q_rand_x
    q_new_y = q_rand_y
  elif smallest>radius:
    dy=math.fabs(q_rand_y-q_near_y)
    dx=math.fabs(q_rand_x-q_near_x)
    angle=math.atan(dy/dx)
  
    #imagine q_near as a point in the origin, we take into account the possibilities of q_new being in any of the 4 quadrants
    if q_near_x<q_rand_x:
      if q_near_y<q_rand_y:
        x_sign=1; y_sign=1
      else:
        x_sign=1; y_sign=-1
    else:
      if q_near_y<q_rand_y:
        x_sign=-1; y_sign=1
      else:
        x_sign=-1;  y_sign=-1
  
    q_new_x = q_near_x + x_sign*(radius*math.cos(angle))
    q_new_y = q_near_y + y_sign*(radius*math.cos(angle))
  else:
    print "Failed to extend"
    return False
  
  #plot q_new
  if 0<q_new_x<env.size_x and 0<q_new_y<env.size_y:
    if not check_path_collision(q_near_x,q_near_y,q_new_x,q_new_y,20):
      #print "q_near: ",q_near_x,q_near_y
      #print "q_new: ",q_new_x,q_new_y
      next=1
      pl.plot([q_near_x,q_new_x],[q_near_y,q_new_y],'b-',linewidth=1.0)
      pl.show()
      pl.pause(0.001)
  
  if next:
    g.add_vertex(repr(c+1))
    g.add_edge(repr(c+1),[q_new_x,q_new_y])
    if check_goal(q_new_x,q_new_y,x_goal,y_goal,margin):
      solved=1
    return True
  else:
    return False

def check_goal(x,y,x_goal,y_goal,margin):
  d=math.sqrt(math.pow(x-x_goal,2)+math.pow(y-y_goal,2))
  if d<margin:
    return True
  else:
    return False


if __name__ == "__main__":

  #pl.ion()
  np.random.seed(4)
  env = env_2D.Environment(10, 6, 5)
  g = graph.Graph(graphdict)
  pl.clf()
  env.plot()

  q = env.random_query()
  if q is not None:
    x_start, y_start, x_goal, y_goal = q
    g.add_vertex('start')
    g.add_edge('start',[x_start,y_start])
  env.plot_query(x_start, y_start, x_goal, y_goal)
  
  c=0
  solved = 0
  radius = 2	#furthest possible distance between q_near and q_new
  margin = 0.2	#margin between goal coordinates and closest graph vertex
  while not solved:
    q_rand_x,q_rand_y = generate_qrand(g)
    if q_rand_x and q_rand_y:
      if extend(g,c,x_start,y_start,q_rand_x,q_rand_y,radius):
        c+=1

  pl.show(block=True)
