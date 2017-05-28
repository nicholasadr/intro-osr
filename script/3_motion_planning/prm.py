'''
Bug:
1) Sometimes path will cross obstacle region if argument 'd' in check_edge_collision is not large enough e.g. line 71
Next:
1) Optimise code running time
2) Accept terminal parameter for division within edge collision checking, radius size for edge creation, no of random plots to make, start and end coordinates
3) Add comments
'''

import numpy as np
import matplotlib.pyplot as pl
import sys
sys.path.append('../../')
import env_2D
import graph
import math

q_rand = []		#list to contain random coordinates
graphdict = {}		#graph dictionary

#generate random coordinates and append to q_rand list
def generate_rand(n):
  while len(q_rand)<(n):
    x_rand = np.random.rand()*env.size_x
    y_rand = np.random.rand()*env.size_y
    if [x_rand,y_rand] not in q_rand:
      if not env.check_collision(x_rand, y_rand):
        q_rand.append([x_rand,y_rand])
        pl.scatter(x_rand,y_rand)

#check if q_rand coordinate is within radius r of current coordinate
def check_radius(x1,y1,x2,y2,r):
  if math.sqrt(math.fabs(x2-x1))+math.sqrt(math.fabs(y2-y1))<math.sqrt(r):
    return True
  return False

#check if the proposed edges pass through collision
def check_edge_collision(x1,y1,x2,y2,d):
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

#build graph_dict representing graph of x,y coordinates
def list_2_graph(q_rand,g):
  for i in range(len(q_rand)): 
    #print "i: ",i
    g.add_vertex(repr(i+1))
    for j in range(len(q_rand)):
     #print "j: ",j
     if i==j:
       continue 
     if check_radius(q_rand[i][0],q_rand[i][1],q_rand[j][0],q_rand[j][1],4) and not check_edge_collision(q_rand[i][0],q_rand[i][1],q_rand[j][0],q_rand[j][1],20):
       #print "j+1: ",str(j+1)
       #print "i+1: ",i+1,g.graph_dict[repr(i+1)]
       #print "full_graph: ",g.graph_dict
       if not str(j+1) in g.graph_dict[repr(i+1)]:
         #print "adding"
         #g.add_edge({repr(i+1),repr(j+1)})
         g.add_edge(repr(i+1),repr(j+1))
         #print i+1,g.graph_dict[repr(i+1)]
         #print "full_graph2: ",g.graph_dict
         #print " "
         #print(g.edges())

#visualize edges
def draw_edges(q_rand,g):
  for i in range(len(g.edges())):
    a = g.edges()[i]
    pl.plot([q_rand[int(list(a)[0])-1][0],q_rand[int(list(a)[1])-1][0]],[q_rand[int(list(a)[0])-1][1],q_rand[int(list(a)[1])-1][1]],'g--')

#finding path
def find_path(x1,y1,x2,y2,q_rand,g):
  if (env.check_collision(x1,y1) or env.check_collision(x2,y2)):
    print "Coordinates in obstacle region"
    return False
  else:
    s=0;e=0;r_start=env.size_x+env.size_y;r_end=env.size_x+env.size_y
    for i in range(len(q_rand)):
      x=q_rand[i][0]; y=q_rand[i][1]
      r_s = (math.sqrt(math.fabs(x-x1)))+(math.sqrt(math.fabs(y-y1)))
      r_e = (math.sqrt(math.fabs(x-x2)))+(math.sqrt(math.fabs(y-y2)))
      if r_s<r_start:
        r_start=r_s
        s=i
      if r_e<r_end:
        r_end=r_e
        e=i
    #print "s+1: ",s+1
    #print "e+1: ",e+1
    pl.scatter(q_rand[s][0],q_rand[s][1],c='red',zorder=5)
    pl.scatter(q_rand[e][0],q_rand[e][1],c='red',zorder=10)

    roadmap = g.find_path(g.graph_dict,str(s+1),str(e+1))
    print roadmap
    
    for i in range(len(roadmap)-1):
      pl.plot([q_rand[int(roadmap[i])-1][0],q_rand[int(roadmap[i+1])-1][0]],[q_rand[int(roadmap[i])-1][1],q_rand[int(roadmap[i+1])-1][1]],'.b-',linewidth=2.0)

if __name__ == "__main__":

  #pl.ion()
  np.random.seed(4)
  env = env_2D.Environment(10,6,5)
  g = graph.Graph(graphdict)
  pl.clf()
  env.plot()

  generate_rand(50)
  list_2_graph(q_rand,g)
  draw_edges(q_rand,g)
  
  #print "full graph: "
  #print(g.graph_dict)

  #finding shortest path
  x_start=3.311; y_start=3.432
  x_end=9.516; y_end=4.395
  find_path(x_start,y_start,x_end,y_end,q_rand,g)

  #pl.plot([1,3],[1,3],'.g-')
  #q = env.random_query()

  #if q is not None:
  #  x_start, y_start, x_goal, y_goal = q
  #  env.plot_query(x_start, y_start, x_goal, y_goal)

  pl.show(block=True)
  pl.pause(0.0001)

