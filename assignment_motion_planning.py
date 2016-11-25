import numpy as np
import openravepy as orpy
env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('envs/pick_and_place.xml')

def create_box(T, color = [0, 0.6, 0]):
    box = orpy.RaveCreateKinBody(env, '')
    box.SetName('box')
    box.InitFromBoxes(np.array([[0,0,0,0.035,0.03,0.005]]), True)    
    g = box.GetLinks()[0].GetGeometries()[0]
    g.SetAmbientColor(color)
    g.SetDiffuseColor(color)
    box.SetTransform(T)
    env.Add(box,True)
    return box

T = np.eye(4)
container_center = np.array([0.4, 0.2, 0.195])

# Destination
T[:3, 3] = container_center + np.array([0, -0.5, 0])
destination0 = create_box(T, color = [0, 0, 0.6])
T[:3, 3] = container_center + np.array([0, -0.6, 0])
destination1 = create_box(T, color = [0, 0, 0.6])

# Generate random box positions
boxes = []
nbox_per_layer = 2
n_layer = 20
h = container_center[2]

for i in range(n_layer):
    nbox_current_layer = 0
    while nbox_current_layer < nbox_per_layer:
        theta = np.random.rand()*np.pi
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        T[0, 3] = container_center[0] + (np.random.rand()-0.5)*0.2
        T[1, 3] = container_center[1] + (np.random.rand()-0.5)*0.1
        T[2, 3] = h        
        box = create_box(T)
        if env.CheckCollision(box):
            env.Remove(box)
        else:
            boxes.append(box)
            nbox_current_layer += 1
    h += 0.011


# Enter your code below

