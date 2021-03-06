# -*- coding: utf-8 -*-
"""ENPM662_Proj2.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1cUAO4sgq_P-LNHZlmkFlyytAWLzJw2G7
"""
import math
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

theta1 = sp.Symbol('theta1')
theta2 = sp.Symbol('theta2')
theta3 = sp.Symbol('theta3')
a1 = 62.3
a2 = 118
a3 = 118
X_LF = 75
Y_LF = 102
Z_LF = 0
def inv_kinematic(x4, y4, z4, leg):
  D = (x4**2+y4**2-a1**2+z4**2-a2**2-a3**2)/(2*a2*a3)
  theta1 = -np.arctan2(-y4, x4)-np.arctan2(np.sqrt(x4**2+y4**2-a1**2), -a1)
  if leg == 1 or leg == 3:
    theta3 = np.arctan2(-np.sqrt(1-D**2), D) #Legs for 1 and 3
  else:
    theta3 = np.arctan2(np.sqrt(1-D**2), D) #Legs for 2 and 4
  theta2 = np.arctan2(z4, np.sqrt(x4**2+y4**2-a1**2))-np.arctan2(a3*np.sin(theta3), a2+a3*np.cos(theta3))
  return theta1/2/math.pi*360, theta2/2/math.pi*360, theta3/2/math.pi*360



T01 = sp.Matrix([[sp.cos(theta1), -sp.sin(theta1), 0, -a1*sp.cos(theta1)], 
              [sp.sin(theta1), sp.cos(theta1), 0, -a1*sp.sin(theta1)],
              [0, 0, 1, 0], 
              [0 , 0, 0, 1]])
T12 = sp.Matrix([[0, 0, -1, 0], 
              [-1, 0, 0, 0],
              [0, 1, 0, 0], 
              [0 , 0, 0, 1]])

T23 = sp.Matrix([[sp.cos(theta2), -sp.sin(theta2), 0, a2*sp.cos(theta2)],
              [sp.sin(theta2), sp.cos(theta2), 0, a2*sp.cos(theta2)],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
T34 = sp.Matrix([[sp.cos(theta3), -sp.sin(theta3), 0, a3*sp.cos(theta3)],
              [sp.sin(theta3), sp.cos(theta3), 0, a3*sp.sin(theta3)],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

#calculate Jacobian
T02 = T01 * T12
T03 = T02 * T23
T04 = T03 * T34
Z00 = sp.Matrix([[0], [0], [0]])
Z01 = sp.Matrix([[T01[0,2]],
              [T01[1,2]],
              [T01[2,2]]])
Z02 = sp.Matrix([[T02[0,2]],
              [T02[1,2]],
              [T02[2,2]]])
Z03 = sp.Matrix([[T03[0,2]],
              [T03[1,2]],
              [T03[2,2]]])
Z04 = sp.Matrix([[T04[0,2]],
              [T04[1,2]],
              [T04[2,2]]])
O0 = sp.Matrix([[0], [0], [0]])
O1 = sp.Matrix([[T01[0,3]],
              [T01[1,3]],
              [T01[2,3]]])
O2 = sp.Matrix([[T02[0,3]],
              [T02[1,3]],
              [T02[2,3]]])
O3 = sp.Matrix([[T03[0,3]],
              [T03[1,3]],
              [T03[2,3]]])
O4 = sp.Matrix([[T04[0,3]],
              [T04[1,3]],
              [T04[2,3]]])
O40 = O4-O0
O41 = O4-O1
O42 = O4-O2
O43 = O4-O3
J = sp.Matrix([[Z01[1]*O40[2]-Z01[2]*O40[1], Z02[1]*O41[2]-Z02[2]*O41[1], Z03[1]*O42[2]-Z03[2]*O42[1], Z04[1]*O43[2]-Z04[2]*O43[1]],
            [Z01[2]*O40[0]-Z01[0]*O40[2], Z02[2]*O41[0]-Z02[0]*O41[2], Z03[2]*O42[0]-Z03[0]*O42[2], Z04[2]*O43[0]-Z04[0]*O43[2]],
            [Z01[0]*O40[1]-Z01[1]*O40[0], Z02[0]*O41[1]-Z02[1]*O41[0], Z03[0]*O42[1]-Z03[1]*O42[0], Z04[0]*O43[1]-Z04[1]*O43[0]],
            [Z01, Z02, Z03, Z04]])
print(J)


q0 = sp.Matrix([[0], [-45], [90]])
for i in range(6):
  for j in range(4):
    J[i,j] = J[i,j].subs([(theta1,0),(theta2,-45),(theta3,90)])
    J[i,j] = J[i,j].subs([(sp.sin(90), math.sin(math.radians(90))), (sp.cos(90), math.cos(math.radians(90)))])
    J[i,j] = J[i,j].subs([(sp.sin(45), math.sin(math.radians(45))), (sp.cos(45), math.cos(math.radians(45)))])
print(J)


J_np = np.array(J, dtype='float')
J_inv = np.linalg.pinv(J_np)
print(J_inv)

#Get the velocity and inverse Jacobian to obtain the differential q
zeta = sp.Symbol("zeta")
vel = sp.Matrix([[0], [42*2*math.pi/5*sp.cos(zeta)], [42*2*math.pi/5*sp.sin(zeta)], [0],[0],[0]])
diff_q = J_inv * vel
print(diff_q)
diff_q.row_del(1)
print(diff_q)

#Plug the differential q into h function and draw the values on XYZ plane

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('X(mm)')
ax.set_ylabel('Y(mm)')
ax.set_zlabel('Z(mm)')
'''
plt.title("YZ frame")
plt.xlabel("Z(mm)")
plt.ylabel("Y(mm)")
'''
#Assume the time peroid dt is 0.1 for time from 0s to 5s
for t in np.arange(0,5,0.1):
  tmp_q = sp.Matrix([[0],[0],[0]])
  for j in range(3):
    tmp_q[j,0] = diff_q[j,0].subs([(sp.sin(zeta), math.sin(math.radians(72*t))), (sp.cos(zeta), math.cos(math.radians(72*t)))])
  #get the current q by adding q0 + diff_q*dt
  cur_q = q0 + tmp_q*0.1
  #get the position by plugging cur_q into h function
  tmp_h = sp.Matrix([[0],[0],[0]])
  for i in range(3):
    tmp_h[i,0] = O4[i,0].subs([(theta1,cur_q[0]),(theta2,cur_q[1]),(theta3,cur_q[2])])
  ax.scatter(np.asarray(tmp_h[0], dtype = "float"), np.asarray(tmp_h[1], dtype = "float"), np.asarray(tmp_h[2], dtype = "float"), c='blue')
  #plt.scatter(tmp_h[2],tmp_h[1])
  plt.pause(0.001)
plt.show()
