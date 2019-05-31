#!/usr/bin/env python
# coding=utf-8
import math
import numpy as np
from numpy import linalg as LA
import rospy
from std_msgs.msg import String
from dwmodule.msg import distances

D=np.array([1,1,1,1])
first = False

def callback(data):
	global D, first

	if not first:
		first = True

	D[0] = data.d1
	D[1] = data.d2
	D[2] = data.d3
	D[3] = data.d4


def null(a, rtol=1e-5):
    u, s, v = np.linalg.svd(a)
    rank = (s > rtol*s[0]).sum()
    return rank, v[rank:].T.copy()


def Trilateration(P, D, W):
	mp,npl = P.shape #DUDA: por qué sacar estos valores si son fijos
	ns = len(D)
	if ns is not npl:
		print('Number of reference point and distances are different')
		return 0,0
	
	a = np.empty((0,4))
	b = np.empty((0,1))
	for i1 in xrange(0,npl):
		x = P.item(0,i1)
		y = P.item(1,i1)
		z = P.item(2,i1)
		s = D.item(i1)
		a = np.concatenate((a,[[1,-2*x,-2*y,-2*z]]),axis=0)
		b = np.concatenate((b,[[s**2-x**2-y**2-z**2]]),axis=0)

	if npl is 3:
		Xp = np.matmul(LA.pinv(a),b)
		xp = Xp[1:4,:]
		rank, Z = null(A)
		z = Z[1:4,:]

		if rank is 3:
			a2 = z.item(0)**2 + z.item(1)**2 + z.item(2)**2
			a1 = 2*(z.item(0)*xp.item(0) + z.item(1)*xp.item(1) + z.item(2)*xp.item(2))-Z.item(0)
			a0 = xp.item(0)^2 +  xp.item(1)^2+  xp.item(2)^2-Xp.item(0)
			p = np.array([a2 ,a1 ,a0])
			t = np.roots(p)
			N1 = Xp + t.item(0)*Z
			N2 = Xp + t.item(1)*Z
			return N1, N2

	if npl>3:
		if not np.allclose(W, np.diag(np.ones(len(W)))):
			C = LA.inv(W)*W
			Xpdw=LA.multi_dot([LA.inv(LA.multi_dot([LA.inv(A),C,A])),LA.inv(A),C,b])
		else:
			Xpdw=np.dot(LA.pinv(A),b)

		N1 = Xpdw
		N2 = N1

		return N1, N2



def main():

	global D, first

	rospy.init_node('trilat',anonymous=False)
	rospy.Subscriber("topicdwm",distances,callback)


	P0 = np.array ([[0], [0], [0.98]])
	P1 =np.array([[6.62], [-1.88], [2.21]])
	P2 =np.array([[12.4],  [2.04] , [0.81]])
	P3 =np.array([[5.62],  [5.82] , [1.80]])

	P=np.concatenate((P0,P1,P2,P3),axis = 0)

	#Posicion real del tag
	N=np.array([[7.5], [1.5], [0.93]])

	#Distancias reales entre anchors y tag
	dd0=LA.norm(P0-N)
	dd1=LA.norm(P1-N)
	dd2=LA.norm(P2-N)
	dd3=LA.norm(P3-N)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		if first:
			W = np.diag(np.ones(len(D)))
			N1, N2 = Trilateration(P,D,W)
			print "hey"
		rate.sleep()


	#[D]=dwm_distance_get()
	#dd = np.array([dd0, dd1,dd2,dd3])
	#print("Distancias reales \n",dd)
	#print("Distancias medidas por módulos \n",D)

	#print("Posiciones de Anchors \n",P)

	#print("---Resultado trilat_dwm --- \n")

	#W = np.diag(np.ones(len(D)))

	#N1, N2 = Trilateration(P,D,W)

	#Nsol1 = N1(2:4,1)

	#print(Nsol1)
	#N1,N2 = Trilateration(P,3,2)
'''
	a=[]
	for i in range(5):    
	    a.append(i)
	print a
'''

if __name__ == '__main__':
	main()