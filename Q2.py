import numpy as np
import matplotlib.pyplot as plt

#control	1--N	2--S	3--E	4--W
#return the (stage cost, target node) i.e.(l(x,u),x') pair
def cost_motion(node,control):
	if(node==2):
		return -10,22
	if(node==4):
		return -5,14
	if(control==1):
		if (node==1 or node==3 or node==5):
			return 1,node
		else:
			return 0,node-5
	elif (control==2):
		if (node==21 or node==22 or node==23 or node==24 or node==25):
			return 1,node
		else:
			return 0,node+5
	elif (control==3):
		if (node==5 or node==10 or node==15 or node==20 or node==25):
			return 1,node
		else:
			return 0,node+1
	elif (control==4):
		if (node==1 or node==6 or node==11 or node==16 or node==21):
			return 1,node
		else:
			return 0,node-1
	return

#visualize the result
def visualize(V, policy):
	# visualize the value function
	V = np.reshape(V,(5,5))
	fig, ax = plt.subplots()
	ax.matshow(V, cmap='seismic')
	for (i, j), z in np.ndenumerate(V):
		ax.text(j, i, '{:0.1f}'.format(z), ha='center', va='center',
				bbox=dict(boxstyle='round', facecolor='white', edgecolor='0.3'))
	plt.title('V*')
	plt.show()

	# visualize the policy
	policy = np.reshape(policy,(5,5))
	fig, ax = plt.subplots()
	ax.set_xticks(np.arange(0, 5, 1))
	ax.set_yticks(np.arange(-5, 0, 1))
	x_pos = []
	y_pos = []
	x_direct = []
	y_direct = []
	for i in range(5):
		for j in range(5):
			curr = policy[i,j]
			x_pos += [j+0.5]
			y_pos += [-i-0.5]
			if curr == 1:
				x_direct+=[0]
				y_direct+=[1]
			elif curr==2:
				x_direct+=[0]
				y_direct+=[-1]
			elif curr==3:
				x_direct+=[1]
				y_direct+=[0]
			elif curr==4:
				x_direct+=[-1]
				y_direct+=[0]
	ax.quiver(x_pos,y_pos,x_direct,y_direct)
	plt.grid()
	plt.title('$\pi$*')
	plt.show()

#Gauss-Seidel Value Iteration
def gauss_value_iteration(gamma,threshold):
	#Initialize value and policy function
	control = np.ones(25)
	Vk_0 = np.zeros(25)
	q = 0
	while (True):
		q+=1
		Vk_1 = np.copy(Vk_0)
		for i in range (1,26):
			N_cost,N_goal = cost_motion(i,1)
			S_cost,S_goal = cost_motion(i,2)
			E_cost,E_goal = cost_motion(i,3)
			W_cost,W_goal = cost_motion(i,4)
			a = np.array([	N_cost+gamma*Vk_0[N_goal-1],
							S_cost+gamma*Vk_0[S_goal-1],
							E_cost+gamma*Vk_0[E_goal-1],
							W_cost+gamma*Vk_0[W_goal-1]])
			Vk_0[i-1] = np.amin(a)
			control[i-1] = np.argmin(a)+1
		if (np.amax(abs(Vk_0-Vk_1)) < threshold):
			break
	print('Total number of iterations = '+str(q))
	visualize(Vk_0,control)
	return

#Regular Value Iteration
def value_iteration(gamma,threshold):
	#Initialize value and policy function
	control = np.ones(25)
	Vk_0 = np.zeros(25)
	q = 0
	while (True):
		q+=1
		Vk_1 = np.copy(Vk_0)
		for i in range (1,26):
			N_cost,N_goal = cost_motion(i,1)
			S_cost,S_goal = cost_motion(i,2)
			E_cost,E_goal = cost_motion(i,3)
			W_cost,W_goal = cost_motion(i,4)
			a = np.array([	N_cost+gamma*Vk_0[N_goal-1],
							S_cost+gamma*Vk_0[S_goal-1],
							E_cost+gamma*Vk_0[E_goal-1],
							W_cost+gamma*Vk_0[W_goal-1]])
			Vk_1[i-1] = np.amin(a)
			control[i-1] = np.argmin(a)+1
		if (np.amax(abs(Vk_0-Vk_1)) < threshold):
			break
		Vk_0 = np.copy(Vk_1)
	print('Total number of iterations = '+str(q))
	visualize(Vk_0,control)
	return


#Policy Iteration
def policy_iteration(gamma,threshold):
	#Initialize value and policy function
	control = np.ones(25)
	Vk_0 = np.zeros(25)
	Vk_1 = np.zeros(25)
	q = 0
	while (True):
		q+=1
		Vk_odd = np.copy(Vk_0)
		#Policy Evaluation
		while(True):
			for i in range(1,26):
				cost,goal = cost_motion(i,control[i-1])
				Vk_1[i-1] = cost+gamma*Vk_0[goal-1]

			if (np.amax(abs(Vk_0-Vk_1)) < threshold):
				break
			Vk_0 = np.copy(Vk_1)
		
		#Policy Improvement
		for i in range (1,26):
			N_cost,N_goal = cost_motion(i,1)
			S_cost,S_goal = cost_motion(i,2)
			E_cost,E_goal = cost_motion(i,3)
			W_cost,W_goal = cost_motion(i,4)
			a = np.array([	N_cost+gamma*Vk_0[N_goal-1],
							S_cost+gamma*Vk_0[S_goal-1],
							E_cost+gamma*Vk_0[E_goal-1],
							W_cost+gamma*Vk_0[W_goal-1]])
			control[i-1] = np.argmin(a)+1

		if (np.amax(abs(Vk_odd-Vk_0)) < threshold):
			break

	print('Total number of iterations = '+str(q))
	visualize(Vk_0,control)
	return

def Qvalue_iteration(gamma,threshold):
	#Initialize value and policy function
	control = np.ones(25)
	Q_0 = np.zeros((25,4))
	Q_1 = np.zeros((25,4))
	q = 0
	while (True):
		q+=1
		Q_1 = np.copy(Q_0)
		for i in range(1,26):
			for j in range(4):
				cost,goal = cost_motion(i,j+1)
				min_Q_value = np.amin(Q_0[goal-1,:])
				Q_0[i-1,j] = cost + gamma*min_Q_value
		if (np.amax(abs(Q_0-Q_1)) < threshold):
			break
	V = np.amin(Q_0,axis=1)
	policy = np.argmin(Q_0,axis=1) + 1
	print('Total number of iterations = '+str(q))
	visualize(V,policy)
	return
	
if __name__ == '__main__':
	threshold = 0.00001		#threshold to determine convergence i.e. max|V'(x) - V(x)| < threshold
	gamma = 0.9				#discount factor
	#gamma = 0.8
	#gamma = 0.99
	#gauss_value_iteration(gamma,threshold)
	#value_iteration(gamma,threshold)
	#policy_iteration(gamma,threshold)
	Qvalue_iteration(gamma,threshold)