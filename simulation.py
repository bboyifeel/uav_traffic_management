import numpy as np
import networkx as nx
import multinetx as mx
import matplotlib.pyplot as plt
import math
from datetime import datetime
import time
import copy


# N number of nodes for each layer
N					= 10
NUMBER_OF_DRONS 	= 10
# MAX_VELOCITY 		= [15, 25, 42]   	km/h 
MAX_VELOCITY		= [ 4.2, 7, 11.7 ] # m/s
START_POSITION			= 0
DESTINATION_POSITION	= 8
SLOWER 					= [1, 0.8, 0.6]  # slowering constant

RULE_LOW			= 3
RULE_HIGH			= 4

class UAV(object):
	# rule:
	# 		1 - fixed with best time calculated at the beginning (selfish blunt time)
	#		2 - fixed with best energy calculated at the beginning (selfish blunt energy)
	#		3 - optimize energy
	#		4 - optimize time
	def __init__(self, sourcePosition, destination, shortPath, rule):
		self.sourcePosition = sourcePosition
		self.path = [sourcePosition]

		self.energy = 0
		self.time = 0

		self.qCount = 0  # queueing/hovering count
		self.rule = rule
		self.destination = destination
		self.shortPath = shortPath

	def setNewPath(self, newShortPath):
		self.shortPath = newShortPath
		return self.shortPath

	def getShortPath(self):
		return self.shortPath

	def setRule(self, newRule):
		self.rule = newRule
		return self.rule

	def getRule(self):
		return self.rule

	def getPosition(self):
		return self.path[-1]

	def getDestination(self):
		return self.destination

	def setPosition(self, node):
		self.path.append(node)

	def getPath(self):
		return self.path

	def addEnergy(self, energy):
		self.energy += energy
		return energy

	def addTime(self, time):
		self.time += time
		return time

	def getTime(self):
		return self.time

	def getEnergy(self):
		return self.energy

	def hover(self, energy, time):
		self.addEnergy(energy)  # it should be the power (when u = 0)
		self.addTime(time)
		self.qCount += 1
		return self.qCount

	def getCount(self):
		return self.qCount


class Register(object):
	def __init__(self, phlist):
		self.phlist = phlist

	def objectsInTheLink(self):
		counter = 0
		timestamp = datetime.timestamp(datetime.now())
		print("I'm checking how many is in the link and now is " + str(timestamp))
		toRemove = []
		for i in self.phlist:
			print(i)
			if timestamp > i:
				toRemove.append(i)
			else:
				counter = counter + 1
		
		for i in toRemove:
			self.phlist.remove(i)

		print(str(counter) + " UAVs are in the link")
		return counter


####################################################################################
#   Functions
####################################################################################
# Functions for generating traffic, moving and calculating energy, time,
# hovering/queuing count and layer switching
# Generate the list of drones >>> traffic generator
def generateDrone(graph, number, position, destination):
	dronesList = []
	shortPathT = nx.shortest_path(graph, source=position, target=destination, weight='time')
	shortPathE = nx.shortest_path(graph, source=position, target=destination, weight='energy')
	for n in range(number):  # check
		rule = np.random.randint(low=RULE_LOW, high=RULE_HIGH, size=1)[0]
		if rule == 1:
			d = UAV(position, destination, shortPathT, 1)
		elif rule == 2:
			d = UAV(position, destination, shortPathE, 2)
		elif rule == 3:
			d = UAV(position, destination, shortPathT, 3)
		else:
			d = UAV(position, destination, shortPathE, 4)
		dronesList.append(d)
	return dronesList

# Function that defines the movement of the drones given list of instances of the UAV class
# all drones move at the same time (with this code)
def droneMove(graph, dronesList):
	while True:
		# Loop through the list of drones and for every drone reached destination
		flag = True
		for drone in dronesList:
			destination = drone.getDestination()
			if drone.getPosition() != destination:
				flag = False
				break
		if flag:
			break

		for drone in dronesList:
			AttributesmVelocity	= nx.get_edge_attributes(graph, 'maxVelocity')
			AttributesCapacity	= nx.get_edge_attributes(graph, 'capacity')
			AttributesDistance	= nx.get_edge_attributes(graph, 'distance')
			AttributesTime		= nx.get_edge_attributes(graph, 'time')
			AttributesEnergy	= nx.get_edge_attributes(graph, 'energy')
	
			pos = drone.getPosition()  # present position
			destination = drone.getDestination()
			if (pos == destination):
				continue

			path = drone.getPath()  # Way drone moved
			rule = drone.getRule()  # Drone rule
			
			shortPath = drone.getShortPath()  # Shortest Path to reach the destination			
			index = shortPath.index(pos)
			nextMove = shortPath[index + 1]
			register = Register(graph.nodes[pos][nextMove])
			nUAVs = register.objectsInTheLink()
			
			distance = getAttribute(AttributesDistance, pos, nextMove)
			capacity = getAttribute(AttributesCapacity, pos, nextMove)  # capacity == critical nUAVs
			vel = getAttribute(AttributesmVelocity, pos, nextMove)
			if nUAVs <= capacity[0]: s_constant = SLOWER[0]  # drones fly with mVelocity
			elif capacity[0] < nUAVs <= capacity[1]: s_constant = SLOWER[1]
			elif capacity[1] < nUAVs <= capacity[2]: s_constant = SLOWER[2]
			else: s_constant = 0

			if s_constant != 0:     # can enter the edge
				now = datetime.now()
				startTimeStamp = datetime.timestamp(now)
				pheromoneTime = addPhero(distance, vel * s_constant)
				finishTimeStamp = startTimeStamp + pheromoneTime
				
				register.phlist.append(finishTimeStamp)

				drone.addEnergy(getEnergy(vel * s_constant, distance))
				drone.addTime(distance / vel * s_constant)
				drone.setPosition(nextMove)
			else:       			# can't enter the edge
				# it should only hover, it can't never change its own way.
				if rule == 1 or rule == 2:
					drone.hover(getEnergy(0, 0), distance / vel)
				else:
					# generate random number between 0 and 10
					# if number is greater or eq to 5 then hover
					# otherwise recalculate best new path 
					dice = np.random.randint(low=0, high=10)
					if dice >=5:
						drone.hover(getEnergy(0, 0), distance / vel)
					else:
						# loop through neighbours
						# and update time and energy attributes to all neighbours
						listNeighbors = list(graph.neighbors(pos))
						oldTime = {}
						oldEnergy = {}
						for n in listNeighbors:
							capacity 	= getAttribute(AttributesCapacity, pos, n)
							vel 		= getAttribute(AttributesmVelocity, pos, n)
							distance 	= getAttribute(AttributesDistance, pos, n)

							oldTime[n]		= getAttribute(AttributesTime, pos, n)
							oldEnergy[n] 	= getAttribute(AttributesEnergy, pos, n)

							register 	= Register(graph.nodes[pos][n])
							nUAVs 		= register.objectsInTheLink()

							if capacity[0] >= nUAVs: s_constant = SLOWER[0]
							elif capacity[0] < nUAVs <= capacity[1]: s_constant = SLOWER[1]
							elif capacity[1] < nUAVs <= capacity[2]: s_constant = SLOWER[2]
							else: s_constant = 0.0000000001 # to avoid exception check 
							
							linkTravelTime = distance / vel * s_constant
							linkTravelEnergy = getEnergy(vel * s_constant, distance)							
							setAttribute(AttributesTime, pos, n, linkTravelTime)
							setAttribute(AttributesEnergy, pos, n, linkTravelEnergy)

						# find a new shortest_path and update drone with it
						shortPath = nx.shortest_path(graph, source=pos, target=destination, weight='time')
						if rule == 4:
							shortPath = nx.shortest_path(graph, source=pos, target=destination, weight='energy')
						drone.setNewPath(shortPath)
						
						# change time and energy back, so prev changes would go unnoticed for other drones
						for n in listNeighbors:
							setAttribute(AttributesTime, pos, n, oldTime[n])
							setAttribute(AttributesEnergy, pos, n, oldEnergy[n])

						fromPos = shortPath[0]
						toPos = shortPath[1]
						register = Register(graph.nodes[fromPos][toPos])
						nUAVs = register.objectsInTheLink()

						# take next step and find slowering constant			
						distance 	= getAttribute(AttributesDistance, fromPos, toPos)
						capacity 	= getAttribute(AttributesCapacity, fromPos, toPos)  		# capacity == critical nUAVs
						vel 		= getAttribute(AttributesmVelocity, fromPos, toPos)
						if 		nUAVs <= capacity[0]: 					s_constant = SLOWER[0] 	# drones fly with mVelocity
						elif 	capacity[0] < nUAVs <= capacity[1]: 	s_constant = SLOWER[1]
						elif 	capacity[1] < nUAVs <= capacity[2]: 	s_constant = SLOWER[2]
						else: 											s_constant = 0

						if s_constant != 0:
							now 			= datetime.now()
							startTimeStamp 	= datetime.timestamp(now)
							pheromoneTime 	= addPhero(distance, vel * s_constant)
							finishTimeStamp	= startTimeStamp + pheromoneTime

							register.phlist.append(finishTimeStamp)

							drone.addEnergy(getEnergy(vel * s_constant, distance))
							drone.addTime(distance / vel * s_constant)
							drone.setPosition(toPos)
						else:
							drone.hover(getEnergy(0, 0), distance / vel)

# Switch layer when the velocity is higher than
# should be changed
def layerSwitches(drone):
	interLayerLinks = mg.get_inter_layer_edges()
	counter = 0
	path = drone.getPath()
	for n in range(len(path) - 1):
		if find(path[n], path[n + 1], interLayerLinks):
			counter += 1
	return counter


def find(nodeOne, nodeTwo, List):
	return (nodeOne, nodeTwo) in List or (nodeTwo, nodeOne) in List


# weights manipulation functions
def getAttribute(listOfweights, source, position):
	if (source, position) in listOfweights:
		attribute = listOfweights[source, position]
	else:
		attribute = listOfweights[position, source]
	return attribute

# weights manipulation functions
def setAttribute(listOfweights, source, position, value):
	if (source, position) in listOfweights:
		listOfweights[source, position] = value
	else:
		listOfweights[position, source] = value

def updateAttribute(listOfweights, source, position, value):
	Tempattribute = getAttribute(listOfweights, source, position)
	update = Tempattribute + value
	if (source, position) in listOfweights:
		listOfweights[source, position] = update
	else:
		listOfweights[position, source] = update
	return update


# Functions for calculating total and average time and energy for all traffic
def totalTime(dronesList):
	# For all drones, sum addTime
	totalTime = 0
	for drone in dronesList:
		totalTime += drone.getTime()
	return totalTime


def totalEnergy(dronesList):
	# For all drones, sum addEnergy
	totalEnergy = 0
	for drone in dronesList:
		totalEnergy += drone.getEnergy()
	return totalEnergy


def averageTotalTime(dronesList):
	averageTotalTime = totalTime(dronesList) / len(dronesList)
	return averageTotalTime


def averageTotalEnergy(dronesList):
	averageTotalEnergy = totalEnergy(dronesList) / len(dronesList)
	return averageTotalEnergy


def addPhero(distance, speed):
	evap = 1.1 * distance / speed
	return evap

# Energy function -> will change
# When the layer is changed? -> basic consuming energy is getting higher?
# W = 10, S = 0.827, N = 6, Cd = 0.96, p = 1.0926
def getEnergy(speed, distance):
	T = math.sqrt(100 + (0.433718 * speed ** 2))
	power = T / ((1873545 * math.pi) * math.sqrt(speed ** 2 + 10 ** 2)) + 0.433718 * speed ** 2 * distance
	return power


# speed Layer1 = (0,15], Layer2 = (15,20], Layer3 = (20,25] roughly
# page8 of Practical Endurance Estimation Minimizing Energy Consumption of Multirotor Unmanned Aerial Vehicles
# node_information: [(startime, endtime), evaporationtime]
# edge_information: capacity, distance, optimalvelocity(max),
def setLayerAttributes(graph):
	# Add weights (maximum capacity, distance, time, energy) for layer network
	for l in range(graph.get_number_of_layers()):
		maxVelocities	= graph.get_intra_layer_edges_of_layer(layer=l)
		capacities 		= graph.get_intra_layer_edges_of_layer(layer=l)
		distances		= graph.get_intra_layer_edges_of_layer(layer=l)
		linkTravelTimes 				= graph.get_intra_layer_edges_of_layer(layer=l)
		linkTravelEnergyConsumptions 	= graph.get_intra_layer_edges_of_layer(layer=l)
		
		numberOfEdges = len(maxVelocities)
		_capacity = list(np.random.randint(low=2, high=5, size=numberOfEdges))
		_distance = list(np.random.randint(low=5, high=30, size=numberOfEdges))

		for i, e in enumerate(maxVelocities):
			maxVelocities[i] 	= (e[0], e[1], MAX_VELOCITY[l])
			capacities[i] 		= (e[0], e[1], [_capacity[i], _capacity[i] * 4/3, _capacity[i] * 5/3])
			distances[i] 		= (e[0], e[1], _distance[i])
			linkTravelTimes[i] 				= (e[0], e[1], _distance[i] / MAX_VELOCITY[l])
			linkTravelEnergyConsumptions[i] = (e[0], e[1], getEnergy(MAX_VELOCITY[l], _distance[i]))
			
			graph.nodes[e[0]][e[1]] = [] 	# evapoTimeEnd
			graph.nodes[e[1]][e[0]] = [] 	# evapoTimeEnd


		graph.add_weighted_edges_from(maxVelocities,				'maxVelocity')
		graph.add_weighted_edges_from(capacities, 					'capacity')
		graph.add_weighted_edges_from(distances, 					'distance')
		graph.add_weighted_edges_from(linkTravelTimes, 				'time')
		graph.add_weighted_edges_from(linkTravelEnergyConsumptions, 'energy')

	# Add weights (maximum capacity, distance, time, energy) for interlayer network
	interLink = graph.get_inter_layer_edges()

	for l in range(graph.get_number_of_layers() - 1):
		interestedLink = []
		interestedLink2 = []
		interestedLink3 = []
		interestedLink4 = []
		interestedLink5 = []

		layerNode1 = list(range(l * N, ((l + 1) * N)))
		layerNode2 = list(range((l + 1) * N, ((l + 2) * N)))

		# initialize node information
		# for e in layerNode1:
		#     graph.nodes[e]['timeStamp'] = [(0,0)]
		#     graph.nodes[e]['evaporationTime'] = [(0,0)]
		#     graph.nodes[e]['UAV'] = [(0,0)]  # reach node, #UAV
		for e in interLink:
			if (e[0] in layerNode1 or e[0] in layerNode2) and (e[1] in layerNode1 or e[1] in layerNode2):
				interestedLink.append(e)
				interestedLink2.append(e)
				interestedLink3.append(e)
				interestedLink4.append(e)
				interestedLink5.append(e)
				graph.nodes[e[0]][e[1]] = []  # evapoTimeEnd
				graph.nodes[e[1]][e[0]] = []  # evapoTimeEnd
				# interestedLink4.append(e)

		for i, e in enumerate(interestedLink):
			interestedLink[i] = (e[0], e[1], MAX_VELOCITY[l])
			interestedLink2[i] = (e[0], e[1], [_capacity[i], _capacity[i]*4/3, _capacity[i]*5/3])
			interestedLink3[i] = (e[0], e[1], _distance[i])
			interestedLink4[i] = (e[0], e[1], _distance[i] / MAX_VELOCITY[l])
			interestedLink5[i] = (e[0], e[1], getEnergy(MAX_VELOCITY[l], _distance[i]))

			graph.nodes[e[0]][e[1]] = []  # evapoTimeEnd
			graph.nodes[e[1]][e[0]] = []  # evapoTimeEnd

		graph.add_weighted_edges_from(interestedLink, 	'maxVelocity')
		graph.add_weighted_edges_from(interestedLink2, 	'capacity')
		graph.add_weighted_edges_from(interestedLink3, 	'distance')
		graph.add_weighted_edges_from(interestedLink4, 	'time')
		graph.add_weighted_edges_from(interestedLink4, 	'energy')

	return graph


####################################################################################
#   Main
####################################################################################
if __name__ == '__main__':

	# Initialization of graph including number of nodes and number of layers (N nodes for each layer)
	# For the test constructions N has to be > 4 and even number
	g1 = mx.generators.erdos_renyi_graph(N, 0.5, seed=217)
	g1 = mx.generators.cycle_graph(g1)
	g2 = mx.generators.erdos_renyi_graph(N, 0.5, seed=211)
	g2 = mx.generators.cycle_graph(g2)
	g3 = mx.generators.erdos_renyi_graph(N, 0.5, seed=208)
	g3 = mx.generators.cycle_graph(g3)
	
	if N > 4 and N%2 == 0:
		g1.add_edge(0,3);
		g2.add_edge(0,3);
		g3.add_edge(0,3);
		k = 4
		j = N-1
		while True:
			g1.add_edge(k,j);
			g2.add_edge(k,j);
			g3.add_edge(k,j);
			k = k+1
			j = j-1
			if j-k < 2:
				break

	adj_block = mx.lil_matrix(np.zeros((N * 3, N * 3)))
	adj_block[0:  N, N:2 * N] = np.identity(N)  # L_12
	adj_block[N:2 * N, 2 * N:3 * N] = np.identity(N)  # L_23
	adj_block += adj_block.T
	adj_block[adj_block > 1] = 1

	mg = mx.MultilayerGraph(list_of_layers=[g1, g2, g3], inter_adjacency_matrix=adj_block)
	mg = setLayerAttributes(mg)

	AttributesmVelocity	= nx.get_edge_attributes(mg, 'maxVelocity')
	AttributesCapacity	= nx.get_edge_attributes(mg, 'capacity')
	AttributesDistance	= nx.get_edge_attributes(mg, 'distance')
	AttributesTime		= nx.get_edge_attributes(mg, 'time')
	AttributesEnergy	= nx.get_edge_attributes(mg, 'energy')


	####################################################################################
	#   Traffic Generation and Testing Motion Rules
	####################################################################################

	drones = generateDrone(mg, NUMBER_OF_DRONS, START_POSITION, DESTINATION_POSITION)

	print('***********************  PATH  ***********************')
	T = nx.shortest_path(mg, source=0, target=8, weight='time')
	print('Dijkstra_Time', T)
	E = nx.shortest_path(mg, source=0, target=8, weight='energy')
	print('Dijkstra_Energy', E)

	print('**********************  MOTION  **********************')
	
	droneMove(mg, drones)
	
	counter = 0
	for drone in drones:
		counter += 1
		if drone.getRule() == 1:
			print('Drone number: ', counter, ',minTime', 'Path', drone.getPath())
			print('Queuing', drone.getCount())
			print('Layer Switches', layerSwitches(drone))
			print('Time for this drone', drone.getTime())
			print('Energy for this drone', drone.getEnergy())
		elif drone.getRule() == 2:
			print('Drone number: ', counter, ',minEnergy', 'Path', drone.getPath())
			print('Queuing', drone.getCount())
			print('Layer Switches', layerSwitches(drone))
			print('Time for this dron', drone.getTime())
			print('Energy for this drone', drone.getEnergy())
		else:
			# print(ct, ce, drone.getTime(), drone.getEnergy())
			print('Drone number: ', counter, ',Optimisation', 'Path', drone.getPath())
			print('Queuing', drone.getCount())
			print('Layer Switches', layerSwitches(drone))
			print('Time for this dron', drone.getTime())
			print('Energy for this drone', drone.getEnergy())

	print('*********************  RESULTS  *********************')

	print('Total Energy', totalEnergy(drones))
	print('Average Energy', averageTotalEnergy(drones))
	print('Total Time', totalTime(drones))
	print('Average Time', averageTotalTime(drones))
	layerCounter = 0
	hoverCount = 0
	for drone in drones:
		layerCounter += layerSwitches(drone)
		hoverCount += drone.getCount()
	print('Total Layer changes', layerCounter)
	print('Total Queuing/Hovering Events', hoverCount)

	####################################################################################
	#   Plotting Graphs
	####################################################################################
	# print(mg.node)
	# print('***********************  PLOT  ***********************')
	# Plotting the weighted graph
	# fig = plt.figure(figsize=(10, 5))
	# ax1 = fig.add_subplot(121)
	# ax1.imshow(mx.adjacency_matrix(mg, weight='distance').todense(),
	# 		   origin='upper', interpolation='nearest', cmap=plt.cm.jet_r)
	# ax1.set_title('supra adjacency matrix')

	# ax2 = fig.add_subplot(122, projection='3d')
	# ax2.axis('on')
	# ax2.set_title('edge colored network')
	# pos = mx.get_position3D(mg)
	# mx.Figure3D(mg, pos=pos, ax=ax2, node_size=50, with_labels=True,
	# 			edge_color=[mg[a][b]['distance'] for a, b in mg.edges()], edge_cmap=plt.cm.jet_r)
	# plt.show()
	# print('***********************  END  ***********************')