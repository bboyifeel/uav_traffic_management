import numpy as np
import networkx as nx
import multinetx as mx
import matplotlib.pyplot as plt
import math
from datetime import datetime
import xlwt
from xlwt import Workbook as wb

# N number of nodes for each layer
N = 80
NUMBER_OF_DRONS = [50, 150, 300]
DESTINATION_POSITION = 182

# MAX_VELOCITY 		= [15, 25, 42]   	km/h 
MAX_VELOCITY = [4.2, 7, 11.7]  # m/s
START_POSITION = 0
# DESTINATION_POSITION = np.random.randint(0, NUMBER_OF_DRONS,1)[0]
SLOWER = [1, 0.8, 0.6]  # slowering constant

RULE_LOW = 1
RULE_HIGH = 2


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
        # print("I'm checking how many is in the link and now is " + str(timestamp))
        toRemove = []
        for i in self.phlist:
            if timestamp > i:
                toRemove.append(i)
            else:
                counter = counter + 1

        for i in toRemove:
            self.phlist.remove(i)

        # print(str(counter) + " UAVs are in the link")
        return counter


####################################################################################
#   Functions
####################################################################################
# Functions for generating traffic, moving and calculating energy, time,
# hovering/queuing count and layer switching
# Generate the list of drones >>> traffic generator
def generateDrone(graph, number, position, destination, rule):
    dronesList = []
    shortPathT = nx.shortest_path(graph, source=position, target=destination, weight='time')
    shortPathE = nx.shortest_path(graph, source=position, target=destination, weight='energy')
    for n in range(number):  # check
        # rule = np.random.randint(low=RULE_LOW, high=RULE_HIGH, size=1)[0]
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
            # print(drone.getPath())
            break

        for drone in dronesList:
            AttributesmVelocity = nx.get_edge_attributes(graph, 'maxVelocity')
            AttributesCapacity = nx.get_edge_attributes(graph, 'capacity')
            AttributesDistance = nx.get_edge_attributes(graph, 'distance')
            AttributesTime = nx.get_edge_attributes(graph, 'time')
            AttributesEnergy = nx.get_edge_attributes(graph, 'energy')

            pos = drone.getPosition()  # present position
            destination = drone.getDestination()
            if (pos == destination):
                continue

            path = drone.getPath()  # Way drone moved
            rule = drone.getRule()  # Drone rule

            shortPath = drone.getShortPath()  # Shortest Path to reach the destination
            index = shortPath.index(pos)
            listNeighbors = list(graph.neighbors(pos))
            nextMove = shortPath[index + 1]
            register = Register(graph.nodes[pos][nextMove])
            nUAVs = register.objectsInTheLink()

            distance = getAttribute(AttributesDistance, pos, nextMove)
            capacity = getAttribute(AttributesCapacity, pos, nextMove)  # capacity == critical nUAVs
            vel = getAttribute(AttributesmVelocity, pos, nextMove)
            if nUAVs <= capacity[0]:
                s_constant = SLOWER[0]  # drones fly with mVelocity
            elif capacity[0] < nUAVs <= capacity[1]:
                s_constant = SLOWER[1]
            elif capacity[1] < nUAVs <= capacity[2]:
                s_constant = SLOWER[2]
            else:
                s_constant = 0

            if s_constant != 0:  # can enter the edge

                now = datetime.now()
                startTimeStamp = datetime.timestamp(now)
                pheromoneTime = addPhero(distance, vel * s_constant)
                finishTimeStamp = startTimeStamp + pheromoneTime

                register.phlist.append(finishTimeStamp)

                drone.addEnergy(getEnergy(vel * s_constant, distance))
                drone.addTime(distance / vel * s_constant)
                drone.setPosition(nextMove)

            else:  # can't enter the edge
                # it should only hover, it can't never change its own way.
                # if rule == 1 or rule == 2:
                #     drone.hover(getEnergy(0, 0), distance / vel)
                # else:
                # generate random number between 0 and 10
                # if number is greater or eq to 5 then hover
                # otherwise recalculate best new path
                dice = np.random.randint(low=0, high=10, size=1)[0]
                if dice >= 5:
                    drone.hover(getEnergy(0, 0), distance / vel)
                else:
                    # loop through neighbours
                    # and update time and energy attributes to all neighbours

                    # oldTime = {}
                    # oldEnergy = {}
                    tempTime = getAttribute(AttributesTime, pos, nextMove)
                    tempEnergy = getAttribute(AttributesEnergy, pos, nextMove)
                    OriginalTime = tempTime
                    OriginalEnrgy = tempEnergy
                    if rule == 3:
                        for n in listNeighbors:
                            if n not in path:
                                capacity = getAttribute(AttributesCapacity, pos, n)
                                vel = getAttribute(AttributesmVelocity, pos, n)
                                distance = getAttribute(AttributesDistance, pos, n)

                                register = Register(graph.nodes[pos][n])
                                nUAVs = register.objectsInTheLink()

                                if capacity[0] >= nUAVs:
                                    s_constant = SLOWER[0]
                                elif capacity[0] < nUAVs <= capacity[1]:
                                    s_constant = SLOWER[1]
                                elif capacity[1] < nUAVs <= capacity[2]:
                                    s_constant = SLOWER[2]
                                else:
                                    s_constant = 0.0000000001  # to avoid exception check

                                linkTravelTime = distance / (vel * s_constant)
                                # print(linkTravelTime, tempTime)

                                if tempTime > linkTravelTime:
                                    nextMove = n
                                    tempTime = linkTravelTime

                        setAttribute(AttributesTime, pos, nextMove, tempTime)

                        shortPath = nx.shortest_path(graph, source=nextMove, target=destination, weight='time')
                        # shortPath = shortPath_1 + shortPath_2[1:]
                    # find a new shortest_path and update drone with it

                    elif rule == 4:
                        for n in listNeighbors:
                            if n not in path:
                                capacity = getAttribute(AttributesCapacity, pos, n)
                                vel = getAttribute(AttributesmVelocity, pos, n)
                                distance = getAttribute(AttributesDistance, pos, n)

                                register = Register(graph.nodes[pos][n])
                                nUAVs = register.objectsInTheLink()

                                if capacity[0] >= nUAVs:
                                    s_constant = SLOWER[0]
                                elif capacity[0] < nUAVs <= capacity[1]:
                                    s_constant = SLOWER[1]
                                elif capacity[1] < nUAVs <= capacity[2]:
                                    s_constant = SLOWER[2]
                                else:
                                    s_constant = 0.0000000001  # to avoid exception check

                                linkTravelEnergy = getEnergy(vel * s_constant, distance)
                                # print(linkTravelEnergy, tempEnergy)

                                if tempEnergy > linkTravelEnergy:
                                    nextMove = n
                                    tempEnergy = linkTravelEnergy

                        setAttribute(AttributesEnergy, pos, nextMove, tempEnergy)
                        # shortPath_1 = nx.shortest_path(graph, source=pos, target=nextMove, weight='energy')
                        shortPath = nx.shortest_path(graph, source=nextMove, target=destination, weight='energy')
                        # shortPath = shortPath_1 + shortPath_2[1:]

                    if s_constant != 0:
                        now = datetime.now()
                        startTimeStamp = datetime.timestamp(now)
                        pheromoneTime = addPhero(distance, vel * s_constant)
                        finishTimeStamp = startTimeStamp + pheromoneTime

                        register.phlist.append(finishTimeStamp)

                        drone.addEnergy(getEnergy(vel * s_constant, distance))
                        drone.addTime(distance / vel * s_constant)

                        drone.setPosition(nextMove)
                        drone.setNewPath(shortPath)
                        # print(shortPath)
                        setAttribute(AttributesTime, pos, nextMove, OriginalTime)
                        setAttribute(AttributesEnergy, pos, nextMove, OriginalEnrgy)

                    else:
                        drone.hover(getEnergy(0, 0), distance / vel)


# Switch layer when the velocity is higher than
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
        maxVelocities = graph.get_intra_layer_edges_of_layer(layer=l)
        capacities = graph.get_intra_layer_edges_of_layer(layer=l)
        distances = graph.get_intra_layer_edges_of_layer(layer=l)
        linkTravelTimes = graph.get_intra_layer_edges_of_layer(layer=l)
        linkTravelEnergyConsumptions = graph.get_intra_layer_edges_of_layer(layer=l)

        numberOfEdges = len(maxVelocities)
        _capacity = list(np.random.randint(low=2, high=N, size=numberOfEdges))
        _distance = list(np.random.randint(low=5, high=30, size=numberOfEdges))

        for i, e in enumerate(maxVelocities):
            maxVelocities[i] = (e[0], e[1], MAX_VELOCITY[l])
            capacities[i] = (e[0], e[1], [_capacity[i], _capacity[i] * 4 / 3, _capacity[i] * 5 / 3])
            distances[i] = (e[0], e[1], _distance[i])
            linkTravelTimes[i] = (e[0], e[1], _distance[i] / MAX_VELOCITY[l])
            linkTravelEnergyConsumptions[i] = (e[0], e[1], getEnergy(MAX_VELOCITY[l], _distance[i]))

            graph.nodes[e[0]][e[1]] = []  # evapoTimeEnd
            graph.nodes[e[1]][e[0]] = []  # evapoTimeEnd

        graph.add_weighted_edges_from(maxVelocities, 'maxVelocity')
        graph.add_weighted_edges_from(capacities, 'capacity')
        graph.add_weighted_edges_from(distances, 'distance')
        graph.add_weighted_edges_from(linkTravelTimes, 'time')
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
            interestedLink2[i] = (e[0], e[1], [_capacity[i], _capacity[i] * 4 / 3, _capacity[i] * 5 / 3])
            interestedLink3[i] = (e[0], e[1], _distance[i])
            interestedLink4[i] = (e[0], e[1], _distance[i] / MAX_VELOCITY[l])
            interestedLink5[i] = (e[0], e[1], getEnergy(MAX_VELOCITY[l], _distance[i]))

            graph.nodes[e[0]][e[1]] = []  # evapoTimeEnd
            graph.nodes[e[1]][e[0]] = []  # evapoTimeEnd

        graph.add_weighted_edges_from(interestedLink, 'maxVelocity')
        graph.add_weighted_edges_from(interestedLink2, 'capacity')
        graph.add_weighted_edges_from(interestedLink3, 'distance')
        graph.add_weighted_edges_from(interestedLink4, 'time')
        graph.add_weighted_edges_from(interestedLink4, 'energy')

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

    if N > 4 and N % 2 == 0:
        g1.add_edge(0, 3);
        g2.add_edge(0, 3);
        g3.add_edge(0, 3);
        k = 4
        j = N - 1
        while True:
            g1.add_edge(k, j);
            g2.add_edge(k, j);
            g3.add_edge(k, j);
            k = k + 1
            j = j - 1
            if j - k < 2:
                break

    adj_block = mx.lil_matrix(np.zeros((N * 3, N * 3)))
    adj_block[0:  N, N:2 * N] = np.identity(N)  # L_12
    adj_block[N:2 * N, 2 * N:3 * N] = np.identity(N)  # L_23
    adj_block += adj_block.T
    adj_block[adj_block > 1] = 1

    mg = mx.MultilayerGraph(list_of_layers=[g1, g2, g3], inter_adjacency_matrix=adj_block)
    mg = setLayerAttributes(mg)

    AttributesmVelocity = nx.get_edge_attributes(mg, 'maxVelocity')
    AttributesCapacity = nx.get_edge_attributes(mg, 'capacity')
    AttributesDistance = nx.get_edge_attributes(mg, 'distance')
    AttributesTime = nx.get_edge_attributes(mg, 'time')
    AttributesEnergy = nx.get_edge_attributes(mg, 'energy')

    ####################################################################################
    #   Traffic Generation and Testing Motion Rules
    ####################################################################################

    row = 0
    saveFile = wb()

    turn = 0
    sheet1 = saveFile.add_sheet('10')

    sheet1.write(row, 0, 'Testrun')
    sheet1.write(row, 1, 'Rule')
    sheet1.write(row, 2, 'Drone Number')
    sheet1.write(row, 3, 'Path')
    sheet1.write(row, 4, 'Queuing')
    sheet1.write(row, 5, 'Layer Switches')
    sheet1.write(row, 6, 'Time')
    sheet1.write(row, 7, 'Energy')
    sheet1.write(row, 8, 'Traffic')

    print('***********************  PATH  ***********************')
    T = nx.shortest_path(mg, source=START_POSITION, target=DESTINATION_POSITION, weight='time')
    print('Dijkstra_Time', T)
    E = nx.shortest_path(mg, source=START_POSITION, target=DESTINATION_POSITION, weight='energy')
    print('Dijkstra_Energy', E)
    print('***********************  PATH  ***********************')
    testrun = 1
    while(testrun < 31):
        print(testrun)
        turn = 0
        while (turn < 3):
            drones3 = generateDrone(mg, NUMBER_OF_DRONS[turn], START_POSITION, DESTINATION_POSITION, 3)
            drones4 = generateDrone(mg, NUMBER_OF_DRONS[turn], START_POSITION, DESTINATION_POSITION, 4)
            # print(NUMBER_OF_DRONS[turn])

            # print('**********************  MOTION  **********************')

            droneMove(mg, drones3)
            droneMove(mg, drones4)

            counter = 0
            for drone in drones3:
                # print(counter)
                counter += 1
                row += 1
                sheet1.write(row, 0, testrun)
                sheet1.write(row, 1, drone.getRule())
                sheet1.write(row, 2, counter)
                sheet1.write(row, 3, ",".join(str(e) for e in drone.getPath()))
                sheet1.write(row, 4, drone.getCount())
                sheet1.write(row, 5, layerSwitches(drone))
                sheet1.write(row, 6, drone.getTime())
                sheet1.write(row, 7, drone.getEnergy())
                sheet1.write(row, 8, NUMBER_OF_DRONS[turn])

            # print('*********************  RESULTS3  *********************')
            #
            # print('Total Energy', totalEnergy(drones3))
            # print('Average Energy', averageTotalEnergy(drones3))
            # print('Total Time', totalTime(drones3))
            # print('Average Time', averageTotalTime(drones3))
            # layerCounter = 0
            # hoverCount = 0
            # for drone in drones3:
            #     layerCounter += layerSwitches(drone)
            #     hoverCount += drone.getCount()
            # print('Total Layer changes', layerCounter)
            # print('Total Queuing/Hovering Events', hoverCount)

            for drone in drones4:
                # print(counter)
                counter += 1
                row += 1
                sheet1.write(row, 0, testrun)
                sheet1.write(row, 1, drone.getRule())
                sheet1.write(row, 2, counter)
                sheet1.write(row, 3, ",".join(str(e) for e in drone.getPath()))
                sheet1.write(row, 4, drone.getCount())
                sheet1.write(row, 5, layerSwitches(drone))
                sheet1.write(row, 6, drone.getTime())
                sheet1.write(row, 7, drone.getEnergy())
                sheet1.write(row, 8, NUMBER_OF_DRONS[turn])

            # print('*********************  RESULTS4  *********************')
            #
            # print('Total Energy', totalEnergy(drones4))
            # print('Average Energy', averageTotalEnergy(drones4))
            # print('Total Time', totalTime(drones4))
            # print('Average Time', averageTotalTime(drones4))
            # layerCounter = 0
            # hoverCount = 0
            # for drone in drones4:
            #     layerCounter += layerSwitches(drone)
            #     hoverCount += drone.getCount()
            # print('Total Layer changes', layerCounter)
            # print('Total Queuing/Hovering Events', hoverCount)

            # layerCounter = 0
            # hoverCount = 0
            # for drone in drones4:
            #     layerCounter += layerSwitches(drone)
            #     hoverCount += drone.getCount()
            # print('Total Layer changes', layerCounter)
            # print('Total Queuing/Hovering Events', hoverCount)
            turn += 1
            # fig = plt.figure(figsize=(10, 5))
            # ax1 = fig.add_subplot(121)
            # ax1.imshow(mx.adjacency_matrix(mg, weight='distance').todense(),
            #            origin='upper', interpolation='nearest', cmap=plt.cm.jet_r)
            # ax1.set_title('supra adjacency matrix')
            #
            # ax2 = fig.add_subplot(122, projection='3d')
            # ax2.axis('on')
            # ax2.set_title('edge colored network')
            # pos = mx.get_position3D(mg)
            # mx.Figure3D(mg, pos=pos, ax=ax2, node_size=50, with_labels=True,
            #             edge_color=[mg[a][b]['distance'] for a, b in mg.edges()], edge_cmap=plt.cm.jet_r)
            # plt.show()
        testrun += 1
    saveFile.save('30_test.xls')
    print('30_test')
    ####################################################################################
    #   Plotting Graphs
    ####################################################################################

    print(mg.node)
    print('***********************  PLOT  ***********************')
    # Plotting the weighted graph
    #     fig = plt.figure(figsize=(10, 5))
    #     ax1 = fig.add_subplot(121)
    #     ax1.imshow(mx.adjacency_matrix(mg, weight='distance').todense(),
    #             origin='upper', interpolation='nearest', cmap=plt.cm.jet_r)
    #     ax1.set_title('supra adjacency matrix')
    #
    #     ax2 = fig.add_subplot(122, projection='3d')
    #     ax2.axis('on')
    #     ax2.set_title('edge colored network')
    #     pos = mx.get_position3D(mg)
    #     mx.Figure3D(mg, pos=pos, ax=ax2, node_size=50, with_labels=True,
    #             edge_color=[mg[a][b]['distance'] for a, b in mg.edges()], edge_cmap=plt.cm.jet_r)
    #     plt.show()
    print('***********************  END  ***********************')
