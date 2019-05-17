# timestamp# find constant min(ui) = to find (hovering) energy# node_information: [(startime, endtime), evaporationtime]# edge_information: capacity, distance, optimalvelocity(max),# how much the speed is reduced,# speed Layer1 = (0,15], Layer2 = (15,20], Layer3 = (20,25], page8 of Practical Endurance Estimation Minimizing Energy Consumption of Multirotor Unmanned Aerial Vehicles# AttributeTime = distance/Meanvelocityoflayer or hovering time?import numpy as npimport networkx as nximport multinetx as mximport matplotlib.pyplot as pltimport mathclass UAV(object):    def __init__(self, sourcePosition, destination, shortPath, rule):        self.sourcePosition = sourcePosition        self.path = [sourcePosition]        self.shortPath = [shortPath]        self.energy = 0        self.time = 0        self.cT = 0        self.cE = 0        self.qCount = 0  # queueing/hovering count        self.rule = rule        self.destination = destination        self.shortPath = shortPath        self.ui = 20    def setNewPath(self, newShortPath):        self.shortPath = newShortPath        return self.shortPath    def getShortPath(self):        return self.shortPath    def setRule(self, newRule):        self.rule = newRule        return self.rule    def getRule(self):        return self.rule    def getPosition(self):        return self.path[-1]    def getDestination(self):        return self.destination    def setPosition(self, node):        self.path.append(node)    def getPath(self):        return self.path    def addEnergy(self, energy):        self.energy += energy        return energy    def addTime(self, time):        self.time += time        return time    def getTime(self):        return self.time    def getEnergy(self):        return self.energy    def hover(self):        self.addEnergy(self.ui) #it should be the power (when u = 0)        self.addTime(1) # temp        self.qCount += 1        return self.qCount    def getCount(self):        return self.qCount#####################################################################################   Functions##################################################################################### Functions for generating traffic, moving and calculating energy, time,# hovering/queuing count and layer switching# Generate the list of drones >>> traffic generatordef generateDrone(graph, number, position, destination):    dronesList = []    shortPathT = nx.shortest_path(graph, source=position, target=destination, weight='time')    shortPathE = nx.shortest_path(graph, source=position, target=destination, weight='energy')    for n in range(number):  # check        rule = np.random.randint(low=1, high=6, size=1)[0]        if rule == 1:            d = UAV(position, destination, shortPathT, 1)        elif rule == 2:            d = UAV(position, destination, shortPathE, 2)        else:            d = UAV(position, destination, shortPathE, rule)        dronesList.append(d)    return dronesList# Function that defines the movement of the drones given list of instances of the UAV class# all drones move at the same time (with this code)def droneMove(graph, dronesList):    # Loop through the list of drones and for every drone reached destination    while 1:        flag = 1        for drone in dronesList:            if drone.getPosition() != drone.getDestination():                flag = 0        if flag == 1:            break        counter = 0        AttributesTime = nx.get_edge_attributes(graph, 'time')        AttributesCapacity = nx.get_edge_attributes(graph, 'capacity')        AttributesnUAV = nx.get_edge_attributes(graph, 'nUAV')        AttributesDistance = nx.get_edge_attributes(graph, 'distance')        for drone in dronesList:            counter += 1            pos = drone.getPosition()            path = drone.getPath()            rule = drone.getRule()            destination = drone.getDestination()            if rule == 1 or rule == 2:                shortPath = drone.getShortPath()                index = shortPath.index(pos)            # print('index', index , pos)            if (pos == destination):                continue            # it should only hover, it can't never change its own way.            if rule == 1 or rule == 2:                u = 10                move = shortPath[index + 1]                distance = getAttribute(AttributesDistance,pos,move)                # print('index', index , pos , move)                capacity = getAttribute(AttributesCapacity, pos, move)                nUAV = getAttribute(AttributesnUAV, pos, move)                if (capacity > 0) or (nUAV > (capacity / 4)):                    ######################################################                    #   u should be changed with the slowering constant  #                    ######################################################                    if (nUAV <= (capacity / 2)) :                        u = 10  # max velocity(temp)                    # getEnergy(dronespeed, rotorspeed)                    elif (nUAV > (capacity / 2)) and  (nUAV <= (capacity / 4)):                        u = 8                    drone.addEnergy(getEnergy(10, pos, move, distance))                    drone.addTime(distance/u)                    drone.setPosition(move)                    # Update Capacities of edges of graph                    updateAttribute(AttributesCapacity, pos, move, -1)                    if len(drone.getPath()) > 2:                        tempPath = drone.getPath()                        tempIndex = len(drone.getPath())                        updateAttribute(AttributesCapacity, tempPath[tempIndex - 3], tempPath[tempIndex - 2], +1)                else:                    drone.hover()                    ####################################################################################                    #   Option 4 SWITCH TO ONE OF THE GREEDY ALGORITHMS                    ####################################################################################                    prob = np.random.randint(low=1, high=10, size=1)                    if prob > 5:                        if rule == 1:                            drone.setRule(4)                        elif rule == 2:                            drone.setRule(3)            #######################################################################################            #   Rule 3 is to switch to greedy algorithm ENERGY            #######################################################################################            if drone.getRule() == 3:                # print('Not defined')                Previous = 9999999999                nextBestMove = -1                listNeighbors = list(graph.neighbors(pos))                for n in listNeighbors:                    distance = getAttribute(AttributesDistance, pos, n)                    if (n not in drone.getPath() and getEnergy(10, pos, n, distance) < Previous and getAttribute(AttributesCapacity, pos, n) > 0):                        nextBestMove = n                        Previous = getAttribute(AttributesTime, pos, nextBestMove)                if nextBestMove == -1:                    newShortPath = nx.shortest_path(graph, source=pos, target=drone.getDestination(), weight='time')                    drone.setNewPath(newShortPath)                    drone.setRule(2)                    drone.hover()                    break                else:                    drone.addEnergy(getEnergy(10, pos, n, distance))                    drone.addTime(getAttribute(AttributesTime, pos, nextBestMove))                    drone.setPosition(nextBestMove)                    # Update Capacities of edges of graph                    updateAttribute(AttributesCapacity, pos, nextBestMove, -1)                    if len(drone.getPath()) > 2:                        tempPath = drone.getPath()                        tempIndex = len(drone.getPath())                        updateAttribute(AttributesCapacity, tempPath[tempIndex - 3], tempPath[tempIndex - 2], +1)            #######################################################################################            #   Rule 4 is to switch to greedy algorithm TIME            #######################################################################################            if drone.getRule() == 4:                # print('Not defined')                Previous = 9999999999                nextBestMove = -1                listNeighbors = list(graph.neighbors(pos))                # print("Present Pos: ", drone.getPath())                # print("Neighbors: ",listNeighbors)                for n in listNeighbors:                    distance = getAttribute(AttributesDistance, pos, n)                    if (n not in drone.getPath() and getAttribute(AttributesTime, pos, n) < Previous and getAttribute(AttributesCapacity, pos, n) > 0):                        nextBestMove = n                        Previous = getAttribute(AttributesTime, pos, nextBestMove)                if nextBestMove == -1:                    newShortPath = nx.shortest_path(graph, source=pos, target=drone.getDestination(), weight='time')                    drone.setNewPath(newShortPath)                    drone.setRule(1)                    drone.hover()                    break                else:                    drone.addEnergy(getEnergy(10, pos, n, distance))                    drone.addTime(getAttribute(AttributesTime, pos, nextBestMove))                    drone.setPosition(nextBestMove)                    # Update Capacities of edges of graph                    updateAttribute(AttributesCapacity, pos, nextBestMove, -1)                    if len(drone.getPath()) > 2:                        tempPath = drone.getPath()                        tempIndex = len(drone.getPath())                        updateAttribute(AttributesCapacity, tempPath[tempIndex - 3], tempPath[tempIndex - 2], +1)            #######################################################################################            #   Rule 5 is to switch to            #######################################################################################            if drone.getRule() == 5:                Previous = 9999999999                nextBestMove = -1                listNeighbors = list(graph.neighbors(pos))                for n in listNeighbors:                    distance = getAttribute(AttributesDistance, pos, n)                    if (n not in drone.getPath() and (getEnergy(10, pos, n, distance))< Previous and getAttribute(AttributesCapacity, pos, n) > 0):                        nextBestMove = n                        Previous = (getEnergy(10, pos, n, distance))                if nextBestMove == -1:                    newShortPath = nx.shortest_path(graph, source=pos, target=drone.getDestination(), weight='time')                    drone.setNewPath(newShortPath)                    drone.setRule(4)                    drone.hover()                    break                else:                    drone.addEnergy(getEnergy(10, pos, n, distance))                    drone.addTime(getAttribute(AttributesTime, pos, nextBestMove))                    drone.setPosition(nextBestMove)                    # Update Capacities of edges of graph                    updateAttribute(AttributesCapacity, pos, nextBestMove, -1)                    if len(drone.getPath()) > 2:                        tempPath = drone.getPath()                        tempIndex = len(drone.getPath())                        updateAttribute(AttributesCapacity, tempPath[tempIndex - 3], tempPath[tempIndex - 2], +1)# Switch layer when the velocity is higher than# should be changeddef layerSwitches(drone):    interLayerLinks = mg.get_inter_layer_edges()    counter = 0    path = drone.getPath()    for n in range(len(path) - 1):        if find(path[n], path[n + 1], interLayerLinks):            counter += 1    return counterdef find(nodeOne, nodeTwo, List):    return (nodeOne, nodeTwo) in List or (nodeTwo, nodeOne) in List# weights manipulation functionsdef getAttribute(listOfweights, source, position):    if (source, position) in listOfweights:        attribute = listOfweights[source, position]    else:        attribute = listOfweights[position, source]    return attributedef updateAttribute(listOfweights, source, position, value):    Tempattribute = getAttribute(listOfweights, source, position)    update = Tempattribute + value    if (source, position) in listOfweights:        listOfweights[source, position] = update    else:        listOfweights[position, source] = update    return update# Functions for calculating total and average time and energy for all trafficdef totalTime(dronesList):    # For all drones, sum addTime    totalTime = 0    for drone in dronesList:        totalTime += drone.getTime()    return totalTimedef totalEnergy(dronesList):    # For all drones, sum addEnergy    totalEnergy = 0    for drone in dronesList:        totalEnergy += drone.getEnergy()    return totalEnergydef averageTotalTime(dronesList):    averageTotalTime = totalTime(dronesList) / len(dronesList)    return averageTotalTimedef averageTotalEnergy(dronesList):    averageTotalEnergy = totalEnergy(dronesList) / len(dronesList)    return averageTotalEnergy# find best 0 <= cT, cE <= 1# def updateEdgeWeight():#     onlytime: cT=1, onlyEnerge: cE=1, ouralgorithm: try to find best cT & cE to make min(edgeweight)#     for drone in dronesList:def setLayerAtt(graph):    # time = list(np.random.randint(low=1, high=5, size=3))    # energy = list(np.random.randint(low=1, high=5, size=3))    distance = list(np.random.randint(low=2, high=30, size=3))    capacity = list(np.random.randint(low=10, high=50, size=3))    mVelocity = [8, 17, 23]    # Add weights (time energy and maximum capacity)for intralayer network    for l in range(graph.get_number_of_layers()):        link = graph.get_intra_layer_edges_of_layer(layer=l)        link2 = graph.get_intra_layer_edges_of_layer(layer=l)        link3 = graph.get_intra_layer_edges_of_layer(layer=l)        link4 = graph.get_intra_layer_edges_of_layer(layer=l)        # time = list(np.random.randint(low=10 - (2 * l), high=30 - (2 * l), size=len(link)))        # energy = list(np.random.randint(low=1 + (10 * l), high=4 + (10 * l), size=len(link)))        capacity = list(np.random.randint(low=2, high=4, size=len(link)))        distance = list(np.random.randint(low=2, high=30, size=len(link)))        # time = list(1/(1+(10*l)))        # energy = list(1+(10*l))        # capacity = list(1+(2*l)        for i, e in enumerate(link):            link[i] = (e[0], e[1], distance[i]/mVelocity[l])            link2[i] = (e[0], e[1], 0)            link3[i] = (e[0], e[1], capacity[i])            link4[i] = (e[0], e[1], distance[i])        graph.add_weighted_edges_from(link, 'time')        graph.add_weighted_edges_from(link2, 'nUAV')        graph.add_weighted_edges_from(link3, 'capacity')        graph.add_weighted_edges_from(link4, 'distance')        # mg.set_edges_weights(inter_layer_edges_weight=10)        # Add weights (time energy and maximum capacity)for interlayer network    interLink = graph.get_inter_layer_edges()    for l in range(graph.get_number_of_layers() - 1):        interestedLink = []        interestedLink2 = []        interestedLink3 = []        interestedLink4 = []        layerNode1 = list(range(l * N, ((l + 1) * N)))        layerNode2 = list(range((l + 1) * N, ((l + 2) * N)))        for e in interLink:            if (e[0] in layerNode1 or e[0] in layerNode2) and (e[1] in layerNode1 or e[1] in layerNode2):                interestedLink.append(e)                interestedLink2.append(e)                interestedLink3.append(e)                interestedLink4.append(e)        for i, e in enumerate(interestedLink):            interestedLink[i] = (e[0], e[1], distance[l]/mVelocity[l])            interestedLink2[i] = (e[0], e[1], 0)            interestedLink3[i] = (e[0], e[1], capacity[l])            interestedLink4[i] = (e[0], e[1], distance[l])            # graph.nodes[i]['#UAVs'] = i        graph.add_weighted_edges_from(interestedLink, 'time')        graph.add_weighted_edges_from(interestedLink2, 'nUAV')        graph.add_weighted_edges_from(interestedLink3, 'capacity')        graph.add_weighted_edges_from(interestedLink4, 'distance')    # print('**********************  GRAPH  **********************')    # for e in mg.edges():    # 	print (e, mg.edges[e[0],e[1]])    return graph## Energy function -> will changedef getEnergy(speed, pos, next, distance):    # W = 10, S = 0.827, N = 6, Cd = 0.96, p = 1.0926    # if (pos, next) in listofdis:    #     dis = listofdis[pos, next]    T = math.sqrt(100+(0.433718*speed**2))    power = T / ((1873545* math.pi) * math.sqrt(speed**2 + 10**2))+ 0.433718*speed**2*distance    return power#####################################################################################   Main####################################################################################if __name__ == '__main__':    # lists of weights for the links/edges between layers    # time = [5*n for n in range(1,5)]    # energy = [5*n for n in range(1,5)]    # capacity = [5*n for n in range(1,5)]    # #Initialization of graph including number of nodes and number of layers(N nodes for each layer)    N = 4    g1 = mx.generators.erdos_renyi_graph(N, 0.5, seed=218)    g2 = mx.generators.erdos_renyi_graph(N, 0.5, seed=211)    g3 = mx.generators.erdos_renyi_graph(N, 0.5, seed=208)    adj_block = mx.lil_matrix(np.zeros((N * 3, N * 3)))    # adj_block[0:  N,  N:2*N] = np.random.poisson(1, size = (N,N))    # L_12    # adj_block[N:2*N,2*N:3*N] = np.random.poisson(1, size = (N,N))    # L_23    adj_block[0:  N, N:2 * N] = np.identity(N)  # L_12    # adj_block[0: N, 2*N : 3*N] = np.identity(N)    adj_block[N:2 * N, 2 * N:3 * N] = np.identity(N)  # L_23    # # use symmetric inter-adjacency matrix    adj_block += adj_block.T    adj_block[adj_block > 1] = 1    mg = mx.MultilayerGraph(list_of_layers=[g1, g2, g3], inter_adjacency_matrix=adj_block)    mg = setLayerAtt(mg)    AttributesTime = nx.get_edge_attributes(mg, 'time')    AttributesUAV = nx.get_edge_attributes(mg, 'nUAV')    AttributesCapacity = nx.get_edge_attributes(mg, 'capacity')    AttributesDistance = nx.get_edge_attributes(mg, 'distance')    print (mg)    ####################################################################################    #   Traffic Generation and Testing Motion Rules    ####################################################################################    for i in range(1):        drones = generateDrone(mg, 10, 0, 7)        print('***********************  PATH  ***********************')        T = nx.shortest_path(mg, source=0, target=7, weight='time')        print('Dijkstra_Time', T)        E = nx.shortest_path(mg, source=0, target=7, weight='energy')        print('Dijkstra_Energy', E)        print('**********************  MOTION  **********************')        droneMove(mg, drones)        counter = 0        for drone in drones:            counter += 1            if drone.getRule() == 1:                print('Drone number: ', counter, ',minTime', 'Path', drone.getPath())                print('Queuing', drone.getCount())                print('Layer Switches', layerSwitches(drone))                print('Time for this drone', drone.getTime())                print('Energy for this drone', drone.getEnergy())            elif drone.getRule() == 2:                print('Drone number: ', counter, ',minEnergy', 'Path', drone.getPath())                print('Queuing', drone.getCount())                print('Layer Switches', layerSwitches(drone))                print('Time for this dron', drone.getTime())                print('Energy for this drone', drone.getEnergy())            elif drone.getRule() == 3:                print('Drone number: ', counter, ',GreedyMinEnergy', 'Path', drone.getPath())                print('Queuing', drone.getCount())                print('Layer Switches', layerSwitches(drone))                print('Time for this dron', drone.getTime())                print('Energy for this drone', drone.getEnergy())            elif drone.getRule() == 4:                print('Drone number: ', counter, ',GreedyMinTime', 'Path', drone.getPath())                print('Queuing', drone.getCount())                print('Layer Switches', layerSwitches(drone))                print('Time for this dron', drone.getTime())                print('Energy for this drone', drone.getEnergy())            elif drone.getRule() == 5:                # print(ct, ce, drone.getTime(), drone.getEnergy())                print('Drone number: ', counter, ',Optimisation', 'Path', drone.getPath())                print('Queuing', drone.getCount())                print('Layer Switches', layerSwitches(drone))                # print('Time for this dron', ct * drone.getTime())                # print('Energy for this drone', ce * drone.getEnergy())        print('*********************  RESULTS  *********************')        print('Total Energy', totalEnergy(drones))        print('Average Energy', averageTotalEnergy(drones))        print('Total Time', totalTime(drones))        print('Average Time', averageTotalTime(drones))        layerCounter = 0        hoverCount = 0        for drone in drones:            layerCounter += layerSwitches(drone)            hoverCount += drone.getCount()        print('Total Layer changes', layerCounter)        print('Total Queuing/Hovering Events', hoverCount)    ####################################################################################    #   Plotting Graphs    ####################################################################################    print(mg.node)    print('***********************  PLOT  ***********************')    # Plotting the weighted graph    fig = plt.figure(figsize=(15, 5))    ax1 = fig.add_subplot(121)    ax1.imshow(mx.adjacency_matrix(mg, weight='time').todense(),               origin='upper', interpolation='nearest', cmap=plt.cm.jet_r)    ax1.set_title('supra adjacency matrix')    ax2 = fig.add_subplot(122, projection='3d')    ax2.axis('on')    ax2.set_title('edge colored network')    pos = mx.get_position3D(mg)    # pos = mx.get_position(mg,mx.fruchterman_reingold_layout(g1),    #				  layer_vertical_shift=0.2,    #				  layer_horizontal_shift=0.0,    #				  proj_angle=47)    # mx.draw_networkx(mg,pos=pos,ax=ax2,node_size=50,with_labels=True,    # 			 edge_color=[mg[a][b]['time'] for a,b in mg.edges()],    # 			 edge_cmap=plt.cm.jet_r)    mx.Figure3D(mg, pos=pos, ax=ax2, node_size=50, with_labels=True,                edge_color=[mg[a][b]['time'] for a, b in mg.edges()], edge_cmap=plt.cm.jet_r)    plt.show()    print('***********************  END  ***********************')