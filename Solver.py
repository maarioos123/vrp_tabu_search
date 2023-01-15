from VRP_Model import *
import random
from SolutionDrawer import *
import copy


class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []
        self.sequenceOfNodes = []


class RelocationMove(object):
    def __init__(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = None

    def Initialize(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = 10 ** 9


class SwapMove(object):

    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = None

    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = 10 ** 9


class TwoOptMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = None

    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = 10 ** 9


class CustomerInsertion(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.cost = 10 ** 9


class CustomerInsertionAllPositions(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.insertionPosition = None
        self.cost = 10 ** 9


def createOutputTxt(solver):
    with open('solution.txt', 'w') as f:
        f.write('Cost:\n')
        f.write(str(solver.sol.cost))
        f.write('\n')
        f.write('Routes:')
        f.write('\n')
        f.write(str(len(solver.sol.routes)))
        f.write('\n')
        for i in solver.sol.routes:
            route = [n.ID for n in i.sequenceOfNodes]
            f.write(str(route)[1:-1])
            f.write('\n')


class Solver:
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.matrix
        self.capacity = m.capacity
        self.used = {self.depot.ID}
        self.sol = None
        self.bestSolution = None
        self.minTabuTenure = 20
        self.maxTabuTenure = 30
        self.tabuTenure = 30

    def solve(self):
        self.ApplyNearestNeighborMethod()
        self.ReportSolution(self.sol)
        self.TabuSearch(0)
        # createOutputTxt(self)
        # self.ReportSolution(self.sol)
        return self.sol

    def dot(self, vA, vB):
        return vA[0] * vB[0] + vA[1] * vB[1]

    def ang(self, lineA, lineB):
        # Get nicer vector form
        vA = [(lineA[0][0] - lineA[1][0]), (lineA[0][1] - lineA[1][1])]
        vB = [(lineB[0][0] - lineB[1][0]), (lineB[0][1] - lineB[1][1])]
        # Get dot prod
        dot_prod = self.dot(vA, vB)
        # Get magnitudes
        magA = self.dot(vA, vA) ** 0.5
        magB = self.dot(vB, vB) ** 0.5
        # Get cosine value
        cos_ = dot_prod / magA / magB
        # Get angle in radians and then convert to degrees
        try:
            angle = math.acos(dot_prod / magB / magA)
            ang_deg = math.degrees(angle) % 360

            if ang_deg - 180 >= 0:
                # As in if statement
                return 360 - ang_deg
            else:

                return ang_deg
        except:
            return -100
            pass
        # Basically doing angle <- angle mod 360

    def find_node(self, r, n):

        dist = self.distanceMatrix[n.ID]
        nearest_v = 10000000
        nearest_index = 0
        for i in range(1, len(dist)):
            # min_dist = 1000000
            # for n1 in self.list_of_last_nodes:
            #     if n1.ID != 0 and self.distanceMatrix[i][n1.ID] < min_dist:
            #         min_dist = self.distanceMatrix[i][n1.ID]

            if dist[i] < nearest_v and i not in self.used and r.load + self.allNodes[i].demand < r.capacity:
                nearest_index = i
                nearest_v = dist[i]

        self.used.add(nearest_index)

        return nearest_index, nearest_v

    def find_route(self, node, multiplier, multiplier2):

        dist = self.distanceMatrix[node.ID]
        nearest_v = 100000000
        for l_node in self.list_of_last_nodes:

            waiting_time = dist[l_node.ID] + l_node.waitingtime
            waiting_time = multiplier * dist[l_node.ID] + l_node.waitingtime
            if l_node.ID == 0:
                if waiting_time < nearest_v:
                    nearest_node = l_node
                    nearest_v = waiting_time
            else:
                lineA = ((l_node.x, l_node.y), (node.x, node.y))
                previous_node = l_node.route.sequenceOfNodes[-2]
                lineB = ((l_node.x, l_node.y), (previous_node.x, previous_node.y))

                angle = self.ang(lineA, lineB)
                if angle != -100:
                    val = 180 - angle
                    objective = waiting_time

                    if val < 30:
                        objective = multiplier2 * waiting_time
                    elif val < 40:
                        objective = (multiplier2 + 0.05) * waiting_time
                    elif val > 110:
                        objective = 1.2 * waiting_time

                    if objective < nearest_v and l_node.route.load + node.demand < l_node.route.capacity:
                        nearest_node = l_node
                        self.hhh = dist[l_node.ID]
                        nearest_v = waiting_time
                else:
                    if waiting_time < nearest_v and l_node.route.load + node.demand < l_node.route.capacity:
                        nearest_node = l_node
                        nearest_v = waiting_time

        nearest_v -= (multiplier - 1) * dist[nearest_node.ID]

        return nearest_node, nearest_v

    def find_less_distant_node(self, multiplier, multiplier2):
        dist_of_most_distant = 1000000
        for node in self.customers:
            if node.isRouted == False:
                nearest_node, nearest_v = self.find_route(node, multiplier, multiplier2)
                if nearest_v < dist_of_most_distant:
                    dist_of_most_distant = nearest_v
                    most_distant = node
        return most_distant

    # def find_most_distant_node(self,multiplier):
    #     dist_of_most_distant = -1000000
    #     for node in self.customers:
    #         if node.isRouted == False:
    #             nearest_node, nearest_v = self.find_route(node,multiplier)
    #             if nearest_v > dist_of_most_distant:
    #                 dist_of_most_distant = nearest_v
    #                 most_distant = node
    #     return most_distant
    def find_nearest_node_to_depot(self):
        dist = self.distanceMatrix[0]
        min = 100000000
        for c in self.customers:
            if dist[c.ID] < min:
                min_node = c
                min = dist[c.ID]

        return min_node, min

    def ApplyNearestNeighborMethod(self):
        modelIsFeasible = True
        self.sol = Solution()

        for i in range(0, 14):
            self.sol.routes.append(Route(self.depot, self.capacity))
            self.sol.routes[i].ID = i

        # j = 0
        self.list_of_last_nodes = [r.last_node for r in self.sol.routes]

        number_of_unrouted_nodes = 100
        for i in range(number_of_unrouted_nodes):
            if i == 0:
                node, value = self.find_nearest_node_to_depot()
                route = self.sol.routes[0]
            else:
                multiplier = 1.3

                multiplier2 = 0.8
                node = self.find_less_distant_node(multiplier, multiplier2)

                nearest_last_node, value = self.find_route(node, multiplier, multiplier2)
                if nearest_last_node.ID == 0:
                    for r in self.sol.routes:
                        if len(r.sequenceOfNodes) == 1:
                            route = r
                            break
                else:
                    route = nearest_last_node.route

            self.used.add(node.ID)
            node.isRouted = True
            if len(route.sequenceOfNodes) != 1:
                node.waitingtime += 10
            node.waitingtime += value

            # route = nearest_last_node.route
            route.cost += node.waitingtime
            self.sol.cost += node.waitingtime
            route.load += node.demand
            node.cost_up_to_here = route.cost
            node.positionInRoute = len(route.sequenceOfNodes)
            route.sequenceOfNodes.append(node)
            node.route = route
            route.last_node = node
            self.list_of_last_nodes[route.ID] = node

        if (modelIsFeasible == False):
            print('FeasibilityIssue')
            # reportSolution

    def geteasysol(self):
        routes = [[52, 88, 7, 82, 48, 8, 46], [89, 18, 60, 83, 5, 17, 47, 36], [12, 68, 80, 29, 24, 65],
                  [28, 77, 3, 79, 78, 34, 35], [58, 97, 87, 42, 15, 43], [6, 96, 99, 85, 91, 16, 86, 14],
                  [76, 50, 33, 9, 51, 66, 71], [95, 59, 92, 98, 61, 84, 45, 19, 64], [53, 2, 57, 41, 22, 67],
                  [69, 70, 30, 20, 32, 90, 63, 49, 11], [40, 21, 73, 72, 74, 75, 23],
                  [13, 94, 93, 37, 100, 44, 38], [27, 31, 62, 10, 1, 81], [26, 54, 55, 25, 4, 56, 39]]
        rts = []
        self.sol = Solution()
        for i in routes:
            self.sol.routes.append(Route(self.depot, self.capacity))
            rt = self.sol.routes[-1]
            for j in i:
                rt.sequenceOfNodes.append(self.allNodes[j])
            rts.append(rt)
        for j in self.sol.routes:
            j: Route
            rt_load = 0
            rt_cumulative_cost = 0
            tot_time = 0
            nodes_sequence = j.sequenceOfNodes
            for i in range(len(nodes_sequence) - 1):
                from_node = nodes_sequence[i]
                to_node = nodes_sequence[i + 1]
                tot_time += self.distanceMatrix[from_node.ID][to_node.ID]
                rt_cumulative_cost += tot_time
                tot_time += 10
                rt_load += from_node.demand
            j.load = rt_load
            j.cost = rt_cumulative_cost
        for j in self.sol.routes:
            j: Route
            self.updates_cost_and_positions(j)





    def TabuSearch(self, operator):
        solution_cost_trajectory = []
        random.seed(1)
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        top: TwoOptMove = TwoOptMove()

        SolDrawer.draw(0, self.sol, self.allNodes)

        while terminationCondition is False:
            operator = random.randint(0, 2)

            rm.Initialize()
            sm.Initialize()
            top.Initialize()

            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm, localSearchIterator)
                if rm.originRoutePosition is not None:
                    self.ApplyRelocationMove(rm, localSearchIterator)
                    if rm.originRoutePosition == rm.targetRoutePosition:
                        self.updates_cost_and_positions(self.sol.routes[rm.originRoutePosition])
                    else:
                        self.updates_cost_and_positions(self.sol.routes[rm.originRoutePosition])
                        self.updates_cost_and_positions(self.sol.routes[rm.targetRoutePosition])
            # Swaps
            elif operator == 1:
                self.FindBestSwapMove(sm, localSearchIterator)
                if sm.positionOfFirstRoute is not None:
                    self.ApplySwapMove(sm, localSearchIterator)
                    if sm.positionOfFirstRoute == sm.positionOfSecondRoute:
                        self.updates_cost_and_positions(self.sol.routes[sm.positionOfFirstRoute])
                    else:
                        self.updates_cost_and_positions(self.sol.routes[sm.positionOfFirstRoute])
                        self.updates_cost_and_positions(self.sol.routes[sm.positionOfSecondRoute])
            elif operator == 2:
                self.FindBestTwoOptMove(top, localSearchIterator)
                if top.positionOfFirstRoute is not None:
                    self.ApplyTwoOptMove(top, localSearchIterator)
                    if top.positionOfFirstRoute == top.positionOfSecondRoute:
                        self.updates_cost_and_positions(self.sol.routes[top.positionOfFirstRoute])
                    else:
                        self.updates_cost_and_positions(self.sol.routes[top.positionOfFirstRoute])
                        self.updates_cost_and_positions(self.sol.routes[top.positionOfSecondRoute])

            solution_cost_trajectory.append(self.sol.cost)

            print(localSearchIterator, self.sol.cost, self.bestSolution.cost)

            if (self.sol.cost < self.bestSolution.cost):
                self.bestSolution = self.cloneSolution(self.sol)

            # SolDrawer.draw(localSearchIterator, self.sol, self.allNodes)

            localSearchIterator = localSearchIterator + 1

            if localSearchIterator > 10000:
                terminationCondition = True

        SolDrawer.draw('final_ts', self.bestSolution, self.allNodes)
        SolDrawer.drawTrajectory(solution_cost_trajectory)

        self.sol = self.bestSolution

    def cloneSolution(self, sol: Solution):
        cloned = copy.deepcopy(sol)
        return cloned

    def cloneRoute(self, rt: Route):
        cloned = Route(self.depot, self.capacity)
        cloned.cost = rt.cost
        cloned.load = rt.load
        cloned.sequenceOfNodes = copy.deepcopy(rt.sequenceOfNodes)
        return cloned

    def TestSolution(self):
        totalSolCost = 0
        for r in range(0, len(self.sol.routes)):
            rt: Route = self.sol.routes[r]
            rtCost, rtLoad = self.calculate_route_details(rt.sequenceOfNodes)
            # if abs(rtCost - rt.cost) > 0.0001:
            #     print ('Route Cost problem')
            # if rtLoad != rt.load:
            #     print ('Route Load problem')
            totalSolCost += rtCost

        if abs(totalSolCost - self.sol.cost) > 0.0001:
            print('Solution Cost problem')

    def FindBestRelocationMove(self, rm, iterator):
        for originRouteIndex in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[originRouteIndex]
            for targetRouteIndex in range(0, len(self.sol.routes)):
                rt2: Route = self.sol.routes[targetRouteIndex]
                for originNodeIndex in range(1, len(rt1.sequenceOfNodes) - 1):
                    for targetNodeIndex in range(0, len(rt2.sequenceOfNodes) - 1):
                        if originRouteIndex == targetRouteIndex and (
                                targetNodeIndex == originNodeIndex or targetNodeIndex == originNodeIndex - 1):
                            continue
                        B: Node = rt1.sequenceOfNodes[originNodeIndex]
                        if rt1 != rt2:
                            if rt2.load + B.demand > rt2.capacity:
                                continue
                            originRtCostChange, targetRtCostChange, moveCost = self.Inner_Route_Relocation_Move(
                                originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex)
                        else:
                            moveCost = self.Intra_Route_Relocation_Move(originRouteIndex, targetRouteIndex,
                                                                        originNodeIndex, targetNodeIndex)
                            originRtCostChange = targetRtCostChange = moveCost

                        if (self.MoveIsTabu(B, iterator, moveCost)):
                            continue

                        if (moveCost < rm.moveCost):
                            self.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex,
                                                         targetNodeIndex, moveCost, originRtCostChange,
                                                         targetRtCostChange, rm)

    def Intra_Route_Relocation_Move(self, rt1Index, rt2Index, n1Index, n2Index):
        rt1 = self.sol.routes[rt1Index]
        rt2 = self.sol.routes[rt2Index]
        originNodeIndex = n1Index
        targetNodeIndex = n2Index
        A: Node = rt1.sequenceOfNodes[originNodeIndex - 1]
        B: Node = rt1.sequenceOfNodes[originNodeIndex]
        C: Node = rt1.sequenceOfNodes[originNodeIndex + 1]
        F: Node = rt2.sequenceOfNodes[targetNodeIndex]
        G: Node = rt2.sequenceOfNodes[targetNodeIndex + 1]
        cut_of_AB = len(rt1.sequenceOfNodes) - A.positionInRoute - 1
        cut_of_BC = len((rt1.sequenceOfNodes)) - B.positionInRoute - 1
        cut_of_FG = len(rt2.sequenceOfNodes) - F.positionInRoute - 1
        if originNodeIndex < targetNodeIndex:
            costRemoved = cut_of_AB * self.distanceMatrix[A.ID][B.ID] + cut_of_BC * self.distanceMatrix[B.ID][
                C.ID] + cut_of_FG * self.distanceMatrix[F.ID][G.ID]
            costAdded = cut_of_AB * self.distanceMatrix[A.ID][C.ID] + cut_of_FG * self.distanceMatrix[B.ID][
                G.ID] + (
                                cut_of_FG + 1) * self.distanceMatrix[F.ID][B.ID]
            prev = C
            for i in range(C.positionInRoute + 1, F.positionInRoute + 1):
                nex = rt1.sequenceOfNodes[i]
                costAdded += self.distanceMatrix[prev.ID][nex.ID]
                prev = nex
        else:
            costAdded = cut_of_BC * self.distanceMatrix[A.ID][C.ID] + cut_of_FG * self.distanceMatrix[F.ID][
                B.ID] + (cut_of_FG - 1) * self.distanceMatrix[B.ID][G.ID]
            costRemoved = cut_of_AB * self.distanceMatrix[A.ID][B.ID] + cut_of_BC * self.distanceMatrix[B.ID][
                C.ID] + cut_of_FG * self.distanceMatrix[F.ID][G.ID]
            prev = G
            for i in range(G.positionInRoute + 1, A.positionInRoute + 1):
                nxt = rt1.sequenceOfNodes[i]
                costRemoved += self.distanceMatrix[prev.ID][nxt.ID]
                prev = nxt
        move_cost = costAdded - costRemoved
        return move_cost

    def Inner_Route_Relocation_Move(self, rt1Index, rt2Index, node1Index, node2Index):
        rt1 = self.sol.routes[rt1Index]
        rt2 = self.sol.routes[rt2Index]
        originNodeIndex = node1Index
        targetNodeIndex = node2Index
        A: Node = rt1.sequenceOfNodes[originNodeIndex - 1]
        B: Node = rt1.sequenceOfNodes[originNodeIndex]
        C: Node = rt1.sequenceOfNodes[originNodeIndex + 1]
        F: Node = rt2.sequenceOfNodes[targetNodeIndex]
        G: Node = rt2.sequenceOfNodes[targetNodeIndex + 1]
        cut_of_AB = len(rt1.sequenceOfNodes) - A.positionInRoute - 1
        cut_of_BC = len((rt1.sequenceOfNodes)) - B.positionInRoute - 1
        cut_of_FG = len(rt2.sequenceOfNodes) - F.positionInRoute - 1
        cost_cummulative_removed_of_rt1 = 0
        for i in range(A.positionInRoute, 0, -1):
            pred = i - 1
            n1 = rt1.sequenceOfNodes[i]
            n2 = rt1.sequenceOfNodes[pred]
            cost_cummulative_removed_of_rt1 += self.distanceMatrix[n1.ID][n2.ID]
        cost_removed = cut_of_AB * self.distanceMatrix[A.ID][B.ID] + cut_of_BC * \
                       self.distanceMatrix[B.ID][C.ID]
        cost_cummulative_removed_of_rt1 += (len(rt1.sequenceOfNodes) - 2) * 10
        originRtCostChange = -cost_cummulative_removed_of_rt1 - \
                             cost_removed + cut_of_BC * self.distanceMatrix[A.ID][C.ID]
        cost_cummulative_add_of_rt2 = 0
        for i in range(F.positionInRoute, 0, -1):
            pred = i - 1
            n1 = rt2.sequenceOfNodes[i]
            n2 = rt2.sequenceOfNodes[pred]
            cost_cummulative_add_of_rt2 += self.distanceMatrix[n1.ID][n2.ID]
        cost_cummulative_add_of_rt2 += (len(rt2.sequenceOfNodes) - 1) * 10
        targetRtCostChange = cost_cummulative_add_of_rt2 - cut_of_FG * self.distanceMatrix[F.ID][
            G.ID] \
                             + cut_of_FG * self.distanceMatrix[B.ID][G.ID] \
                             + (cut_of_FG + 1) * self.distanceMatrix[F.ID][B.ID]
        moveCost = targetRtCostChange + originRtCostChange
        return originRtCostChange, targetRtCostChange, moveCost

    def FindBestSwapMove(self, sm, iterator):
        for firstRouteIndex in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[firstRouteIndex]
            for secondRouteIndex in range(firstRouteIndex, len(self.sol.routes)):
                rt2: Route = self.sol.routes[secondRouteIndex]
                for firstNodeIndex in range(1, len(rt1.sequenceOfNodes) - 1):
                    startOfSecondNodeIndex = 1
                    if rt1 == rt2:
                        startOfSecondNodeIndex = firstNodeIndex + 1
                    for secondNodeIndex in range(startOfSecondNodeIndex, len(rt2.sequenceOfNodes) - 1):

                        a1: Node = rt1.sequenceOfNodes[firstNodeIndex - 1]
                        cut_of_a1_b1 = len(rt1.sequenceOfNodes) - a1.positionInRoute - 1
                        b1 = rt1.sequenceOfNodes[firstNodeIndex]
                        c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]
                        cut_of_b1_c1 = len(rt1.sequenceOfNodes) - b1.positionInRoute - 1

                        a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                        cut_of_a2_b2 = len(rt2.sequenceOfNodes) - a2.positionInRoute - 1
                        b2 = rt2.sequenceOfNodes[secondNodeIndex]
                        cut_of_b2_c2 = len(rt2.sequenceOfNodes) - b2.positionInRoute - 1
                        c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
                        old_cst = None
                        moveCost = None
                        costChangeFirstRoute = None
                        costChangeSecondRoute = None
                        if rt1 == rt2:
                            old_cst = rt1
                            if firstNodeIndex == secondNodeIndex - 1:
                                # case of consecutive nodes swap
                                costRemoved = cut_of_a1_b1 * self.distanceMatrix[a1.ID][b1.ID] + cut_of_b1_c1 * \
                                              self.distanceMatrix[b1.ID][b2.ID] + \
                                              cut_of_b2_c2 * self.distanceMatrix[b2.ID][c2.ID]
                                costAdded = cut_of_a1_b1 * self.distanceMatrix[a1.ID][b2.ID] + cut_of_b1_c1 * \
                                            self.distanceMatrix[b2.ID][b1.ID] + \
                                            cut_of_b2_c2 * self.distanceMatrix[b1.ID][c2.ID]
                                moveCost = costAdded - costRemoved
                            else:

                                costRemoved1 = cut_of_a1_b1 * self.distanceMatrix[a1.ID][b1.ID] + cut_of_b1_c1 * \
                                               self.distanceMatrix[b1.ID][c1.ID]
                                costAdded1 = cut_of_a1_b1 * self.distanceMatrix[a1.ID][b2.ID] + cut_of_b1_c1 * \
                                             self.distanceMatrix[b2.ID][c1.ID]
                                costRemoved2 = cut_of_a2_b2 * self.distanceMatrix[a2.ID][b2.ID] + cut_of_b2_c2 * \
                                               self.distanceMatrix[b2.ID][c2.ID]
                                costAdded2 = cut_of_a2_b2 * self.distanceMatrix[a2.ID][b1.ID] + cut_of_b2_c2 * \
                                             self.distanceMatrix[b1.ID][c2.ID]
                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                        else:
                            if rt1.load - b1.demand + b2.demand > self.capacity:
                                continue
                            if rt2.load - b2.demand + b1.demand > self.capacity:
                                continue
                            old_cst = rt1.cost + rt2.cost
                            costRemoved1 = cut_of_a1_b1 * self.distanceMatrix[a1.ID][b1.ID] + cut_of_b1_c1 * \
                                           self.distanceMatrix[b1.ID][c1.ID]
                            costAdded1 = cut_of_a1_b1 * self.distanceMatrix[a1.ID][b2.ID] + cut_of_b1_c1 * \
                                         self.distanceMatrix[b2.ID][c1.ID]
                            costRemoved2 = cut_of_a2_b2 * self.distanceMatrix[a2.ID][b2.ID] + cut_of_b2_c2 * \
                                           self.distanceMatrix[b2.ID][c2.ID]
                            costAdded2 = cut_of_a2_b2 * self.distanceMatrix[a2.ID][b1.ID] + cut_of_b2_c2 * \
                                         self.distanceMatrix[b1.ID][c2.ID]

                            costChangeFirstRoute = costAdded1 - costRemoved1
                            costChangeSecondRoute = costAdded2 - costRemoved2

                            moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)

                        if self.MoveIsTabu(b1, iterator, moveCost) or self.MoveIsTabu(b2, iterator, moveCost):
                            continue

                        if moveCost < sm.moveCost:
                            self.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex,
                                                   moveCost, costChangeFirstRoute, costChangeSecondRoute, sm)

    def ApplyRelocationMove(self, rm: RelocationMove, iterator):

        originRt = self.sol.routes[rm.originRoutePosition]
        targetRt = self.sol.routes[rm.targetRoutePosition]

        B = originRt.sequenceOfNodes[rm.originNodePosition]

        if originRt == targetRt:
            del originRt.sequenceOfNodes[rm.originNodePosition]
            if (rm.originNodePosition < rm.targetNodePosition):
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition, B)
            else:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            # self.updates_cost_and_positions(originRt)
            originRt.cost += rm.moveCost
        else:
            del originRt.sequenceOfNodes[rm.originNodePosition]
            targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            originRt.cost += rm.costChangeOriginRt
            targetRt.cost += rm.costChangeTargetRt
            originRt.load -= B.demand
            targetRt.load += B.demand
            # self.updates_cost_and_positions(originRt)
            # self.updates_cost_and_positions(targetRt)

        self.sol.cost += rm.moveCost

        self.SetTabuIterator(B, iterator)

    def ApplySwapMove(self, sm, iterator):
        rt1 = self.sol.routes[sm.positionOfFirstRoute]
        rt2 = self.sol.routes[sm.positionOfSecondRoute]
        b1: Node = rt1.sequenceOfNodes[sm.positionOfFirstNode]
        b2: Node = rt2.sequenceOfNodes[sm.positionOfSecondNode]
        x1 = b1.positionInRoute
        x2 = b2.positionInRoute
        b1.positionInRoute = x1
        b2.positionInRoute = x2
        rt1.sequenceOfNodes[sm.positionOfFirstNode] = b2
        rt2.sequenceOfNodes[sm.positionOfSecondNode] = b1
        rt1.sequenceOfNodes[sm.positionOfFirstNode].positionInRoute = sm.positionOfFirstNode
        rt2.sequenceOfNodes[sm.positionOfSecondNode].positionInRoute = sm.positionOfSecondNode
        if (rt1 == rt2):
            rt1.cost += sm.moveCost
            # self.updates_cost_and_positions(rt1)
        else:
            rt1.cost += sm.costChangeFirstRt
            rt2.cost += sm.costChangeSecondRt
            rt1.load = rt1.load - b1.demand + b2.demand
            rt2.load = rt2.load + b1.demand - b2.demand
            # self.updates_cost_and_positions(rt1)
            # self.updates_cost_and_positions(rt2)

        self.sol.cost += sm.moveCost


        self.SetTabuIterator(b1, iterator)
        self.SetTabuIterator(b2, iterator)

    def ReportSolution(self, sol):
        tc = 0
        for i in range(0, len(sol.routes)):
            rt = sol.routes[i]
            for j in range(0, len(rt.sequenceOfNodes)):
                if j == 0:
                    print(rt.sequenceOfNodes[j].ID, end=' ')
                else:
                    print("," + str(rt.sequenceOfNodes[j].ID), end=' ')
            print(rt.cost, rt.sequenceOfNodes[-1].cost_up_to_here)
            tc += rt.cost
        print(tc)

    def GetLastOpenRoute(self):
        if len(self.sol.routes) == 0:
            return None
        else:
            return self.sol.routes[-1]

    def IdentifyBestInsertion(self, bestInsertion, rt):
        for i in range(0, len(self.customers)):
            candidateCust: Node = self.customers[i]
            if candidateCust.isRouted is False:
                if rt.load + candidateCust.demand <= rt.capacity:
                    lastNodePresentInTheRoute = rt.sequenceOfNodes[-2]
                    trialCost = self.distanceMatrix[lastNodePresentInTheRoute.ID][candidateCust.ID]
                    if trialCost < bestInsertion.cost:
                        bestInsertion.customer = candidateCust
                        bestInsertion.route = rt
                        bestInsertion.cost = trialCost

    def ApplyCustomerInsertion(self, insertion):
        insCustomer = insertion.customer
        rt = insertion.route
        # before the second depot occurrence
        insIndex = len(rt.sequenceOfNodes) - 1
        rt.sequenceOfNodes.insert(insIndex, insCustomer)

        beforeInserted = rt.sequenceOfNodes[-3]

        costAdded = self.distanceMatrix[beforeInserted.ID][insCustomer.ID] + self.distanceMatrix[insCustomer.ID][
            self.depot.ID]
        costRemoved = self.distanceMatrix[beforeInserted.ID][self.depot.ID]

        rt.cost += costAdded - costRemoved
        self.sol.cost += costAdded - costRemoved

        rt.load += insCustomer.demand

        insCustomer.isRouted = True

    def StoreBestRelocationMove(self, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost,
                                originRtCostChange, targetRtCostChange, rm: RelocationMove):
        rm.originRoutePosition = originRouteIndex
        rm.originNodePosition = originNodeIndex
        rm.targetRoutePosition = targetRouteIndex
        rm.targetNodePosition = targetNodeIndex
        rm.costChangeOriginRt = originRtCostChange
        rm.costChangeTargetRt = targetRtCostChange
        rm.moveCost = moveCost

    def StoreBestSwapMove(self, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost,
                          costChangeFirstRoute, costChangeSecondRoute, sm):
        sm.positionOfFirstRoute = firstRouteIndex
        sm.positionOfSecondRoute = secondRouteIndex
        sm.positionOfFirstNode = firstNodeIndex
        sm.positionOfSecondNode = secondNodeIndex
        sm.costChangeFirstRt = costChangeFirstRoute
        sm.costChangeSecondRt = costChangeSecondRoute
        sm.moveCost = moveCost

    def CalculateTotalCost(self, sol):
        c = 0
        for i in range(0, len(sol.routes)):
            rt = sol.routes[i]
            ci, l = self.calculate_route_details(rt.sequenceOfNodes)
            c += ci
        return c

    def calculate_route_details(self, nodes_sequence):
        rt_load = 0
        rt_cumulative_cost = 0
        tot_time = 0
        for i in range(len(nodes_sequence) - 1):
            from_node = nodes_sequence[i]
            to_node = nodes_sequence[i + 1]
            tot_time += self.distanceMatrix[from_node.ID][to_node.ID]
            rt_cumulative_cost += tot_time
            tot_time += 10
            rt_load += from_node.demand
        return rt_cumulative_cost, rt_load

    def MoveIsTabu(self, n: Node, iterator, moveCost):
        if moveCost + self.sol.cost < self.bestSolution.cost - 0.001:
            return False
        if iterator < n.isTabuTillIterator:
            return True
        return False

    def SetTabuIterator(self, n: Node, iterator):
        # n.isTabuTillIterator = iterator + self.tabuTenure
        n.isTabuTillIterator = iterator + random.randint(self.minTabuTenure, self.maxTabuTenure)

    def FindBestTwoOptMove(self, top, iterator):
        for rtInd1 in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[rtInd1]
            for rtInd2 in range(rtInd1, len(self.sol.routes)):
                rt2: Route = self.sol.routes[rtInd2]
                for nodeInd1 in range(0, len(rt1.sequenceOfNodes) - 1):
                    start2 = 0
                    if (rt1 == rt2):
                        start2 = nodeInd1 + 2

                    for nodeInd2 in range(start2, len(rt2.sequenceOfNodes) - 1):
                        moveCost = 10 ** 9

                        A = rt1.sequenceOfNodes[nodeInd1]
                        B = rt1.sequenceOfNodes[nodeInd1 + 1]
                        K = rt2.sequenceOfNodes[nodeInd2]
                        L = rt2.sequenceOfNodes[nodeInd2 + 1]

                        if rt1 == rt2:
                            if nodeInd1 == 0 and nodeInd2 == len(rt1.sequenceOfNodes) - 2:
                                continue

                            moveCost = self.same_route_two_opt(nodeInd1, nodeInd2, rt1)
                        else:
                            if nodeInd1 == 0 and nodeInd2 == 0:
                                # NEED TO BE AND
                                # IS A PROBLEM THAT IF ONE OF THE NODEIND =0 THEN THE COST IS NOT CALCULATED PROPERLY
                                # TODO FIX
                                continue
                            if nodeInd1 == len(rt1.sequenceOfNodes) - 2 and nodeInd2 == len(rt2.sequenceOfNodes) - 2:
                                continue

                            if self.CapacityIsViolated(rt1, nodeInd1, rt2, nodeInd2):
                                continue
                            cost_old = rt1.cost + rt2.cost
                            cost_new = self.calculate_two_opt_cost(rtInd1, rtInd2, nodeInd1, nodeInd2)
                            moveCost = cost_new - cost_old

                        if self.MoveIsTabu(A, iterator, moveCost) or self.MoveIsTabu(K, iterator, moveCost):
                            continue

                        if moveCost < top.moveCost:
                            self.StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top)

    def calculate_two_opt_cost(self, positionOfFirstRoute, positionOfSecondRoute, positionOfFirstNode,
                               positionOfSecondNode):
        rt1: Route = self.sol.routes[positionOfFirstRoute]
        rt2: Route = self.sol.routes[positionOfSecondRoute]
        n1: Node = rt1.sequenceOfNodes[positionOfFirstNode]
        n2: Node = rt2.sequenceOfNodes[positionOfSecondNode]
        relocated_segment1 = rt1.sequenceOfNodes[positionOfFirstNode + 1:]
        relocated_segment2 = rt2.sequenceOfNodes[positionOfSecondNode + 1:]
        rt_cumulative_cost = n1.cost_up_to_here
        tot_time = n1.waitingtime
        from_node = n1
        for i in range(len(relocated_segment2)):
            to_node = relocated_segment2[i]
            tot_time += self.distanceMatrix[from_node.ID][to_node.ID]
            tot_time += 10
            rt_cumulative_cost += tot_time
            from_node = to_node
        rt1_new_cost = rt_cumulative_cost
        rt_cumulative_cost = n2.cost_up_to_here
        tot_time = n2.waitingtime
        from_node = n2
        for i in range(len(relocated_segment1)):
            to_node = relocated_segment1[i]
            tot_time += self.distanceMatrix[from_node.ID][to_node.ID]
            tot_time += 10
            rt_cumulative_cost += tot_time
            from_node = to_node
        rt2_new_cost = rt_cumulative_cost
        return rt1_new_cost + rt2_new_cost

    def same_route_two_opt(self, node1Index, node2Index, rt: Route):
        reversedSegment = reversed(
            rt.sequenceOfNodes[node1Index + 1: node2Index + 1])
        relocated_segment = list(reversedSegment)
        prev: Node = rt.sequenceOfNodes[node1Index]
        cost_added = 0
        times = len(rt.sequenceOfNodes) - node1Index - 1
        for i in range(0, len(relocated_segment)):
            next_node: Node = relocated_segment[i]
            cost_added += times * self.distanceMatrix[prev.ID][next_node.ID]
            prev = next_node
            times -= 1
        cost_added += times * self.distanceMatrix[relocated_segment[-1].ID][rt.sequenceOfNodes[node2Index + 1].ID]
        prev: Node = rt.sequenceOfNodes[node1Index]
        cost_removed = 0
        times = len(rt.sequenceOfNodes) - prev.positionInRoute - 1
        for i in range(node1Index + 1, node2Index + 2):
            next_node: Node = rt.sequenceOfNodes[i]
            cost_removed += times * self.distanceMatrix[prev.ID][next_node.ID]
            prev = next_node
            times -= 1
        move_cost = cost_added - cost_removed
        return move_cost

    def CapacityIsViolated(self, rt1, nodeInd1, rt2, nodeInd2):

        rt1FirstSegmentLoad = 0
        for i in range(0, nodeInd1 + 1):
            n = rt1.sequenceOfNodes[i]
            rt1FirstSegmentLoad += n.demand
        rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad

        rt2FirstSegmentLoad = 0
        for i in range(0, nodeInd2 + 1):
            n = rt2.sequenceOfNodes[i]
            rt2FirstSegmentLoad += n.demand
        rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad

        if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity):
            return True
        if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity):
            return True

        return False

    def ApplyTwoOptMove(self, top, iterator):
        rt1: Route = self.sol.routes[top.positionOfFirstRoute]
        rt2: Route = self.sol.routes[top.positionOfSecondRoute]
        if rt1 == rt2:
            # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
            reversedSegment = reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
            # lst = list(reversedSegment)
            # lst2 = list(reversedSegment)
            rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegment

            # reversedSegmentList = list(reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1]))
            # rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegmentList

            # self.updates_cost_and_positions(rt1)
            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfFirstNode], iterator)
            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfSecondNode], iterator)

            rt1.cost += top.moveCost

        else:
            # slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1:]

            # slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1:]

            del rt1.sequenceOfNodes[top.positionOfFirstNode + 1:]
            del rt2.sequenceOfNodes[top.positionOfSecondNode + 1:]

            rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
            rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)
            # self.updates_cost_and_positions(rt1)
            # self.updates_cost_and_positions(rt2)

            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfFirstNode], iterator)
            self.SetTabuIterator(rt2.sequenceOfNodes[top.positionOfSecondNode], iterator)

        self.sol.cost += top.moveCost

    def UpdateRouteCostAndLoad(self, rt: Route):
        tc = 0
        tl = 0
        for i in range(0, len(rt.sequenceOfNodes) - 1):
            A = rt.sequenceOfNodes[i]
            B = rt.sequenceOfNodes[i + 1]
            tc += self.distanceMatrix[A.ID][B.ID]
            tl += A.demand
        rt.load = tl
        rt.cost = tc

    def updates_cost_and_positions(self, route: Route):
        node_before = route.sequenceOfNodes[0]
        route.cost = 0
        route.load = 0
        for i in range(1, len(route.sequenceOfNodes)):
            # zero everything
            n1: Node = route.sequenceOfNodes[i]
            n1.waitingtime = 0
            n1.cost_up_to_here = 0
            n1.positionInRoute = 0
            if i > 1:
                n1.waitingtime += 10
            n1.waitingtime += node_before.waitingtime + self.distanceMatrix[node_before.ID][n1.ID]
            route.cost += n1.waitingtime
            route.load += n1.demand
            n1.cost_up_to_here = route.cost
            n1.positionInRoute = i
            node_before = n1

    def calculate_cost(self, rt: Route):
        cost = 0
        for i in range(0, len(rt.sequenceOfNodes) - 1):
            n = len(rt.sequenceOfNodes) - i - 1
            cost += n * self.distanceMatrix[rt.sequenceOfNodes[i].ID][rt.sequenceOfNodes[i + 1].ID]
        n = len(rt.sequenceOfNodes) - 2
        cummulative = n * (n + 1) / 2
        cost += cummulative * 10
        return cost

    def StoreBestTwoOptMove(self, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top):
        top.positionOfFirstRoute = rtInd1
        top.positionOfSecondRoute = rtInd2
        top.positionOfFirstNode = nodeInd1
        top.positionOfSecondNode = nodeInd2
        top.moveCost = moveCost
