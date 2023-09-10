import copy
from Map import *
import pygame
import numpy as np
import math
import time
import spectra

fps = 1000
node_width = 5
node_height = 5
# List of map/task numbers to run, 1-6
tasks_todo = [6]
# Scalar default 1, using manhattan distance
heuristicScalar = 1


class Node:
    def __init__(self, x, y, nodeMatrix, weight):
        self.nodeMatrix: NodeMap = nodeMatrix
        self.x = x
        self.y = y
        self.weight = weight
        self.path = []
        self.parent: Node = None
        self.pathCost = math.inf
        self.estimatedCost = math.inf

        if self.weight < 0:
            self.weight = math.inf

    def getWeight(self):
        return self.weight

    # returns list of Node neighbour
    def getNodeNeighbours(self):
        list = []
        prunedList = []
        north: Node = self.nodeMatrix.getNode(self.x, self.y - 1)
        east: Node = self.nodeMatrix.getNode(self.x + 1, self.y)
        south: Node = self.nodeMatrix.getNode(self.x, self.y + 1)
        west: Node = self.nodeMatrix.getNode(self.x - 1, self.y)
        list = [north, east, south, west]
        for node in list:
            if node != None:
                if node.getWeight != math.inf:
                    prunedList.append(node)
        return prunedList

    def calculatePathCost(self):
        if self == self.nodeMatrix.getStart():
            return 0
        if self.parent == None:
            return math.inf
        return self.parent.calculatePathCost() + self.getWeight()

    def setPathCost(self, cost):
        self.pathCost = cost

    def getPathCost(self):
        return self.pathCost

    def getEstimatedCost(self):
        return self.estimatedCost

    def setEstimatedCost(self, cost):
        self.estimatedCost = cost

    def setParent(self, parent):
        self.parent = parent

    def getHeuristicCost(self):
        return self.nodeMatrix.heuristicCost(self)

    def __str__(self):
        return "{x:" + str(self.x) + ", y:" + str(self.y) + ", pathCost:" + str(self.calculatePathCost()) + ", heuristic:" + str(self.getHeuristicCost()) + "}"


class NodeMap:
    def __init__(self, mapObj: Map_Obj):
        nodeMatrix = []
        self.mapObj = mapObj
        self.width = mapObj.get_maps()[0].shape[1]
        self.height = mapObj.get_maps()[0].shape[0]
        for y in range(self.height):
            nodeMatrixRow = []
            for x in range(self.width):
                nodeMatrixRow.append(
                    Node(x, y, self, mapObj.get_cell_value([y, x])))
            nodeMatrix.append(nodeMatrixRow)
        self.nodeMatrix = nodeMatrix

        self.start = self.getNode(
            mapObj.get_start_pos()[1], mapObj.get_start_pos()[0])
        self.goal = self.getNode(mapObj.get_goal_pos()[
                                 1], mapObj.get_goal_pos()[0])
        self.start.setEstimatedCost(0)

    def updateGoalPositionWithTick(self):
        self.mapObj.tick()
        self.goal = self.getNode(self.mapObj.get_goal_pos()[
                                 1], self.mapObj.get_goal_pos()[0])
        return None

    def getNode(self, x, y):
        if self.width <= x or x < 0:
            return None
        if self.height <= y or y < 0:
            return None
        return self.nodeMatrix[y][x]

    def getMatrix(self):
        return self.nodeMatrix

    def getStart(self):
        return self.start

    def getGoal(self):
        return self.goal

    def euclidianDistance(self, node1: Node, node2: Node):
        point1 = np.array((node1.x, node1.y))
        point2 = np.array((node2.x, node2.y))
        dist = np.linalg.norm(point1 - point2)
        return dist

    def manhattanDistance(self, node1: Node, node2: Node):
        point1 = np.array((node1.x, node1.y))
        point2 = np.array((node2.x, node2.y))
        manhattan_distance = np.sum(np.abs(point1-point2))
        return manhattan_distance

    def heuristicCost(self, node: Node):
        # return self.euclidianDistance(node, self.goal)
        return self.manhattanDistance(node, self.goal)

    def getPath(self, node: Node):
        path = []
        path.append(node)
        while node.parent != None:
            path.append(node.parent)
            node = node.parent
        return path


# Returns node path from start to goal
def AStarSearch(nodeMap: NodeMap, mapObj: Map_Obj):
    start: Node = nodeMap.getStart()
    goal: Node = nodeMap.getGoal()
    if start == goal:
        print("Start is equal to goal")
        return []
    else:
        # Queue of nodes to search
        frontier: list[Node] = []
        # List of reached nodes with
        reached: list[Node] = []

        frontier.append(start)
        prevGoal: Node = None
        while goal not in reached and len(frontier) > 0:
            chosenNode = frontier[0]
            # Update all estimated costs if goal moved
            if goal != prevGoal:
                for n in frontier:
                    n.setEstimatedCost(
                        n.getHeuristicCost() + n.calculatePathCost())
            # get lowest total cost node and explore it
            for node in frontier:
                if node.getEstimatedCost() < chosenNode.getEstimatedCost():
                    chosenNode = node
            # print("chosen node:")
            # print(chosenNode)

            neighbours: list[Node] = chosenNode.getNodeNeighbours()
            for n in neighbours:
                if n not in reached:
                    if n != chosenNode:
                        # Check if new distance to node is shorter

                        # Set path cost with parent
                        chosenNode.setPathCost(chosenNode.calculatePathCost())

                        # new path cost for n
                        newCost = chosenNode.getPathCost() + n.getWeight()
                        oldCost = n.calculatePathCost()

                        if (newCost < oldCost) or (oldCost == math.inf and n.getWeight() != math.inf and n.getWeight() > 0):
                            # update total cost and parent
                            n.setParent(chosenNode)
                            n.setEstimatedCost(
                                n.getHeuristicCost() * heuristicScalar + n.calculatePathCost())
                        if n not in frontier:
                            frontier.append(n)

            frontier.remove(chosenNode)
            reached.append(chosenNode)

            # Moving goal
            nodeMap.updateGoalPositionWithTick()
            prevGoal = goal
            goal = nodeMap.getGoal()

        if len(frontier) == 0:
            print("frontier empty")
            for x in range(nodeMap.width):
                for y in range(nodeMap.height):
                    print(nodeMap.getNode(x, y))
        # print("Goal Path Cost: {}".format(goal.getPathCost()))
        return (nodeMap.getPath(goal), reached, goal.getPathCost())


def drawMap(nodeMapFromAStar: NodeMap, goalPath, reached: list[Node], mapObj: Map_Obj):
    nmap = NodeMap(mapObj)
    nodeMatrix = nmap.getMatrix()
    # Settings
    updateFps = fps
    fpsClock = pygame.time.Clock()

    grid_node_width = node_width
    grid_node_height = node_height
    length = grid_node_height*len(nodeMatrix[0])
    width = grid_node_width*len(nodeMatrix)

    gridDisplay = pygame.display.set_mode((length, width))

    # Background color
    gridDisplay.fill((255, 224, 181))

    # Settings End

    def createSquare(x, y, color):
        pygame.draw.rect(gridDisplay, color,
                         (x, y, grid_node_width, grid_node_height))

    walkedNodes: list[Node] = []

    def drawInitialMap():
        # Draw Initial Map
        for y in range(len(nodeMatrix)):
            for x in range(len(nodeMatrix[0])):
                node: Node = nmap.getNode(x, y)
                # Walls
                if node.getWeight() == math.inf:
                    createSquare(x*grid_node_width, y *
                                 grid_node_height, (12, 12, 12))
                # Walkable squares
                elif node.getWeight() >= 0:
                    weight = node.getWeight()
                    reduction = 50*weight
                    clr = 255 - reduction
                    createSquare(x*grid_node_width, y *
                                 grid_node_height, (clr, clr, clr))

        # Draw Start
        createSquare(nmap.getStart().x*grid_node_width,
                     nmap.getStart().y*grid_node_height, (255, 255, 0))
        # Draw Goal
        createSquare(nmap.getGoal().x*grid_node_width,
                     nmap.getGoal().y*grid_node_height, (0, 255, 0))

    scale = spectra.scale(["blue", "#ff4d4d"])
    colorRange = scale.range(len(reached))

    totalRange = colorRange

    def drawUpdates():
        if len(walkedNodes) > 0:
            node = walkedNodes[-1]
            # Calculate color for index of current node
            reachedIndex = reached.index(node)
            createSquare(node.x*grid_node_width, node.y *
                         grid_node_height, (totalRange[reachedIndex].hexcode))

    # First Draw of map
    drawInitialMap()
    pygame.display.update()

    # # Display  weight only for reached nodes text on grid
    # pygame.font.init()
    # white = (255, 255, 255)
    # green = (0, 255, 0)
    # blue = (0, 0, 128)

    # for node in reached:
    #     x = node.x*grid_node_width
    #     y = node.y*grid_node_height
    #     font = pygame.font.SysFont('timesnewroman',  8)
    #     text = font.render(str(node.getPathCost()), True, (0, 0, 0), None)
    #     textRect = text.get_rect()
    #     textRect.x = x + grid_node_width
    #     textRect.y = y + grid_node_height
    #     textRect.center = (x + grid_node_width / 2, y + grid_node_height / 2)
    #     gridDisplay.blit(text, textRect)
    # pygame.display.update()
    # time.sleep(3)

    def convertEuclidToRGB(euclidDistance):
        # Max euclidian distance
        OldMax = nmap.euclidianDistance(nmap.getNode(0, 0), nodeMatrix[-1][-1])
        OldMin = 0
        # Max RGB
        NewMax = 255
        NewMin = 0
        OldRange = (OldMax - OldMin)
        NewRange = (NewMax - NewMin)
        NewValue = (((euclidDistance - OldMin) * NewRange) / OldRange) + NewMin
        return NewValue

    # # Draw euclid distance to goal
    # for y in range(len(nodeMatrix)):
    #     for x in range(len(nodeMatrix[0])):
    #         node: Node = nmap.getNode(x, y)
    #         # Walkable squares
    #         if node.getWeight() != math.inf:
    #             euclidDistance = nmap.euclidianDistance(node, nmap.getGoal())
    #             rgb = convertEuclidToRGB(euclidDistance)
    #             createSquare(x*grid_node_width, y *
    #                          grid_node_height, (rgb, rgb, rgb))
    #         # Walls
    #         elif node.getWeight() == math.inf:
    #             createSquare(x*grid_node_width, y *
    #                          grid_node_height, (100, 0, 0))
    # pygame.display.update()
    # time.sleep(3)

    def redrawMovedGoal(prevGoal: Node, newGoal: Node):
        # Draw white over previous goal
        weight = prevGoal.getWeight()
        reduction = 50*weight
        clr = 255 - reduction
        createSquare(prevGoal.x*grid_node_width, prevGoal.y *
                     grid_node_height, (clr, clr, clr))
        # Draw new goal
        createSquare(newGoal.x*grid_node_width, newGoal.y *
                     grid_node_height, (0, 255, 0))
        return None

    # Draw reached/exploration
    for node in reached:
        pygame.event.get()
        fpsClock.tick(updateFps)
        # Dont draw over start node
        if node != goalPath[-1]:
            walkedNodes.append(node)
        prevGoal = nmap.getGoal()
        mapObj.tick()
        nmap.goal = nmap.getNode(nmap.mapObj.get_goal_pos()[
                                 1], nmap.mapObj.get_goal_pos()[0])
        if prevGoal != nmap.getGoal():
            redrawMovedGoal(prevGoal, nmap.getGoal())
        drawUpdates()
        pygame.display.update()
        pygame.display.set_caption("HeuristicScalar: {}, FPS: {}".format(
            heuristicScalar, fpsClock.get_fps()))

    # Draw path found to goal
    for node in goalPath:
        pygame.event.get()
        fpsClock.tick(updateFps)
        createSquare(node.x*grid_node_width, node.y *
                     grid_node_height, (39, 165, 98))
        pygame.display.update()


def calculateOptimalHeuristicScalarForTask(taskNumber: int):
    global heuristicScalar
    # Start scalar value
    heuristicScalar = 1
    currentOptimalScalar = heuristicScalar
    currentBestPathCost = math.inf
    currentBestReachedSize = math.inf
    scalarIncrement = 0.1
    for i in range(10):
        map = Map_Obj(taskNumber)
        nodeMatrix = NodeMap(map)
        goalPath, reached, goalPathCost = AStarSearch(
            NodeMap(map), map)
        # drawMap(nodeMatrix, goalPath, reached, Map_Obj(taskNumber))
        if goalPathCost == math.inf:
            break
        if goalPathCost < currentBestPathCost:
            currentBestReachedSize = len(reached)
            currentBestPathCost = goalPathCost
            currentOptimalScalar = heuristicScalar
        elif goalPathCost == currentBestPathCost and len(reached) < currentBestReachedSize:
            currentBestReachedSize = len(reached)
            currentBestPathCost = goalPathCost
            currentOptimalScalar = heuristicScalar
        print("HeuristicScalar: {}, GoalPathCost: {}, ReachedSize: {}".format(
            heuristicScalar, goalPathCost, len(reached)))
        heuristicScalar += scalarIncrement
    return currentOptimalScalar, currentBestPathCost, currentBestReachedSize


def main():
    sleepTime = 5
    for task in tasks_todo:
        map = Map_Obj(task)
        nodeMatrix = NodeMap(map)
        goalPath, reached, goalPathCost = AStarSearch(NodeMap(map), map)
        drawMap(nodeMatrix, goalPath, reached, Map_Obj(task))
        time.sleep(sleepTime)
    # print(calculateOptimalHeuristicScalarForTask(6))


if __name__ == "__main__":
    main()
