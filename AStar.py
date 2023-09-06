import copy
from Map import *
import pygame
import numpy as np
import math
import time


class Node:
    def __init__(self, x, y, nodeMatrix, weight):
        self.nodeMatrix: NodeMatrix = nodeMatrix
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


class NodeMatrix:
    def __init__(self, map: Map_Obj):
        nodeMatrix = []
        self.width = map.get_maps()[0].shape[1]
        self.height = map.get_maps()[0].shape[0]
        for y in range(self.height):
            nodeMatrixRow = []
            for x in range(self.width):
                nodeMatrixRow.append(
                    Node(x, y, self, map.get_cell_value([y, x])))
            nodeMatrix.append(nodeMatrixRow)
        self.nodeMatrix = nodeMatrix

        self.start = self.getNode(
            map.get_start_pos()[1], map.get_start_pos()[0])
        self.goal = self.getNode(map.get_goal_pos()[1], map.get_goal_pos()[0])
        self.start.setEstimatedCost(0)

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

    def heuristicCost(self, node: Node):
        return self.euclidianDistance(node, self.goal)

    def getPath(self, node: Node):
        path = []
        path.append(node)
        while node.parent != None:
            path.append(node.parent)
            node = node.parent
        return path


# Returns node path from start to goal
def AStarSearch(nodeMatrix: NodeMatrix):
    start: Node = nodeMatrix.getStart()
    goal: Node = nodeMatrix.getGoal()
    if start == goal:
        print("Start is equal to goal")
        return []
    else:
        # Queue of nodes to search
        frontier: list[Node] = []
        # List of reached nodes with
        reached: list[Node] = []

        frontier.append(start)

        while goal not in reached and len(frontier) > 0:
            chosenNode = frontier[0]
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
                                n.getHeuristicCost() + n.calculatePathCost())
                        if n not in frontier:
                            frontier.append(n)

            frontier.remove(chosenNode)
            reached.append(chosenNode)

        if len(frontier) == 0:
            print("frontier empty")
            for x in range(nodeMatrix.width):
                for y in range(nodeMatrix.height):
                    print(nodeMatrix.getNode(x, y))
        print("Goal Path Cost: {}".format(goal.getPathCost()))
        return (nodeMatrix.getPath(goal), reached)


def drawMap(nodeMatrix: NodeMatrix, goalPath, reached: list[Node]):
    nodeMap = nodeMatrix.getMatrix()
    # Settings
    fps = 60
    fpsClock = pygame.time.Clock()

    length = 20*len(nodeMap[0])
    width = 20*len(nodeMap)
    grid_node_width = 20
    grid_node_height = 20
    gridDisplay = pygame.display.set_mode((length, width))

    # Settings End

    def createSquare(x, y, color):
        pygame.draw.rect(gridDisplay, color,
                         (x, y, grid_node_width, grid_node_height))

    # Draw Map
    for y in range(len(nodeMap)):
        for x in range(len(nodeMap[0])):
            node: Node = nodeMatrix.getNode(x, y)
            # Walls
            if node.getWeight() == math.inf:
                createSquare(x*grid_node_width, y *
                             grid_node_height, (100, 0, 0))
            # Walkable squares
            elif node.getWeight() >= 0:
                weight = node.getWeight()
                reduction = 50*weight
                clr = 255 - reduction
                createSquare(x*grid_node_width, y *
                             grid_node_height, (clr, clr, clr))

    createSquare(nodeMatrix.getStart().x*grid_node_width,
                 nodeMatrix.getStart().y*grid_node_height, (0, 0, 100))
    createSquare(nodeMatrix.getGoal().x*grid_node_width,
                 nodeMatrix.getGoal().y*grid_node_height, (0, 255, 0))

    pygame.display.update()

    # Display  weight only for reached nodes text on grid
    pygame.font.init()
    white = (255, 255, 255)
    green = (0, 255, 0)
    blue = (0, 0, 128)
    # for node in reached:
    #     x = node.x*grid_node_width
    #     y = node.y*grid_node_height
    #     font = pygame.font.SysFont('timesnewroman',  8)
    #     text = font.render(str(node.getWeight()), True, (0, 0, 0), None)
    #     textRect = text.get_rect()
    #     textRect.x = x + grid_node_width
    #     textRect.y = y + grid_node_height
    #     textRect.center = (x + grid_node_width / 2, y + grid_node_height / 2)
    #     gridDisplay.blit(text, textRect)

    # Display Weights
    # flattenedNodeMatrix = np.array(nodeMap)
    # flattenedNodeMatrix = flattenedNodeMatrix.flatten()
    # for node in flattenedNodeMatrix:
    #     x = node.x*grid_node_width
    #     y = node.y*grid_node_height
    #     font = pygame.font.SysFont('timesnewroman',  8)
    #     text = font.render(str(node.getWeight()), True, (0, 0, 0), None)
    #     textRect = text.get_rect()
    #     textRect.x = x + grid_node_width
    #     textRect.y = y + grid_node_height
    #     textRect.center = (x + grid_node_width / 2, y + grid_node_height / 2)
    #     gridDisplay.blit(text, textRect)

    pygame.display.update()

    time.sleep(2)

    for node in reached:
        fpsClock.tick(fps)
        if node != goalPath[-1]:
            createSquare(node.x*grid_node_width, node.y *
                         grid_node_height, (0, 0, 255))
        pygame.display.update()

    for node in goalPath:
        fpsClock.tick(fps)
        createSquare(node.x*grid_node_width, node.y *
                     grid_node_height, (0, 100, 0))
        pygame.display.update()


if __name__ == "__main__":
    # map = Map_Obj(1)
    # nodeMatrix = NodeMatrix(map)
    # goalPath, reached = AStarSearch(NodeMatrix(Map_Obj(1)))
    # drawMap(nodeMatrix, goalPath, reached)
    # time.sleep(5)

    # map = Map_Obj(2)
    # nodeMatrix = NodeMatrix(map)
    # goalPath, reached = AStarSearch(NodeMatrix(map))
    # drawMap(nodeMatrix, goalPath, reached)
    # time.sleep(5)

    # map = Map_Obj(3)
    # nodeMatrix = NodeMatrix(map)
    # goalPath, reached = AStarSearch(NodeMatrix(map))
    # drawMap(nodeMatrix, goalPath, reached)
    # time.sleep(5)

    map = Map_Obj(4)
    nodeMatrix = NodeMatrix(map)
    goalPath, reached = AStarSearch(NodeMatrix(map))
    drawMap(nodeMatrix, goalPath, reached)
    time.sleep(3)
