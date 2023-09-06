from Map import *
import pygame
import numpy as np
import math
import time

map = Map_Obj(1)

# print(map.get_maps()[0])
print(map.get_goal_pos())
print(map.get_start_pos())
print("Width:" + str(len(map.get_maps()[0][0])))  # 39
print("Height:" + str(len(map.get_maps()[0])))  # 47


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

    def getPath(self, path: list = []):
        if self.parent == None:
            return path
        path.append(self)
        self.parent.getPath(path)
        return path

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

        self.goal = Node(map.get_goal_pos()[1], map.get_goal_pos()[
                         0], self, map.get_cell_value([map.get_goal_pos()[0], map.get_goal_pos()[1]]))
        self.start = Node(map.get_start_pos()[1], map.get_start_pos()[
                          0], self, map.get_cell_value([map.get_start_pos()[0], map.get_start_pos()[1]]))
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


# Returns node path from start to goal
def AStarSearch(nodeMatrix: NodeMatrix):
    start = nodeMatrix.getStart()
    goal = nodeMatrix.getGoal()
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

            frontier.remove(chosenNode)
            reached.append(chosenNode)

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
        if len(frontier) == 0:
            print("frontier empty")
            for x in range(nodeMatrix.width):
                for y in range(nodeMatrix.height):
                    print(nodeMatrix.getNode(x, y))
        return (goal.getPath(), reached)


def drawMap(nodeMatrix: NodeMatrix, goalPath, reached: list[Node]):
    nodeMap = nodeMatrix.getMatrix()
    # Settings
    fps = 60
    fpsClock = pygame.time.Clock()

    length = 20*len(nodeMap[0])
    width = 20*len(nodeMap)
    grid_node_width = 15
    grid_node_height = 15
    gridDisplay = pygame.display.set_mode((length, width))

    # Settings End

    def createSquare(x, y, color):
        pygame.draw.rect(gridDisplay, color,
                         (x, y, grid_node_width, grid_node_height))

    # Draw Map
    for y in range(len(nodeMap)):
        for x in range(len(nodeMap[0])):
            node = nodeMatrix.getNode(x, y)
            if node.getWeight() == math.inf:
                createSquare(x*grid_node_width, y *
                             grid_node_height, (100, 0, 0))
            elif node.getWeight() == 1:
                createSquare(x*grid_node_width, y *
                             grid_node_height, (225, 225, 225))

    createSquare(nodeMatrix.getStart().x*grid_node_width,
                 nodeMatrix.getStart().y*grid_node_height, (100, 100, 100))
    createSquare(nodeMatrix.getGoal().x*grid_node_width,
                 nodeMatrix.getGoal().y*grid_node_height, (0, 255, 0))

    pygame.display.update()

    # for node in reached:
    # createSquare(node.x*grid_node_width, node.y*grid_node_height, (0, 0, np.clip(int(node.getHeuristicCost()*20),0,255)))

    pygame.font.init()
    white = (255, 255, 255)
    green = (0, 255, 0)
    blue = (0, 0, 128)
    for node in reached:
        x = node.x*grid_node_width
        y = node.y*grid_node_height
        font = pygame.font.SysFont('timesnewroman',  8)
        text = font.render(str(node.getPathCost()), True, (0, 0, 0), None)
        textRect = text.get_rect()
        textRect.x = x + grid_node_width
        textRect.y = y + grid_node_height
        textRect.center = (x + grid_node_width / 2, y + grid_node_height / 2)
        gridDisplay.blit(text, textRect)

    pygame.display.update()

    # for node in reached:
    # pygame.event.get()
    # fpsClock.tick(fps)


if __name__ == "__main__":
    nodeMatrix = NodeMatrix(map)

    goalPath, explored = AStarSearch(NodeMatrix(Map_Obj(1)))
    drawMap(nodeMatrix, goalPath, explored)
    time.sleep(10)

    print(goalPath)

    nodeMap = nodeMatrix.getMatrix()

    # Print map
    # for y in range(len(nodeMap)):
    #     for x in range(len(nodeMap[0])):
    #         node = nodeMatrix.getNode(x,y)
    #         if node.getWeight() == math.inf:
    #             print("#", end="")
    #         else:
    #             print("O", end="")
    #     print("")
