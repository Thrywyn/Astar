import csv
import random
from mazelib import Maze
from mazelib.generate.Prims import Prims
from mazelib.generate.CellularAutomaton import CellularAutomaton
from mazelib.generate.DungeonRooms import DungeonRooms
import copy
import numpy as np
from mazelib.solve.BacktrackingSolver import BacktrackingSolver
from mazelib.solve.Collision import Collision
import matplotlib.pyplot as plt
import matplotlib
import subprocess
import os
import platform


def main():
    m = Maze()
    m.generator = Prims(100, 100)
    m.generate()
    m.start = (1, 1)
    m.end = (199, 199)

    mazeCopy = m.grid.copy()

    # Convert to format that Map.py can use
    for y in range(len(mazeCopy)):
        for x in range(len(mazeCopy[0])):
            if mazeCopy[y][x] == 1:
                mazeCopy[y][x] = -1
            elif mazeCopy[y][x] == 0:
                mazeCopy[y][x] = 1

    # Randomize maze walls to potentially create more paths
    for y in range(len(mazeCopy)):
        for x in range(len(mazeCopy[0])):
            if mazeCopy[y][x] == -1:
                mazeCopy[y][x] = random.choices(
                    [1, -1], weights=[1, 10], k=1)[0]

    randfile = open("Random.csv", "w")
    writer = csv.writer(randfile, delimiter=",")

    for y in range(len(mazeCopy[0])):
        writer.writerow(mazeCopy[y].tolist())

    randfile.close()

    m.solver = Collision()
    m.solve()

    # Find shortest solution
    shortest = 0
    for i in range(len(m.solutions)):
        if len(m.solutions[i]) < len(m.solutions[shortest]):
            shortest = i

    # For shortest solution, update m with the solution path
    for y, x in m.solutions[shortest]:
        m.grid[y][x] = 2

    # Create colormap that maps 0 to white, 1 to black, and 2 to green
    cmap = matplotlib.colors.ListedColormap(["white", "black", "green"])
    bounds = [0, 0.5, 1.5, 2]
    norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)

    # Plot the maze with the solution path
    plt.figure(figsize=(10, 10))
    plt.imshow(m.grid, cmap=cmap, interpolation='nearest', norm=norm)
    plt.xticks([]), plt.yticks([])
    # plt.show()
    plt.savefig("MazeSolution.png")
    print("Saved figure to MazeSolution.png")
    os.startfile("C:\Repos\Astar\MazeSolution.png")


if __name__ == "__main__":
    main()
