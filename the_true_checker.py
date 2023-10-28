from itertools import product
from ortools.linear_solver import pywraplp

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"Point({self.x}, {self.y}, {self.z})"


def print_values(array, array_name):
    print(f"Values for {array_name}:")
    for i, var in enumerate(array):
        print(f"  {var.name()}: {var.solution_value()}")

def Path(A, B, Ix, Dx, Iy, Dy, Iz, Dz):
    res = 0
    
    for j in range(min(A.x, B.x), max(A.x, B.x)):
        #print("j", j)
        if A.x < B.x: 
            res += Ix[j]
        else:
            res += Dx[j]
    

    for j in range(min(A.y, B.y), max(A.y, B.y)):
        #print("j", j)
        if A.y < B.y: 
            res += Iy[j]
        else:
            res += Dy[j]

    for j in range(min(A.z, B.z), max(A.z, B.z)):
        if A.z < B.z:
            res += Iz[j]
        else:
            res += Dz[j]

    return res

def Path_Dis(A, B, Ix, Dx, Iy, Dy, Iz, Dz):
    res = 0
    
    for j in range(min(A.x, B.x), max(A.x, B.x)):
        #print("j", j)
        if A.x < B.x: 
            res += Ix[j].solution_value()
        else:
            res += Dx[j].solution_value()
    

    for j in range(min(A.y, B.y), max(A.y, B.y)):
        #print("j", j)
        if A.y < B.y: 
            res += Iy[j].solution_value()
        else:
            res += Dy[j].solution_value()

    for j in range(min(A.z, B.z), max(A.z, B.z)):
        if A.z < B.z:
            res += Iz[j].solution_value()
        else:
            res += Dz[j].solution_value()

    return res

def isConfigurationPossible(nodes, nodeDests, Rx, Ry, Rz):
    # Instantiate a Glop solver, naming it LinearExample.
    solver = pywraplp.Solver.CreateSolver("GLOP")
    if not solver:
        return

    # Creating Ix and Dx based on Rx
    Ix = [solver.NumVar(1, solver.infinity(), f"Ix{i}") for i in range(Rx)]
    Dx = [solver.NumVar(1, solver.infinity(), f"Dx{i}") for i in range(Rx)]

    # Creating Iy and Dy based on Ry
    Iy = [solver.NumVar(1, solver.infinity(), f"Iy{i}") for i in range(Ry)]
    Dy = [solver.NumVar(1, solver.infinity(), f"Dy{i}") for i in range(Ry)]

    # Creating Iz and Dz based on Rz
    Iz = [solver.NumVar(1, solver.infinity(), f"Iz{i}") for i in range(Rz)]
    Dz = [solver.NumVar(1, solver.infinity(), f"Dz{i}") for i in range(Rz)]

    for i in range(len(nodeDests)):
        #print(nodes[i].x, nodes[i].y)
        Left = pywraplp.LinearExpr()
        Left = Path(nodes[i], nodeDests[i], Ix, Dx, Iy, Dy, Iz, Dz)
        #print(Left) 
        tmpset = {(nodeDests[i].x, nodeDests[i].y, nodeDests[i].z)}
        for j in range(len(nodeDests)):
            if (nodeDests[j].x, nodeDests[j].y, nodeDests[j].z) not in tmpset:
                tmpset.add((nodeDests[j].x, nodeDests[j].y, nodeDests[j].z))
                Right = Path(nodes[i], nodeDests[j], Ix, Dx, Iy, Dy, Iz, Dz)
                #print(nodes[i].x, nodes[i].y, nodeDests[j].x, nodeDests[j].y)
                #print("Right", Right)
                solver.Add(Left + 0.1 <= Right)
    if Rx > 0:
        solver.Minimize(Ix[0])
    elif Ry > 0:
        solver.Minimize(Iy[0])
    else:
        solver.Minimize(Iz[0])

    # Solve the system.
    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL:
        print("Solution:")
        #print("Objective value =", solver.Objective().Value())
        # Printing values for each array
        print_values(Ix, 'Ix')
        print_values(Dx, 'Dx')
        print_values(Iy, 'Iy')
        print_values(Dy, 'Dy')
        print_values(Iz, 'Iz')
        print_values(Dz, 'Dz')
        for i in range(len(nodeDests)):
            print(nodes[i], "->", nodeDests[i], " Distance:", Path_Dis(nodes[i], nodeDests[i], Ix, Dx, Iy, Dy, Iz, Dz))
        print("------------")
        return True
    else:
        return False


def read_3d_array(filename):
    with open(filename, 'r') as file:
        Rx, Ry, Rz = map(int, file.readline().split())

        # Reading the rest of the file and ignoring whitespaces and newline characters
        data = file.read().split()

        # Creating and populating the 3D array
        c = [[[data[i * Ry * Rz + j * Rz + k] for k in range(Rz)] for j in range(Ry)] for i in range(Rx)]

        return c

def group_coordinates(c_values):
    groups = {}
    Rx, Ry, Rz = len(c_values), len(c_values[0]), len(c_values[0][0])
    for i in range(Rx):
        for j in range(Ry):
            for k in range(Rz):
                value = c_values[i][j][k]
                if value not in groups:
                    groups[value] = []
                groups[value].append((i, j, k))
    return list(groups.values())

def enumerate_representatives(groups):
    representatives = product(*groups)
    return representatives

def create_new_lists(representatives, groups, Rx, Ry, Rz):
    for rep in representatives:
        new_list = [None for _ in range(Rx * Ry * Rz)]
        for group, r in zip(groups, rep):
            for coordinate in group:
                i, j, k = coordinate
                new_list[i * Ry * Rz + j * Rz + k] = Point(*r)
        yield new_list

# input:
c_values = read_3d_array('values.txt')
groups = group_coordinates(c_values)
representatives = enumerate_representatives(groups)

Rx, Ry, Rz = len(c_values), len(c_values[0]), len(c_values[0][0])
nodes = [Point(i, j, k) for i in range(Rx) for j in range(Ry) for k in range(Rz)]
tmp = 0

#print(Rx, Ry, Rz)
for new_list in create_new_lists(representatives, groups, Rx, Ry, Rz):
    if isConfigurationPossible(nodes, new_list, Rx - 1, Ry - 1, Rz - 1):
        tmp = 1
        break

if tmp == 1:
    print("The configuration is possible.")
else:
    print("The configuration is not possible.")