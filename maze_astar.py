import numpy as np
from heapq import heappush, heappop
from animation import draw
import argparse

class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class Maze():
    
    def __init__(self, map, start_state, goal_state, map_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.map = map
        self.visited = [] # state
        self.m, self.n = map.shape 
        self.map_index = map_index
        #print("star state:",start_state)
        #print("goal_state:", goal_state)

    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
    
        draw(self.map, path[::-1], self.map_index)


    def goal_test(self, current_state):
        if np.array_equal(current_state,self.goal_state):
            return True
        else:
            return False


    def get_cost(self, current_state, next_state):
        # your code goes here:
        return 1


    def get_successors(self, state):
        # your code goes here:
        successors = []

        # position of the start state
        rowCurrent,columnCurrent = state[0], state[1]
        colDirections = [-1,1,0,0]
        rowDirections = [0,0,1,-1]
        #print("state:",state)
        #print(self.map)
        for k in range(4):
            newRow = rowCurrent + rowDirections[k]
            newCol = columnCurrent + colDirections[k]
            #print("newRow, newCol:",newRow, newCol)

            if self.map[newRow][newCol] == 1.0:
        
                newState = (newRow,newCol)
                #print("newstate:",newState)
                successors.append(newState)
                    
            
        return successors




        


    # heuristics function
    def heuristics(self, state):
        # your code goes here:
        cost = 0
        cost += abs(state[0]- self.goal_state[0])
        cost += abs(state[1] - self.goal_state[1])
        #print("Cost:",cost)
        return cost


    # priority of node 
    def priority(self, node):
        # your code goes here:
        return self.heuristics(node.state)+node.cost_from_start

    
    # solve it
    def solve(self):
        # your code goes here:
        container = [] # node
        count = 1
        state = self.start_state
        node = Node(state, 0, None)
        self.visited.append(state)

        if self.goal_test(state):
            return state
        
        heappush(container, (self.priority(node), count, node))
        while container:
            currentNode = heappop(container)[2]

            successor = self.get_successors(currentNode.state)

            for next_state in successor:
                beenVistied = False
                for visited in self.visited:
                    if np.array_equal(next_state, visited):
                        beenVistied = True
                if not beenVistied:
                    self.visited.append(next_state)
                    next_cost = currentNode.cost_from_start + self.get_cost(currentNode.state, next_state)
                    next_node = Node(next_state, next_cost, currentNode)
 
                    if self.goal_test(next_state) is True:
                        self.draw(next_node)
                        return 

                    count+=1
                    heappush(container, (self.priority(next_node), count, next_node))


        

            
    
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='maze')
    parser.add_argument('-index', dest='index', required = True, type = int)
    index = parser.parse_args().index

    # Example:
    # Run this in the terminal solving map 1
    #     python maze_astar.py -index 1
    
    data = np.load('map_'+str(index)+'.npz')
    map, start_state, goal_state = data['map'], tuple(data['start']), tuple(data['goal'])

    game = Maze(map, start_state, goal_state, index)
    game.solve()
    