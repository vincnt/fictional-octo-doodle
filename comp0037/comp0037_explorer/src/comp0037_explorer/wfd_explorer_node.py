import rospy
from collections import defaultdict

from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class WfdExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []

    def getAdjacentCells(self,cell):
        cells = []
        max_width = self.occupancyGrid.getWidthInCells()
        max_height = self.occupancyGrid.getHeightInCells()
        for adj in [(1,1),(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1)]:
            new_x = cell[0] + adj[0]
            new_y = cell[1] + adj[1]
            if new_x < max_width and new_x >= 0 \
                and new_y < max_height and new_y >= 0 :
                cells.append((new_x,new_y))
        return cells

    def updateFrontiers(self):
        # Based off the pseudocode in the WFD paper

        og_classification = defaultdict(str)
        map_q = []
        start_pose = (0,0)
        map_q.append(start_pose)
        og_classification[start_pose] = 'MAP-OPEN-LIST'
        new_frontiers = []

        while len(map_q) > 0:
            p = map_q.pop(0)

            if og_classification[p] == "MAP-CLOSE-LIST":
                continue

            if self.isFrontierCell(p[0],p[1]):
                frontier_q = []
                new_frontier = []
                frontier_q.append(p)
                og_classification[p] = 'FRONTIER-OPEN-LIST'

                while len(frontier_q) > 0:
                    q = frontier_q.pop(0)
                    if og_classification[q] in ['MAP-CLOSE-LIST','FRONTIER-CLOSE-LIST']:
                        continue
                    if self.isFrontierCell(q[0],q[1]):
                        new_frontier.append(q)
                        for w in self.getAdjacentCells(q):
                            if og_classification[w] not in ['FRONTIER-OPEN-LIST','FRONTIER-CLOSE-LIST','MAP-CLOSE-LIST']:
                                frontier_q.append(w)
                                og_classification[w] = 'FRONTIER-OPEN-LIST'
                    og_classification[q] = 'FRONTIER-CLOSE-LIST'
                new_frontiers.append(new_frontier)
                for x in new_frontier:
                    og_classification[x] = 'MAP-CLOSE-LIST'
            
            for v in self.getAdjacentCells(p):
                if og_classification[v] not in ['MAP-OPEN-LIST','MAP-CLOSE-LIST']:
                    v_adjacent = self.getAdjacentCells(v)
                    v_labels = [og_classification[z] for z in v_adjacent]
                    if 'MAP-OPEN-LIST' in v_labels:
                        map_q.append(v)
                        og_classification[v] = 'MAP-OPEN-LIST'
            og_classification[p] = 'MAP-CLOSE-LIST'

        self.frontiers = new_frontiers

        # If no more frontiers, returning False is supposed to end the program. 
        if len(new_frontiers) == 0:
            return False

        return True

    def chooseNewDestination(self):
        candidateGood = False

        # Search for the largest frontier
        target_frontier = []
        for frontier in self.frontiers:
            if len(frontier) > len(target_frontier):
                target_frontier = frontier
        print('target frontier', target_frontier)

        # Pick a 'random' eligible cell in the target frontier 
        for possible_destination in target_frontier:
            if possible_destination not in self.blackList:
                candidateGood = True 
                destination = possible_destination
                break
        
        if len(target_frontier) == 0:
            destination = None

        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
            self.blackList.append(goal)
