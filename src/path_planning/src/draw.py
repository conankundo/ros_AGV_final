# ros library
import rospy
from std_msgs.msg import String
from pynput import keyboard
key_pressed = False
# thung ngoc dan library
import pygame
import numpy as np
import math
import random
import csv                      #map.py
import os
from heapq import *           # PathPlanning.py
from collections import deque
# from robot import *
# from map import *
# from pathPlanning import *
import time

class DRAW:
    def __init__(self, map_matrix):
        self.tile_size = 30
        self.length_x = 20
        self.length_y = 20
        self.width = self.tile_size * self.length_x
        self.height = self.tile_size * self.length_y
        pygame.init() 
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Path Planning")
        self.map_matrix = map_matrix
        self.centers = self.getPosition()
        self.pos_posible = []

    def get_rect(self, x, y):
        return x * self.tile_size, y * self.tile_size, self.tile_size, self.tile_size
    
    def getCoordinate(self, index):
        rows, cols = len(self.map_matrix), len(self.map_matrix[0])
        row = index // cols
        col = index % cols  
        return row, col
    
    def getPosition(self):
        centers = []
        for i in range(len(self.map_matrix)):
            for j in range(len(self.map_matrix[0])):
                rect1 = pygame.Rect(self.get_rect(j, i))
                centers.append(rect1.center)  
        return centers

    def draw_robot(self, robot):
        pygame.draw.circle(self.screen, pygame.Color("chartreuse3"), robot.current_pos, self.tile_size / 3, 10)

    def draw_target(self, target_pos): 
        pygame.draw.circle(self.screen, pygame.Color("red"), target_pos, self.tile_size / 5, 10)

    def draw_map(self):
        rows, cols = len(self.map_matrix), len(self.map_matrix[0])
        for row in range(rows):
            for col in range(cols):
                if self.map_matrix[row][col] == 1:
                    pygame.draw.rect(self.screen, pygame.Color("white"), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))
                    self.pos_posible.append(self.get_index(row, col))
                elif self.map_matrix[row][col] == 2:
                    pygame.draw.rect(self.screen, (255, 0, 0), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))
                elif self.map_matrix[row][col] == 3:
                    pygame.draw.rect(self.screen, (89, 100, 150), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))
                elif self.map_matrix[row][col] == 4:
                    pygame.draw.rect(self.screen, (79, 100, 150), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))
                elif self.map_matrix[row][col] == 5:
                    pygame.draw.rect(self.screen, (89, 140, 40), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))
                elif self.map_matrix[row][col] == 6:
                    pygame.draw.rect(self.screen, (13, 100, 150), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))
                elif self.map_matrix[row][col] == 7:
                    pygame.draw.rect(self.screen, (78, 40, 150), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))        
                elif self.map_matrix[row][col] == 8:
                    pygame.draw.rect(self.screen, (0, 30, 50), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))
                elif self.map_matrix[row][col] == 9:
                    pygame.draw.rect(self.screen, pygame.Color("red"), (col * self.tile_size, row * self.tile_size, self.tile_size, self.tile_size))

    def drawMove(self, path):
        for point in path:
            pygame.draw.circle(self.screen, pygame.Color('blue'), self.centers[point], 5)
        if len(path) > 0:
            pygame.draw.circle(self.screen, pygame.Color('darkorange'), self.centers[path[len(path) - 1]], 5)

    def get_mouse_pos(self):
        mouse_pos = pygame.mouse.get_pos()
        row = mouse_pos[1] // self.tile_size
        col = mouse_pos[0] // self.tile_size
        return self.get_index(row, col)

    def get_robot_pos(self, pos):
        row = pos[1] // self.tile_size
        col = pos[0] // self.tile_size
        return self.get_index(row, col)
    
    def get_index(self, row, col):
        return row * self.length_x + col
    
    def plot(self):
        WHITE = (255, 255, 255)
        # init_points = [157, 117, 124, 131, 98, 41]
        init_points = [157]
        target_points = [36, 31, 149, 71, 378, 364]
        init_poses = [self.centers[point] for point in init_points]
        robots = [ROBOT(init_pos) for init_pos in init_poses]
        paths = [astar.Astar(init_point, target_point) for init_point, target_point in zip(init_points, target_points)]
        
        for i in range(len(robots)):
            robots[i].path = paths[i]
            robots[i].centers = self.centers

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
            self.screen.fill(WHITE)
            self.draw_map()
            
            for robot in robots:
                if robot.path == []:
                    targett = self.pos_posible[random.randint(0, len(self.pos_posible) - 1)]
                    robot.path = astar.Astar(self.get_robot_pos(robot.current_pos), targett)
                    print(robot.path)
                    pub.publish(' '.join([str(_) for _ in robot.path]))
                    rate.sleep()
                for other_robot in robots:
                    if robot != other_robot:
                        while np.sqrt((robot.current_pos[0] - other_robot.current_pos[0]) ** 2 + (robot.current_pos[1] - other_robot.current_pos[1]) ** 2) < 30:   
                            robot.velocity = [random.randint(-1, 1), random.randint(-1, 1)]

                robot.followPath(robot.path)
                self.draw_robot(robot)
                self.drawMove(robot.path)
            
            pygame.display.flip()
            self.clock.tick(30)
            
        pygame.quit()

    def draw_finding_path(self, queue):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
            self.screen.fill((255, 255, 255))
            for _, xy in queue:
                col = self.getCoordinate(xy)[1]
                row = self.getCoordinate(xy)[0]
                pygame.draw.rect(self.screen, pygame.Color('darkslategray'), self.get_rect(col, row), 3)
            pygame.display.flip()
            self.clock.tick(7)
class Algorithm:
    def __init__(self, adjacency_list,map_matrix):
        self.adjacency_list = adjacency_list
        self.map_matrix = map_matrix
        

    def getCoordinate(self, index):
        rows, cols = len(self.map_matrix), len(self.map_matrix[0])
        row = index // cols
        col = index % cols  
        return row, col
    
    def heuristic_manhattan(self, node, end_node):
        # Hàm heuristic Mahattan cho A*
        x1, y1 = self.getCoordinate(node)
        x2, y2 = self.getCoordinate(end_node)
        return abs(x1 - x2) + abs(y1 - y2)

    def Astar(self, start, end):
        inf = float('inf')
        # draw = DRAW(self.map_matrix)
        # Khởi tạo mảng khoảng cách và đỉnh cha
        dist = {vertex: inf for vertex in self.adjacency_list}
        parent = {vertex: None for vertex in self.adjacency_list}

        # Đỉnh xuất phát có khoảng cách là 0
        dist[start] = 0

        # Hàng đợi ưu tiên để lựa chọn đỉnh có khoảng cách ngắn nhất
        priority_queue = [(0, start)]

        while priority_queue:
            # draw.draw_finding_path(priority_queue)
            # print(priority_queue)

            # Lấy đỉnh u có khoảng cách ngắn nhất
            cur_dist, u = heappop(priority_queue)

            # Duyệt qua các đỉnh kề của u
            for v, edge_weight in self.adjacency_list[u]:
                new_dist = cur_dist + edge_weight

                # Cập nhật khoảng cách và đỉnh cha nếu có đường đi ngắn hơn
                if new_dist < dist[v]:
                    dist[v] = new_dist
                    parent[v] = u
                    # print(parent[v])
                    # Sử dụng hàm heuristic Mahattan để tối ưu hóa A*
                    priority = new_dist + 0.1*self.heuristic_manhattan(v, end)
                    heappush(priority_queue, (priority, v))
                    # heappush(priority_queue, (new_dist, v))

        # Truy vết đường đi ngắn nhất từ end về start
        path = []
        # print(dist[end])
        cost = dist[end]
        cur_node = end
        while cur_node is not None:
            path.insert(0, cur_node)
            cur_node = parent[cur_node]
            # print(parent) 

        return path
class GRAPH:
    def __init__(self, filename):
        self.filename = filename
        self.map_matrix = self.read_csv(filename)
        self.adj_list = self.create_weighted_adj_list()
        
        
    def create_weighted_adj_list(self):
        rows, cols = len(self.map_matrix), len(self.map_matrix[0])
        adj_list = {}

        def index(row, col):
            return row * cols + col

        for row in range(rows):
            for col in range(cols):
                # weight = self.map_matrix[row][col]
                current_index = index(row, col)
                adj_list[current_index] = []
                if row > 0 and self.map_matrix[row - 1][col] < 9:
                    adj_list[current_index].append((index(row - 1, col), self.map_matrix[row-1][col]))  # Weight is 1 for adjacent nodes
                if row < rows - 1 and self.map_matrix[row + 1][col] < 9:
                    adj_list[current_index].append((index(row + 1, col), self.map_matrix[row+1][col]))
                if col > 0 and self.map_matrix[row][col - 1] < 9:
                    adj_list[current_index].append((index(row, col - 1), self.map_matrix[row][col-1]))
                if col < cols - 1 and self.map_matrix[row][col + 1] < 9:
                    adj_list[current_index].append((index(row, col + 1), self.map_matrix[row][col+1]))

                # if weight != 0:
                #     if row > 0 and self.map_matrix[row - 1][col] == 0:
                #         adj_list[current_index].append((index(row - 1, col), 1))  # Weight is 1 for adjacent nodes
                #     if row < rows - 1 and self.map_matrix[row + 1][col] == 0:
                #         adj_list[current_index].append((index(row + 1, col), 1))
                #     if col > 0 and self.map_matrix[row][col - 1] == 0:
                #         adj_list[current_index].append((index(row, col - 1), 1))
                #     if col < cols - 1 and self.map_matrix[row][col + 1] == 0:
                #         adj_list[current_index].append((index(row, col + 1), 1))
                # if weight == 2:
                #     if row > 0 and self.map_matrix[row - 1][col] == 2:
                #         adj_list[current_index].append((index(row - 1, col), 2))  # Weight is 1 for adjacent nodes
                #     if row < rows - 1 and self.map_matrix[row + 1][col] == 2:
                #         adj_list[current_index].append((index(row + 1, col), 2))
                #     if col > 0 and self.map_matrix[row][col - 1] == 2:
                #         adj_list[current_index].append((index(row, col - 1), 2))
                #     if col < cols - 1 and self.map_matrix[row][col + 1] == 2:
                #         adj_list[current_index].append((index(row, col + 1), 2))
                #     if row > 0 and self.map_matrix[row - 1][col] == 0:
                #         adj_list[current_index].append((index(row - 1, col), 1))  # Weight is 1 for adjacent nodes
                #     if row < rows - 1 and self.map_matrix[row + 1][col] == 0:
                #         adj_list[current_index].append((index(row + 1, col), 1))
                #     if col > 0 and self.map_matrix[row][col - 1] == 0:
                #         adj_list[current_index].append((index(row, col - 1), 1))
                #     if col < cols - 1 and self.map_matrix[row][col + 1] == 0:
                #         adj_list[current_index].append((index(row, col + 1), 1))
        # print(adj_list)
        return adj_list
    
    def read_csv(self,filename):
        map_data = []
        # with open(os.path.join(filename)) as data:
        with open(filename) as data:
            data = csv.reader(data, delimiter=",")
            for row in data:
                result_list = [int(item) if item != '' else 0 for item in row]
                # print(result_list)
                map_data.append(list(result_list))
        # print(map_data)
        return map_data
class ROBOT:
    def __init__(self,initial_pos):    
        self.target_pos = np.array([0,0])
        self.current_pos = initial_pos
        self.velocity = np.zeros(2).astype(int)
        self.color = (200, 0, 0)
        self.trace = []
        self.path = []
        self.centers = []
        self.prev_goal = 0
        # self.map = map
    
    def updatePose(self):
        if np.linalg.norm(self.current_pos - self.target_pos) < 0.01:
            self.velocity = np.zeros(2)
        else:
            self.current_pos = self.current_pos + self.velocity
            self.trace.append(self.current_pos)

    def movetoGoal(self,goal, pre_goal):
        if (goal - pre_goal) == 1:
            self.velocity = np.array([1,0])
        elif (goal - pre_goal) == -1:
            self.velocity = np.array([-1,0])
        elif (goal - pre_goal) == 20:
            self.velocity = np.array([0,1])
        elif (goal - pre_goal) == -20:
            self.velocity = np.array([0,-1])
        # else:
        #     self.velocity = np.array([0,-1])
    
    def followPath(self, path):
        if path != []:
            self.movetoGoal(path[0], self.prev_goal)
            # self.movetoGoal(path[1], path[0] )
            self.target_pos = np.array(self.centers[path[0]])
            self.updatePose()
            # print(np.linalg.norm(self.current_pos - np.array(self.centers[path[0]])))
            if np.linalg.norm(self.current_pos - np.array(self.centers[path[0]])) < 1.5:
                self.prev_goal = path[0]
                path.pop(0)
                # print(path[0], init_point)
            if path == []:
                # break
                pass
        return self.current_pos
    
    def check_range(self, robots):
        for robot in robots:
            if robot != self:
                print(np.linalg.norm(np.array(self.current_pos) - np.array(robot.current_pos)))
                if np.linalg.norm(np.array(self.current_pos) - np.array(robot.current_pos)) < 250:
                    if np.linalg.norm(np.array(self.target_pos) - np.array(self.current_pos)) < np.linalg.norm(np.array(robot.target_pos) - np.array(robot.current_pos)):
                        self.velocity = np.zeros(2)
     
def csv_to_pgm(csv_filename, pgm_filename, scale =10):
    map_matrix = np.loadtxt(csv_filename, delimiter=',', dtype=int)
    scaled_matrix = np.repeat(np.repeat(map_matrix, scale, axis=0), scale, axis=1)
    with open(pgm_filename, 'w') as f:
        f.write("P2\n")
        f.write(f"{scaled_matrix.shape[1]} {scaled_matrix.shape[0]}\n")
        f.write("255\n")
        for row in scaled_matrix:
            for val in row:
                f.write(f"{255 if val == 1 else 0} ")
            f.write("\n")
#########################################################################
# Khởi tạo node 'talker'
rospy.init_node('talker', anonymous=True)

# Tạo publisher, publish thông điệp String trên topic 'chatter'
pub = rospy.Publisher('chatter', String, queue_size=10)

# Thiết lập tần suất publish là 1Hz
rate = rospy.Rate(1)



if __name__ == "__main__":
    map = GRAPH('map.csv')                                                                  # file csv nam trong folder catkin_ws
    csv_to_pgm("map.csv","/home/dac/catkin_ws/src/stage_controller/world/map.pgm", scale=30)
    astar = Algorithm(map.adj_list, map.map_matrix)
    draw = DRAW(map.map_matrix)
    draw.plot()
    
    