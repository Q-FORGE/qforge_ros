#!/usr/bin/env python

# requires pip install pyastar2d, numpy

# initialization example
## --------------------------------------- ##
# zone2_wi_x = 25
# zone2_wi_y = 14

# sparsity = 0.1

# tlhc_world_x = 12.5
# tlhc_world_y = 7.5

# battleShip = BattleGrid(zone2_wi_x, zone2_wi_y, tlhc_world_x, tlhc_world_y, sparsity)
## --------------------------------------- ##

# adding obstical points
## --------------------------------------- ##

# battleShip.add_world_to_grid(-1,2)

# running A*
## --------------------------------------- ##
# battleShip.getWaypoint(-10,0,4,0) start x-y, then goal x-y

import numpy as np
import pyastar2d 


class BattleGrid:

    def __init__(self, x_length_m, y_length_m, x_topLeftworld_m, y_topLeftworld_m, sparsity_m):
        ### ---- ###
        #   x_length_m: lenght of zone 2 in x
        #   y_length_m: length of zone 2 in y
        #   x_topLeftworld_m: x position, reletive to world world, of the top left corner of zone 2
        #   y_topLeftworld_m: y position, reletive to world world, of the top left corner of zone 2
        #   sparsity_m:
        ### ---- ###
        # if(x_length_m % sparsity_m > 0 or y_length_m % sparsity_m > 0 or sparsity_m == 0):
        #     raise ValueError('Please find a spacing is divides both x_length_m and y_length_m without a remainder')

        self.hit_miss_grid = np.zeros(( int(x_length_m / sparsity_m) , int(y_length_m / sparsity_m) ), dtype=np.float32) # Grid marking direct hits from pointcloud
        self.config_space = np.ones(( int(x_length_m / sparsity_m) , int(y_length_m / sparsity_m) ), dtype=np.float32) # Grid marking safe passage routes
        self.config_space_view = np.ones(( int(x_length_m / sparsity_m) , int(y_length_m / sparsity_m) ), dtype=np.float32) # Grid marking safe passage routes

        self.x_length_m = x_length_m
        self.y_length_m = y_length_m

        self.x_topLeftworld_m = x_topLeftworld_m
        self.y_topLeftworld_m = y_topLeftworld_m

        self.sparsity_m = sparsity_m

        self.safe_rad_m = 0.85
        self.safe_rad_cells = int(self.safe_rad_m/self.sparsity_m)

        self.grid_size_x = int(x_length_m / sparsity_m)
        self.grid_size_y = int(y_length_m / sparsity_m)

        self.target_world_x = 1 #m initial x target
        self.target_world_y = 0 #m initial y target

        self.current_pos_x = -8 #m current UAV x position
        self.current_pos_y = 0  #m current UAV y position

        self.target_world_x_cell = 0 #m initial x target
        self.target_world_y_cell = 0 #m initial y target

        self.current_pos_x_cell = 0 #m current UAV x position
        self.current_pos_y_cell = 0  #m current UAV y position

        self.refPath_world = []



    def world_to_gridLoc(self, world_x, world_y, allowOutsideGrid=False): # go from a location in world to a grid index
        x_grid_left_cont = -(world_x - self.x_topLeftworld_m)
        y_grid_left_cont = -(world_y - self.y_topLeftworld_m)

        x_grid_left_disc = int(x_grid_left_cont/self.sparsity_m)
        y_grid_left_disc = int(y_grid_left_cont/self.sparsity_m)

        if (x_grid_left_disc < self.grid_size_x and y_grid_left_disc < self.grid_size_y) or allowOutsideGrid:
            return [x_grid_left_disc, y_grid_left_disc]
        else:
            return 'nan'

    def gridLoc_to_world(self, grid_i, grid_j): # go from a location in world to a grid index

        grid_i_cont = grid_i + 0.5
        grid_j_cont = grid_j + 0.5   

        x_world_m = -grid_i_cont*self.sparsity_m + self.x_topLeftworld_m
        y_world_m = grid_j_cont*self.sparsity_m - self.y_topLeftworld_m   

        return [x_world_m, -y_world_m]

    def circle_fill(self,array,i,j,radius,value):
        for ip in range(i-radius, i+radius+1):
            # print(ip)
            for jp in range(j-radius, j+radius+1):
                inArrayCheck = ip < array.shape[0] and jp < array.shape[1] and jp >= 0 and ip >= 0
                if inArrayCheck:
                    ifPopCheck = array[ip,jp] > 0
                    if ifPopCheck:
                        inRadCheck = (ip - i)**2 + (jp - j)**2 <= radius**2
                        # print("ip: " + str(ip) + " jp: " + str(jp) + " in rad: " + str(np.sqrt((ip - radius)**2 + (jp - radius)**2)))
                        if inRadCheck and inArrayCheck:
                            array[ip,jp] = array[ip,jp] + value
        return array

    def add_world_to_grid(self,world_x_m, world_y_m):

        if world_x_m > -8 and world_x_m < 1:
            grid_loc = self.world_to_gridLoc(world_x_m,world_y_m)
        else:
            grid_loc = 'nan'
        
        if(grid_loc != 'nan'):
            # if(self.hit_miss_grid[grid_loc[0], grid_loc[1]] == 0 and grid_loc[0]):
            if True:
                self.hit_miss_grid[grid_loc[0], grid_loc[1]] = self.hit_miss_grid[grid_loc[0], grid_loc[1]] + 1
                self.config_space = self.circle_fill(self.config_space, grid_loc[0], grid_loc[1], self.safe_rad_cells,  3000)
                self.config_space_view = self.circle_fill(self.config_space_view, grid_loc[0], grid_loc[1], self.safe_rad_cells,  0)

    def getWaypoint(self, uav_x, uav_y, target_x, target_y):
        self.target_world_x = target_x
        self.target_world_y = target_y
        
        self.current_pos_x = uav_x
        self.current_pos_y = uav_y

        target_cell = self.world_to_gridLoc(self.target_world_x, self.target_world_y, True)
        UAV_cell = self.world_to_gridLoc(self.current_pos_x, self.current_pos_y, True)
        
        self.target_world_x_cell = target_cell[0]
        self.target_world_y_cell = target_cell[1]

        self.current_pos_x_cell = UAV_cell[0]
        self.current_pos_y_cell = UAV_cell[1]

        path = pyastar2d.astar_path(self.config_space, (self.current_pos_x_cell, self.current_pos_y_cell), (self.target_world_x_cell, self.target_world_y_cell), allow_diagonal=False)
        if path is not None:
            # path = path[6:-1]
            # path = path[10:12]
            self.refPath_world = []
            for i in range(0,len(path)):
                self.refPath_world.append(self.gridLoc_to_world(path[i,0], path[i,1]))
                # self.config_space_view = np.copy(self.config_space)
                # self.config_space_view[self.config_space_view == np.inf] = 0
                self.config_space_view[path[i,0], path[i,1]] = 0.5 # REMOVE!!
            return path