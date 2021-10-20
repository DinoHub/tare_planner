# Core concepts

## Rolling Grid
The Rolling Grid represents the cuboid of local planning horizon around the robot. Each cell contains an array index value. The main method to call is Roll(), which updates the Grid given a shift of the cuboid due to robot movement.  
The cell positions is expressed in 1 of 3 ways:  
&nbsp; 1. Sub: 3D coordinates of the point. e.g. [20,20,1]  
&nbsp; 2. x,y,z: Same as 1. but split up  
&nbsp; 3. index: Each cell is given a unique index by y*x_max + x. e.g. [20,20,1] win a 40-sized grid will be index 820.
The vector array_ind_to_ind_[] does the opposite of GetCellValue(index), where the input is the value and the stored value is the index.
grid0_ and grid1_ are used as the grid before/after rolling. which_grid_ is a flag which toggles them as previous/current grid
Created as grid_ in viewpoint_manager.cpp.

## Grid
Very similar to Rolling Grid, contains bottom level functions to the grid, such as converting between ind and sub, and getting cell values.  
The vector cells_[] contain the cell values.  
Created as grid0_ and grid1_ in rolling_grid.cpp, and collision_grid_ in viewpoint_manager.cpp
