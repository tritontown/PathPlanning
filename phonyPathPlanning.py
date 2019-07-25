import rospy
from Queue import Queue
import numpy as np
import pprint
import matplotlib.pyplot as plt

class Point(object):

    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

    def setPt(self, x, y):
        self.x = x
        self.y = y

    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def getX(self):
        return self.x
    
    def getY(self):
        return self.y

class PhonyCar(object):

    def __init__(self):
        
        self.location = Point()
        self.location.setX(0)
        self.location.setY(3)

    #TODO: Change values of array indexes

    def moveU(self):
        if(self.location[1] + 1 > 3):
            print("Too far up! Outside of track boundary.")
        else:
            self.location[1] += 1

    def moveD(self):
        if(self.location[1] - 1 < 0):
            print("Too far down! Outside of track boundary.")
        else:
            self.location[1] -= 1

    def moveR(self):
        if(self.location[0] + 1 > 3):
            print("Too far right! Outside of track boundary.")
        else:
            self.location[0] += 1


    def moveL(self):
        if(self.location[0] - 1 < 0):
            print("Too far left! Outside of track boundary.")
        else:
            self.location[0] -= 1

    def getLoc(self):
        return self.location
        

class Tile(object):

    def __init__(self):
        
        self.up = True
        self.down = True
        self.right = True
        self.left = True

    def printMe(self):
        print("Can go up: " + str(self.up) )
        print("Can go down: " + str(self.down) )
        print("Can go right: " + str(self.right) )
        print("Can go left: " + str(self.left) )

    def setU(self, value):
        self.up = value

    def setD(self, value):
        self.down = value

    def setR(self, value):
        self.right = value

    def setL(self, value):
        self.left = value

    def hasU(self):
        return self.up

    def hasD(self):
        return self.down

    def hasR(self):
        return self.right

    def hasL(self):
        return self.left

class Grid(object):
    
    def __init__(self):
        self.constructGrid()

    def constructGrid(self):

        self.grid = []
        
        for i in range(4):
            tiles = [ Tile(), Tile(), Tile(), Tile() ]
            self.grid.append(tiles)

        
        for i in range(4):
            self.grid[i][0].setL(False)
            self.grid[i][3].setR(False)
            self.grid[0][i].setU(False)
            self.grid[3][i].setD(False)
    
    def printGrid(self):
        
        for i in range(4):
            for j in range(4):
                print('Tile (' + str(i) + ', ' + str(j) + '): ')
                self.grid[j][i].printMe()

    def toggleTile(self, x, y, value):
        
        if(self.grid[x][y].hasD()):
            self.grid[x+1][y].setU(value)
        
        if(self.grid[x][y].hasU()):
	    self.grid[x-1][y].setD(value)
        
        if(self.grid[x][y].hasR()):
		self.grid[x][y+1].setL(value)
        
        if(self.grid[x][y].hasL()):
            self.grid[x][y-1].setR(value)

    def dijkstra(self, start_x, start_y):
        
        nodes = Queue()
        self.parentGrid = [] 
        costGrid = []

        
        for i in range(4):
            self.parentGrid.append( [ Point(), Point(), Point(), Point() ] )
            costGrid.append( [ np.inf, np.inf, np.inf, np.inf ] )

        start = Point()
        start.setPt( start_x, start_y )
            
        costGrid[start_x][start_y] = 0
        
        nodes.put(start)


        while( not nodes.empty() ):

            # Node being evaluated
            this_pt = nodes.get()
            this_cost = costGrid[this_pt.getX()][this_pt.getY()]

            # Check for top neighbor
            if( self.grid[this_pt.getX()][this_pt.getY()].hasU() ):
                 
                # Neighbor coordinates
                x_coord = this_pt.getX() - 1
                y_coord = this_pt.getY()
 

                if( costGrid[x_coord][y_coord] > this_cost + 1 ):

                    # Update parent grid
                    self.parentGrid[x_coord][y_coord].setPt(this_pt.getX(), 
                                                        this_pt.getY())
                
                    costGrid[x_coord][y_coord] = this_cost + 1

                    # Push found neighbor
                    nodes.put( Point(x_coord, y_coord) )

            # Check for right neighbor
            if( self.grid[this_pt.getX()][this_pt.getY()].hasR() ):
                
                # Neighbor coordinates
                x_coord = this_pt.getX()
                y_coord = this_pt.getY() + 1

                if( costGrid[x_coord][y_coord] > this_cost + 1 ):
                
                    # Update parent grid
                    self.parentGrid[x_coord][y_coord].setPt(this_pt.getX(), 
                                                        this_pt.getY())
                
                    costGrid[x_coord][y_coord] = this_cost + 1

                    # Push found neighbor
                    nodes.put( Point(x_coord, y_coord) )

            # Check for down neighbor
            if( self.grid[this_pt.getX()][this_pt.getY()].hasD() ):
                
                # Neighbor coordinates
                x_coord = this_pt.getX() + 1
                y_coord = this_pt.getY()

                if( costGrid[x_coord][y_coord] > this_cost + 1 ):
                
                    # Update parent grid
                    self.parentGrid[x_coord][y_coord].setPt(this_pt.getX(), 
                                                        this_pt.getY())
                
                    costGrid[x_coord][y_coord] = this_cost + 1

                    # Push found neighbor
                    nodes.put( Point(x_coord, y_coord) )

            # Check for left neighbor
            if( self.grid[this_pt.getX()][this_pt.getY()].hasL() ):
                
                # Neighbor coordinates
                x_coord = this_pt.getX()
                y_coord = this_pt.getY() - 1

                if( costGrid[x_coord][y_coord] > this_cost + 1 ):
                    
                    # Update parent grid
                    self.parentGrid[x_coord][y_coord].setPt(this_pt.getX(), 
                                                        this_pt.getY())
                
                    costGrid[x_coord][y_coord] = this_cost + 1

                    # Push found neighbor
                    nodes.put( Point(x_coord, y_coord) )


def visualization(parentGrid):
    width = np.shape(parentGrid)[1]
    height = np.shape(parentGrid)[0]
    fig, ax = plt.subplots()
    ax.set_xticks(np.arange(0, width, 1))
    ax.set_yticks(np.arange(-height, 0, 1))
    x_pos = []
    y_pos = []
    x_direct = []
    y_direct = []
    for i in range(height):
        for j in range(width):
            curr = parentGrid[i][j]
            x_pos += [j+0.5]
            y_pos += [-i-0.5]
            if curr.getY() == j:
                if curr.getX()<i:
                    x_direct+=[0]
                    y_direct+=[1]
                else:
                    x_direct+=[0]
                    y_direct+=[-1]
            else:
                if curr.getY()>j:
                    x_direct+=[1]
                    y_direct+=[0]
                else:
                    x_direct+=[-1]
                    y_direct+=[0]
    ax.quiver(x_pos,y_pos,x_direct,y_direct)
    plt.grid()
    plt.title('Optimal Path')
    plt.show()

def main():

    grid = Grid()
    grid.printGrid()

    grid.toggleTile(2,2,False)
    grid.toggleTile(2,3,False)
    grid.toggleTile(2,1,False)

    grid.dijkstra(3,3)

    for row in grid.parentGrid:
        for pt in row:
            print( "(" + str(pt.getX()) + ", " + str(pt.getY()) + ")" )
    visualization(grid.parentGrid)
    """
    while( userInput != 'x' )

        userInput = raw_input("WASD To Move Car: ")

        if( userInput == 'w' || 'W' ):
            car.moveU()

        elif( userInput == 's' || 'S' ):
            car.moveD()

        elif( userInput == 'd' || 'D' ):
            car.moveR()

        elif( userInput == 'a' || 'A' ):
            car.moveL()
        
        for i in range(3):
            for j in range(3):
                if(
    """

if __name__ == '__main__':
    main()

