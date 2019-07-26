import rospy
from Queue import Queue
import numpy as np
import pprint

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
    
    def printMe(self):
        print ( "(" + str(self.x) + ", " + str(self.y) + ")" )

class PhonyCar(object):

    def __init__(self, x = 0, y = 0):
        
        self.location = Point()
        self.location.setPt(x, y)

        self.dirStack = []

    #TODO: Change values of array indexes

    def moveU(self):
        if(self.location.getX() - 1 < 0):
            print("Too far up! Outside of track boundary.")
        else:
            self.location.setX( self.location.getX() - 1 )

    def moveD(self):
        if(self.location.getX() + 1 > 3):
            print("Too far down! Outside of track boundary.")
        else:
            self.location.setX( self.location.getX() + 1 )

    def moveR(self):
        if(self.location.getY() + 1 > 3):
            print("Too far right! Outside of track boundary.")
        else:
            self.location.setY( self.location.getY() + 1 )

    def moveL(self):
        if(self.location.getY() - 1 < 0):
            print("Too far left! Outside of track boundary.")
        else:
            self.location.setY( self.location.getY() - 1 )

    def getLoc(self):
        return self.location

    def pushDir(self, instr):
        self.dirStack.append(instr)

    def popDir(self):
        return self.dirStack.pop()

    
        

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

    def dijkstra(self, start_x, start_y):
        
        nodes = Queue()
        self.parentGrid = [] 
        costGrid = []

        
        for i in range(4):
            self.parentGrid.append( [ Point(), Point(), Point(), Point() ] )
            costGrid.append( [ np.inf, np.inf, np.inf, np.inf ] )

        start = Point()
        start.setPt( start_x, start_y )
           
        self.parentGrid[start_x][start_y].setPt(start_x, start_y)
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

    def createDirStack(self, destination):

        dirStack = []

        dirStack.append(destination)

        dest_x = destination.getX()
        dest_y = destination.getY()

        nextPt = Point()


        while(True):
            nextPt = self.parentGrid[dest_x][dest_y]
                
            if( nextPt.getX() == dest_x and nextPt.getY() == dest_y ):
                break
            else:
                dirStack.append(nextPt)
                dest_x = nextPt.getX()
                dest_y = nextPt.getY()

        return dirStack



def main():
    
    grid = Grid()
    grid.printGrid()

    grid.dijkstra(2, 1)

    destination = Point(3, 3)
    
    
    dirStack = grid.createDirStack(destination)

    for point in dirStack:
        point.printMe()
    

    """
    for row in grid.parentGrid:
        for pt in row:
            print( "(" + str(pt.getX()) + ", " + str(pt.getY()) + ")" )
    #userInput = 'o'
    """
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

