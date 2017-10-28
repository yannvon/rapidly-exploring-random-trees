#!/usr/bin/env python
#
#TODO was forced to add this in order to execute it


import time
import random
import drawSample
import math
import _tkinter
import sys
import imageToRects

#display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")

visualize = 1
prompt_before_next=1  # ask before re-running sonce solved
SMALLSTEP = 10 # what our "local planner" can handle.

XMAX=1800
YMAX=1000
G = [  [ 0 ]  , [] ]   # nodes, edges


s,obstacles = imageToRects.imageToRects(sys.argv[1])

XMAX = s[0]
YMAX = s[1]
       


# goal/target
tx = 800
ty = 150
ta = 0
# start
start_x = 100
start_y = 630
start_a = 0

vertices = [[start_x,start_y]]

sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0

nodes=0
edges=1
maxvertex = 0

#CONSTANTS ADDED
line_size = 50
extension_step_size = 3
# used to set to exporting mode
export = 1
csv = 0
improved = 1
count = 0


def drawGraph(G):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
        if len(vertices)!=1:
            canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


def genPoint():
    # Uniform distribution
    x = random.uniform(0, XMAX)
    y = random.uniform(0, YMAX)
    a = random.uniform(-math.pi, math.pi)
    global count
    if improved and count % 10 == 0:
        count = 0
        x = tx
        y = ty
    count += 1
    return [x, y, a]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    line = []
    llsq = 0.0 # line length squared
    for i in range(len(p1)):  # each dimension
        h = p2[i] - p1[i] 
        line.append( h )
        llsq += h*h
    ll = math.sqrt(llsq)  # length
    # normalize line
    if ll <=0: return [0,0]
    for i in range(len(p1)):  # each dimension
        line[i] = line[i]/ll
    return line

def pointPointDistance(p1,p2):
    """ Return the distance between a pair of points (L2 norm). """
    llsq = 0.0 # line length squared
    for i in range(len(p1)):  # each dimension, general case
        h = p2[i] - p1[i] 
        llsq = llsq + (h*h)
    return math.sqrt(llsq)

def closestPointToPoint(G,p2):
    dmin = 999999999
    for v in G[nodes]:
        p1 = vertices [ v ]
        d = pointPointDistance(p1,p2)
        if d <= dmin:
            dmin = d
            close = v
    return close

def returnParent(k):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            #canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            line = (line_size / 2 * math.cos(vertices[e[0]][2]), line_size / 2 * math.sin(vertices[e[0]][2]))
            objectTail = (vertices[e[0]][0] - line[0], vertices[e[0]][1] - line[1])
            objectHead = (vertices[e[0]][0] + line[0], vertices[e[0]][1] + line[1])

            canvas.polyline([objectTail, objectHead], style=3 )
            return e[0]

def pickGvertex():
    try: edge = random.choice( G[edges] )
    except: return pickvertex()
    v = random.choice( edge )
    return v

def redraw():
    canvas.clear()
    canvas.markit(start_x, start_y, r=SMALLSTEP)
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")

def ccw(A,B,C):
    """ Determine if three points are listed in a counterclockwise order.
    For three points A, B and C. If the slope of the line AB is less than 
    the slope of the line AC then the three points are in counterclockwise order.
    See:  http://compgeom.cs.uiuc.edu/~jeffe/teaching/373/notes/x06-sweepline.pdf
    """
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
        """ do lines AB and CD intersect? """
        i = ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        #if i: 
        #    canvas.polyline(  [ A,B ], style=4  , tags = ("debug"))
        #    canvas.polyline(  [ C,D ], style=4  , tags = ("debug"))
        #else: 
        #    canvas.polyline(  [ A,B ], style=1  , tags = ("debug")) # green
        #    canvas.polyline(  [ C,D ], style=1  , tags = ("debug"))
        return i
        

def lineHitsRect(p1,p2,r):
   rline = ( (r[0],r[1]), (r[0],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[0],r[1]), (r[2],r[1]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[0],r[3]), (r[2],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[2],r[1]), (r[2],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1

   return 0

def inRect(p,rect,dilation):
   """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
   if p[0]<rect[0]-dilation: return 0
   if p[1]<rect[1]-dilation: return 0
   if p[0]>rect[2]+dilation: return 0
   if p[1]>rect[3]+dilation: return 0
   return 1


def rrt_search(G, tx, ty, ta):
    # Iterate until close enough to solution
    dist = SMALLSTEP + 1
    num_rrt_iterations = 0;

    while dist > SMALLSTEP:
        num_rrt_iterations += 1
        dist = pointPointDistance(vertices[closestPointToPoint(G, (tx, ty, ta))], (tx, ty, ta))
        #print dist

        # Step 1: generate random point
        xrand = genPoint()

        # Step 2: find nearest vertex in graph
        xnearest = closestPointToPoint(G, xrand)

        # Step 3: steer towards it
        nearestP = vertices[xnearest]
        xnew = []
        vector = lineFromPoints(nearestP, xrand)
        xnew.append(nearestP[0] + extension_step_size * vector[0])
        xnew.append(nearestP[1] + extension_step_size * vector[1])
        xnew.append((nearestP[2] + vector[2]) % math.pi) #fixme dont understand this.. do !

        # Step 4.1: only add if inside
        dontAdd = 0
        if xnew[0] > XMAX or xnew[0] < 0 or xnew[1] > YMAX or xnew[1] < 0:
            dontAdd = 1

        # Step 4.2: only add it if it is obstacle free, but we only check endstate!
        for o in obstacles:
            line = (line_size/2*math.cos(xnew[2]), line_size/2*math.sin(xnew[2]))
            objectTail = (xnew[0] - line[0], xnew[1] - line[1])
            objectHead = (xnew[0] + line[0], xnew[1] + line[1])
            # this check is not really enough, but we suppose that our small movements are small enough

            if lineHitsRect(objectTail, objectHead, o) \
                    or inRect(objectHead, o, 0.01) \
                    or inRect(objectTail, o, 0.01):
                # there is a collision!
                dontAdd = 1

        # Step 5: add it to graph
        if dontAdd == 0:
            xnewIndex = pointToVertex(xnew)
            G[nodes].append(xnewIndex)
            G[edges].append((xnearest, xnewIndex))

        # Step 6: visualize change
        if visualize and dontAdd == 0:
            canvas.polyline([objectTail, objectHead])
            canvas.polyline([vertices[xnearest], vertices[xnewIndex]]) #fixme why does this work? should it not be 2D?
            canvas.events()

    # Step 7: return number of iterations needed
    return num_rrt_iterations


#ADDITIONAL FUNCTION: retraces shortest path
def retrace_shortest_path(G):
    xnearest = closestPointToPoint(G, (tx, ty, ta))
    xnew = (tx, ty, ta)
    xnewIndex = pointToVertex(xnew)
    G[nodes].append(xnewIndex)
    G[edges].append((xnearest, xnewIndex))

    node = len(vertices) - 1
    rrt_path_length = 0

    while node != 0:
        # print node
        node = returnParent(node)
        rrt_path_length += 1

    return rrt_path_length


if visualize:
    canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)


if 0:  # line intersection testing
        obstacles.append( [ 75,60,125,500 ] )  # tall vertical
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
        lines = [
           ( (70,50), (150,150) ),
           ( (50,50), (150,20) ),
           ( (20,20), (200,200) ),
           ( (300,300), (20, 200)  ),
           ( (300,300), (280, 90)  ),
           ]
        for l in lines:
           for o in obstacles:
              lineHitsRect(l[0],l[1],o)
        canvas.mainloop()
    

if 0:
    # random obstacle field
    for nobst in range(0,6000):
        wall_discretization=SMALLSTEP*2  # walls are on a regular grid.
        wall_lengthmax=10.  # fraction of total (1/lengthmax)
        x = wall_discretization*int(random.random()*XMAX/wall_discretization)
        y = wall_discretization*int(random.random()*YMAX/wall_discretization)
        #length = YMAX/wall_lengthmax
        length = SMALLSTEP*2
        if random.choice([0,1]) >0:
            obstacles.append( [ x,y,x+SMALLSTEP,y+10+length ] )  # vertical
        else:
            obstacles.append( [ x,y,x+10+length,y+SMALLSTEP ] )  # horizontal
else:
  if 0:
    # hardcoded simple obstacles
    obstacles.append( [ 300,0,400,95 ] )  # tall vertical
    # slightly hard
    obstacles.append( [ 300,805,400,YMAX ] )  # tall vertical
    #obstacles.append( [ 300,400,1300,430 ] )
    # hard
    obstacles.append( [ 820,220,900,940 ] )
    obstacles.append( [ 300,0,  400,95 ] )  # tall vertical
    obstacles.append( [ 300,100,400,YMAX ] )  # tall vertical
    obstacles.append( [ 200,300,800,400 ] )  # middle horizontal
    obstacles.append( [ 380,500,700,550 ] )
    # very hard
    obstacles.append( [ 705,500,XMAX,550 ] )




if visualize:
    for o in obstacles: canvas.showRect(o, outline='red', fill='blue')


maxvertex += 1

if export:
    file = "line_robot.csv"  # where you want the file to be downloaded to

    csv = open(file, "w")

    columnTitleRow = "step_size, line_size, num_rrt_iterations, rrt_path_length\n" #fixme
    csv.write(columnTitleRow)


while 1:
    #change line_size from 25 to 75
    line_size += 5
    if line_size > 75:
        line_size = 25
    # graph G
    G = [[0], []]   # nodes, edges
    vertices = [[start_x, start_y, start_a]] #fixme
    redraw()

    if visualize:
        canvas.markit( tx, ty, r=SMALLSTEP )
        drawGraph(G)

    num_rrt_iterations = rrt_search(G, tx, ty, ta)
    print "number of iterations: {}".format(num_rrt_iterations)

    # ADDITIONAL CODE
    # retrace shortest path
    rrt_path_length = retrace_shortest_path(G)

    #export into csv file for conveniance
    if export:
        row = "{},{},{},{}\n".format(extension_step_size, line_size, num_rrt_iterations, rrt_path_length)
        csv.write(row)

    # comment this to keep previous graphs displayed
    #canvas.delete()

    print "rrt_path_size: {} extension_step_size: {}".format(rrt_path_length, extension_step_size)
    print "total path length: {} shortest path: {}".format(rrt_path_length*extension_step_size, pointPointDistance((start_x, start_y), (tx, ty)))


csv.close()


#canvas.showRect(rect,fill='red')

if visualize:
    canvas.mainloop()
