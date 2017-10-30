#!/usr/bin/env python
#

import time
import random
import drawSample
import math
import _tkinter
import sys
import imageToRects
import numpy

#display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
visualize = 1
prompt_before_next = 0  # ask before re-running once solved
SMALLSTEP = 10 # what our "local planner" can handle.

XMAX=1800
YMAX=1000
G = [[0], []]   # nodes, edges

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

vertices = [[start_x, start_y]]

sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0

nodes=0
edges=1

# CONSTANTS ADDED
line_size = 50
max_iterations = 1000000 #fixme

# Used to set to exporting mode and improved heuristic
export = 1
csv = 0
improved = 0
count = 0
n_samples = 10


def drawGraph(G):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
        if len(vertices) != 1:
            canvas.polyline([vertices[i[0]], vertices[i[1]]])


def genPoint():
    # Uniform distribution
    x = random.uniform(0, XMAX)
    y = random.uniform(0, YMAX)
    a = random.uniform(0, math.pi)

    # Additional improvement idea, not used though.
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


def lineFromPoints(p1, p2):
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
            # Added drawing of lines
            line = (line_size / 2 * math.cos(vertices[e[0]][2]), line_size / 2 * math.sin(vertices[e[0]][2]))
            objectTail = (vertices[e[0]][0] - line[0], vertices[e[0]][1] - line[1])
            objectHead = (vertices[e[0]][0] + line[0], vertices[e[0]][1] + line[1])
            if visualize:
                canvas.polyline([objectTail, objectHead], style=5)
                canvas.polyline([vertices[e[0]], vertices[e[1]]], style=3)
                canvas.events()
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
    dist = pointPointDistance(vertices[closestPointToPoint(G, (tx, ty, ta))], (tx, ty, ta))
    iterations = 0;

    while dist > SMALLSTEP and iterations < max_iterations :
        iterations += 1

        # Step 1: generate random point
        xrand = genPoint()

        # Step 2: find nearest vertex in graph
        xnearest = closestPointToPoint(G, xrand)

        # NOTE: Step 2 does not always correctly chose the correct angle since angle 0 and Pi are considered
        #       being far apart, where as in reality they aren't. This is not extremely crucial for the algorithm
        #       and was thus not taken into consideration.
        #       The same remark also applies for the calculation of the distance to the end point.

        # Step 3: steer towards it
        nearestP = vertices[xnearest]
        xnew = []

        # NOTE: Treat angles differently to ensure that we steer towards the closest angle!
        #       If we are in this case we remove or add pi in order to steer towards the right angle.
        #       We will later take the modulus of the angle to ensure that the angle stays in the 0, Pi range.
        if xrand[2] - nearestP[2] > math.pi / 2:
            xrand[2] -= math.pi
        elif nearestP[2] - xrand[2] > math.pi / 2:
            xrand[2] += math.pi

        vector = lineFromPoints(nearestP, xrand)
        xnew.append(nearestP[0] + SMALLSTEP * vector[0])
        xnew.append(nearestP[1] + SMALLSTEP * vector[1])
        xnew.append((nearestP[2] + SMALLSTEP * vector[2]) % math.pi)

        # Step 4.1: only add vertex if it is inside
        dontAdd = 0
        if xnew[0] > XMAX or xnew[0] < 0 or xnew[1] > YMAX or xnew[1] < 0:
            dontAdd = 1

        # Step 4.2: only add it if it is obstacle free, but we only check endstate!
        else:
            for o in obstacles:
                line = (line_size/2*math.cos(xnew[2]), line_size/2*math.sin(xnew[2]))
                objectTail = (xnew[0] - line[0], xnew[1] - line[1])
                objectHead = (xnew[0] + line[0], xnew[1] + line[1])

                # Note: The following condition to check for collision is not really precise,
                #       but we suppose that our small movements are small enough such that
                #       we can use this simplification.
                if lineHitsRect(objectTail, objectHead, o) \
                        or inRect(objectHead, o, 0.1) \
                        or inRect(objectTail, o, 0.1):
                    # there is a collision!
                    dontAdd = 1
                    break

        # Step 5: add it to graph
        if dontAdd == 0:
            xnewIndex = pointToVertex(xnew)
            G[nodes].append(xnewIndex)
            G[edges].append((xnearest, xnewIndex))

        # Step 6: visualize change
        if visualize:
            if dontAdd == 0:
                canvas.polyline([vertices[xnearest], vertices[xnewIndex]])
            canvas.events()

        # Step 7: update new distance to goal state
        dist = pointPointDistance(vertices[closestPointToPoint(G, (tx, ty, ta))], (tx, ty, ta))

    # Step 8: return number of iterations needed
    return iterations


# Additional function that retraces shortest path
def retrace_shortest_path(G):
    # Add goal vertex and then retrace graph from there
    xnearest = closestPointToPoint(G, (tx, ty, ta))
    xnew = (tx, ty, ta)
    xnewIndex = pointToVertex(xnew)
    G[nodes].append(xnewIndex)
    G[edges].append((xnearest, xnewIndex))

    node = len(vertices) - 1
    path_length = 0

    # Iterate through graph
    while node != 0:
        node = returnParent(node)
        path_length += 1

    return path_length


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
    for o in obstacles:
        canvas.showRect(o, outline='red', fill='blue')

# Additional code that allows for easy exporting of the gathered data.
if export:
    filename = "line_robot.csv"
    csv = open(filename, "w")
    titleRow = "step_size, line_size, num_rrt_iterations, rrt_path_length, std_num_rrt_iterations\n"
    csv.write(titleRow)

for line_size in range(10, 200, 20):

    samples_rrt_it = []
    samples_path_len = []
    for i in range(0, n_samples):
        print "line size: {}, sample: {}".format(line_size, i)
        # graph G
        G = [[0], []]   # nodes, edges
        vertices = [[start_x, start_y, start_a]]

        if visualize:
            redraw()
            canvas.markit( tx, ty, r=SMALLSTEP )
            drawGraph(G)

        num_rrt_iterations = rrt_search(G, tx, ty, ta)

        # retrace shortest path
        rrt_path_length = retrace_shortest_path(G)

        if export:
            # store data
            samples_path_len.append(rrt_path_length)
            samples_rrt_it.append(num_rrt_iterations)

        if prompt_before_next:
            raw_input("press Enter to continue..")

        # comment this to keep previous graphs displayed
        if visualize:
            canvas.delete()

        print numpy.std(samples_rrt_it, ddof=1)

    # export into csv file for convenience
    if export:
        row = "{},{},{},{},{}\n".format(SMALLSTEP, line_size, numpy.average(samples_rrt_it),
                                        numpy.average(samples_path_len),
                                        numpy.std(samples_rrt_it, ddof=1))
        csv.write(row)

csv.close()


#canvas.showRect(rect,fill='red')

if visualize:
    canvas.mainloop()
