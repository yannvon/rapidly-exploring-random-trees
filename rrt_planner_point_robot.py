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

# display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
visualize = 1
prompt_before_next = 1  # ask before re-running sonce solved #fixme use?
SMALLSTEP = 4  # what our "local planner" can handle.

XMAX = 1800
YMAX = 1000
G = [[0], []]  # nodes, edges

s, obstacles = imageToRects.imageToRects(sys.argv[1])

XMAX = s[0]
YMAX = s[1]

# goal/target
tx = 900
ty = 300
# start
start_x = 10
start_y = 270

vertices = [[start_x, start_y]]

sigmax_for_randgen = XMAX / 2.0 #fixme what is this used for?
sigmay_for_randgen = YMAX / 2.0

nodes = 0
edges = 1
maxvertex = 0 #fixme what is this used for?

#FIXME CONSTANTS ADDED
extension_step_size = 5
num_rrt_iterations = 3000
x_variance = 250 #fixme #250 is good
y_variance = 250
count = 1 #used for genPoint() improvement
improved = 0
export = 1
csv = 0

def drawGraph(G):
    global vertices, nodes, edges   #todo why use global here?
    if not visualize: return
    for i in G[edges]:
        if len(vertices) != 1:
            canvas.polyline([vertices[i[0]], vertices[i[1]]])


def genPoint():
    # TODO : Function to implement the sampling technique

    # --- Unifrom Distribution
    x = random.uniform(0, XMAX)
    y = random.uniform(0, YMAX)

    # --- Gaussian distribution with mean at the goal
    #x = random.gauss(tx, x_variance)
    #y = random.gauss(ty, y_variance)

    #fixme improvement by taking goal one out of 10 times
    if (improved):
        global count
        count = count + 1

        if (count % 10 == 0):
            x = tx
            y = ty

    #fixme remove this output
    if visualize:
        canvas.markit(x, y, 0, 0.1)
    return [x, y]


def genvertex():
    vertices.append(genPoint())
    return len(vertices) - 1


def pointToVertex(p):
    vertices.append(p)
    return len(vertices) - 1


def pickvertex():
    return random.choice(range(len(vertices)))


def lineFromPoints(p1, p2):
    line = []
    llsq = 0.0  # line length squared
    for i in range(len(p1)):  # each dimension
        h = p2[i] - p1[i]
        line.append(h)
        llsq += h * h
    ll = math.sqrt(llsq)  # length
    # normalize line
    if ll <= 0: return [0, 0]
    for i in range(len(p1)):  # each dimension
        line[i] = line[i] / ll
    return line


def pointPointDistance(p1, p2):
    """ Return the distance between a pair of points (L2 norm). """
    llsq = 0.0  # line length squared
    # faster, only for 2D
    h = p2[0] - p1[0]
    llsq = llsq + (h * h)
    h = p2[1] - p1[1]
    llsq = llsq + (h * h)
    return math.sqrt(llsq)


def closestPointToPoint(G, p2):
    dmin = 999999999
    for v in G[nodes]:
        p1 = vertices[v]
        d = pointPointDistance(p1, p2)
        if d <= dmin:
            dmin = d
            close = v
    return close


def returnParent(k):
    """ Return parent node for input node k. """
    for e in G[edges]:
        if e[1] == k:
            if visualize:
                canvas.polyline([vertices[e[0]], vertices[e[1]]], style=3)
            return e[0]


def pickGvertex():
    try:
        edge = random.choice(G[edges])
    except:
        return pickvertex()
    v = random.choice(edge)
    return v


def redraw():
    canvas.clear()
    canvas.markit(start_x, start_y, r=SMALLSTEP)
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def ccw(A, B, C):
    """ Determine if three points are listed in a counterclockwise order.
    For three points A, B and C. If the slope of the line AB is less than
    the slope of the line AC then the three points are in counterclockwise order.
    See:  http://compgeom.cs.uiuc.edu/~jeffe/teaching/373/notes/x06-sweepline.pdf
    """
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def intersect(A, B, C, D):
    """ do lines AB and CD intersect? """
    i = ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
    # if i:
    #    canvas.polyline(  [ A,B ], style=4  , tags = ("debug"))
    #    canvas.polyline(  [ C,D ], style=4  , tags = ("debug"))
    # else:
    #    canvas.polyline(  [ A,B ], style=1  , tags = ("debug")) # green
    #    canvas.polyline(  [ C,D ], style=1  , tags = ("debug"))
    return i


def lineHitsRect(p1, p2, r):
    rline = ((r[0], r[1]), (r[0], r[3]))
    if intersect(p1, p2, rline[0], rline[1]): return 1
    rline = ((r[0], r[1]), (r[2], r[1]))
    if intersect(p1, p2, rline[0], rline[1]): return 1
    rline = ((r[0], r[3]), (r[2], r[3]))
    if intersect(p1, p2, rline[0], rline[1]): return 1
    rline = ((r[2], r[1]), (r[2], r[3]))
    if intersect(p1, p2, rline[0], rline[1]): return 1

    return 0


def inRect(p, rect, dilation):
    """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
    if p[0] < rect[0] - dilation: return 0
    if p[1] < rect[1] - dilation: return 0
    if p[0] > rect[2] + dilation: return 0
    if p[1] > rect[3] + dilation: return 0
    return 1


# TODO: Implement the rrt_search algorithm in this function.
def rrt_search(G, tx, ty):


    # Iterate until close enough to solution
    dist = 100
    num_rrt_iterations = 0;  # fixme change use of variable at start of script
    drawGraph(G)
    while (dist > SMALLSTEP):
        num_rrt_iterations += 1
        dist = pointPointDistance(vertices[closestPointToPoint(G, (tx, ty))], (tx, ty))
        print dist

        # Step 1: generate random point
        xrand = genPoint()

        # Step 2: find nearest vertex in graph
        xnearest = closestPointToPoint(G, xrand)

        # Step 3: steer towards it
        nearestP = vertices[xnearest]
        xnew = map( lambda pair: pair[0] + extension_step_size*pair[1], zip(nearestP, lineFromPoints(nearestP, xrand)))

        #Step 4.1: only add if inside
        dontAdd = 0
        if (xnew[0] > XMAX or xnew[0] < 0 or xnew[1] > YMAX or xnew[1] < 0):
            dontAdd = 1
        # Step 4.2: only add it if it is obstacle free
        for o in obstacles:
            if lineHitsRect(nearestP, xnew, o):
            #if inRect(xnew, o, 0.001): #fixme dilation? doesn't work, can skip over a rectangle!
                #there is a collision!
                dontAdd = 1

        # Step 5: add it to graph
        if dontAdd == 0 :
            xnewIndex = pointToVertex(xnew)
            G[nodes].append(xnewIndex)
            G[edges].append((xnearest, xnewIndex))


        # Step 6: visualize change
        if visualize and dontAdd == 0:
            canvas.polyline([vertices[xnearest], vertices[xnewIndex]])
            canvas.events()

    # Step 7: return number of iterations needed
    return num_rrt_iterations

#ADDITIONAL FUNCTION: retraces shortest path
def retrace_shortest_path(G):
    xnearest = closestPointToPoint(G, (tx, ty))
    xnew = (tx, ty)
    xnewIndex = pointToVertex(xnew)
    G[nodes].append(xnewIndex)
    G[edges].append((xnearest, xnewIndex))

    node = len(vertices) - 1
    rrt_path_length = 0

    while (node != 0):
        # print node
        node = returnParent(node)
        rrt_path_length += 1

    return rrt_path_length


if visualize:
    canvas = drawSample.SelectRect(xmin=0, ymin=0, xmax=XMAX, ymax=YMAX, nrects=0,
                                   keepcontrol=0)  # , rescale=800/1800.)

if 0:  # line intersection testing
    obstacles.append([75, 60, 125, 500])  # tall vertical
    for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    lines = [
        ((70, 50), (150, 150)),
        ((50, 50), (150, 20)),
        ((20, 20), (200, 200)),
        ((300, 300), (20, 200)),
        ((300, 300), (280, 90)),
    ]
    for l in lines:
        for o in obstacles:
            lineHitsRect(l[0], l[1], o)
    canvas.mainloop()

if 0: #fixme was 0
    # random obstacle field
    for nobst in range(0, 60): #fixme was 6000
        wall_discretization = SMALLSTEP * 2  # walls are on a regular grid.
        wall_lengthmax = 10.  # fraction of total (1/lengthmax)
        x = wall_discretization * int(random.random() * XMAX / wall_discretization)
        y = wall_discretization * int(random.random() * YMAX / wall_discretization)
        # length = YMAX/wall_lengthmax
        length = SMALLSTEP * 2
        if random.choice([0, 1]) > 0:
            obstacles.append([x, y, x + SMALLSTEP, y + 10 + length])  # vertical
        else:
            obstacles.append([x, y, x + 10 + length, y + SMALLSTEP])  # horizontal
else:
    if 0:
        # hardcoded simple obstacles
        obstacles.append([300, 0, 400, 95])  # tall vertical
        # slightly hard
        obstacles.append([300, 805, 400, YMAX])  # tall vertical
        # obstacles.append( [ 300,400,1300,430 ] )
        # hard
        obstacles.append([820, 220, 900, 940])
        obstacles.append([300, 0, 400, 95])  # tall vertical
        obstacles.append([300, 100, 400, YMAX])  # tall vertical
        obstacles.append([200, 300, 800, 400])  # middle horizontal
        obstacles.append([380, 500, 700, 550])
        # very hard
        obstacles.append([705, 500, XMAX, 550])

if visualize:
    for o in obstacles: canvas.showRect(o, outline='red', fill='blue')

maxvertex += 1
if export:
    file = "point_robot.csv"  # where you want the file to be downloaded to

    csv = open(file, "w")

    columnTitleRow = "step_size, num_rrt_iterations, rrt_path_length\n"
    csv.write(columnTitleRow)


for i in range(1, 15):   #fixme was while 1
    extension_step_size = i
    for j in range(0, 20):

        # graph G
        G = [[0], []]  # nodes, edges
        vertices = [[10, 270], [20, 280]]
        if visualize:
            redraw()

        G[edges].append((0, 1))
        G[nodes].append(1)
        if visualize: canvas.markit(tx, ty, r=SMALLSTEP)

        if visualize:
            drawGraph(G)
        num_rrt_iterations = rrt_search(G, tx, ty)


        print "number of iterations: {}".format(num_rrt_iterations)

        # ADDITIONAL CODE
        # retrace shortest path
        rrt_path_length = retrace_shortest_path(G)

        #if visualize:   #fixme is this call necessary?
        #    drawGraph(G)

        if export:
            row = "{},{},{}\n".format(extension_step_size, num_rrt_iterations, rrt_path_length)
            csv.write(row)

        # Uncommonet this to keep previous graphs displayed
        canvas.delete()

        print "rrt_path_size: {} extension_step_size: {}".format(rrt_path_length, extension_step_size)
        print "total path length: {} shortest path: {}".format(rrt_path_length*extension_step_size, pointPointDistance((start_x, start_y), (tx, ty)))
        #fixme distance is incorrect since first step is weird and last step too

csv.close()
# canvas.showRect(rect,fill='red')

if visualize:
    canvas.mainloop()
