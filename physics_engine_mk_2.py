import sys, pygame, math
import pygame.freetype
pygame.init()
screen = pygame.display.set_mode((1500,1000))
clock = pygame.time.Clock()
DISPLAYSURF = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.display.set_caption("Physics engine mk. 1")
display_font = pygame.freetype.SysFont('Arial', 24)
objectcolour = (60,60,60)
black = (0,0,0)
red = (255,0,0)
blue = (0,255,0)
bounces = 0
objects = []
spawnShape = "square"

def dot(a,b):
    return(a[0]*b[0] + a[1]*b[1])

def vecSum(a,b):
    return([a[0] + b[0], a[1] + b[1]])

def vecDiff(a,b):
    return([a[0] - b[0], a[1] - b[1]])

def vecScale(vec,const):
    return([vec[0] * const, vec[1] * const])

def normalize(a):
    magnitude = math.sqrt(a[0]**2 + a[1]**2)
    if magnitude == 0:
        return [0,0]
    return([a[0] / magnitude, a[1] / magnitude])

def magnitude(a):
    return(math.sqrt(a[0]**2 + a[1]**2))

def crossProduct1(a,b):
    return(a[0]*b[1] - a[1]*b[0])

def crossProduct2(vector,s):
    return([s*vector[1], -s*vector[0]])

def crossProduct3(s, vector):
    return([-s*vector[1], s*vector[0]])

def normalR(vector):
    return([vector[1], -vector[0]])

def normalL(vector):
    return([-vector[1], vector[0]])

# Polygon, center, points, mass, velocity, elasticity, friction, orientation, angular velocity, worldspace points, moment of inertia
def addPolygon(center, points, mass, velocity,elasticity,friction, orientation,angularV, inertia):
    totalDistance = 0
    for i in points:
        totalDistance += math.sqrt(i[0]**2 + i[1]**2)
    averageDistance = totalDistance/len(points)
    Box = ["Polygon", center, points, mass, velocity,elasticity,friction, math.radians(orientation),angularV,[], inertia] #mass*math.pi*averageDistance**4 / 4
    objects.append(Box)

# Circle, center, radius, mass, velocity, elasticity, friction, orientation, angular velocity, blank, moment of inertia
def addCircle(center, radius, mass, velocity, elasticity,friction):
    Circle = ["Circle", center, radius, mass, velocity, elasticity,friction,0,0, 0,mass*math.pi*radius**4 / 8000]
    objects.append(Circle)

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def checkCollision(object1, object2):
    if (object1[0] == "Polygon") & (object2[0] == "Polygon"):
        #print("collision check")
        for i in range(len(object1[9])):
            for j in range(len(object2[9])):
                if intersect(object1[9][i], object1[9][(i+1) % len(object1[9])], object2[9][j], object2[9][(j+1) % len(object2[9])]):
                    #print("colliding")
                    return True
        #print("not colliding")
        return False

    if (object1[0] == "Circle") & (object2[0] == "Circle"):
        radius = (object1[2] + object2[2])**2
        #print((object1[1][0] - object2[1][0])**2 + (object1[1][1] - object2[1][1])**2 - radius)
        if (object1[1][0] - object2[1][0])**2 + (object1[1][1] - object2[1][1])**2 < radius:
            #print("collision")
            return True
        else: return False

    if (object1[0] == "Circle") & (object2[0] == "Polygon"):
        bestDepth = -9999
        for i in range(len(object2[9])):
            p1 = object2[9][i]
            p2 = object2[9][(i+1) % len(object2[9])]
            d = crossProduct1(vecDiff(p2, p1), vecDiff(object1[1], p1)) / magnitude(vecDiff(p2, p1))
            if d > bestDepth:
                bestDepth = d
                closestSide = i

        radius = object1[2]**2
        if dot(vecDiff(object1[1], object2[9][closestSide]), vecDiff(object2[9][closestSide], object2[9][(closestSide+1) % len(object2[9])])) >= 0:
            if (object1[1][0] - object2[9][closestSide][0])**2 + (object1[1][1] - object2[9][closestSide][1])**2 < radius:
                return True
            else: return False

        if dot(vecDiff(object1[1], object2[9][(closestSide+1) % len(object2[9])]), vecDiff(object2[9][(closestSide+1) % len(object2[9])], object2[9][closestSide])) >= 0:
            if (object1[1][0] - object2[9][(closestSide+1) % len(object2[9])][0])**2 + (object1[1][1] - object2[9][(closestSide+1) % len(object2[9])][1])**2 < radius:
                return True
            else: return False
        
        if bestDepth <= object1[2]:
            return True
        else: return False
        
        

    if (object1[0] == "Polygon") & (object2[0] == "Circle"):
        return checkCollision(object2, object1)


    return False
    
def findDepth(point,polygon):
    bestDepth = -9999
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i+1) % len(polygon)]
        d = crossProduct1(vecDiff(p2, p1), vecDiff(point, p1)) / magnitude(vecDiff(p2, p1))
        if d > bestDepth:
            bestDepth = d
    return(bestDepth)


def findCollisonNormal(object1,object2):
    global collisionDepth
    global supportPoint
    if (object1[0] == "Polygon") & (object2[0] == "Polygon"):
        collisionDepth = 10000
        for i in range(len(object1[9])):
            depth = findDepth(object1[9][i], object2[9])
            if depth < collisionDepth:
                referenceObject = 1
                collisionDepth = depth
                supportPoint = object1[9][i]
            elif (depth <= 0) & (abs(depth)-abs(collisionDepth) < 1):
               supportPoint[0] = (supportPoint[0] + object1[9][i][0]) /2
               supportPoint[1] = (supportPoint[1] + object1[9][i][1]) /2

        for i in range(len(object2[9])):
            depth = findDepth(object2[9][i], object1[9])
            if depth < collisionDepth:
                referenceObject = 2
                collisionDepth = depth
                supportPoint = object2[9][i]
            elif (depth <= 0) & (abs(depth)-abs(collisionDepth) < 1):
                supportPoint[0] = (supportPoint[0] + object2[9][i][0]) /2
                supportPoint[1] = (supportPoint[1] + object2[9][i][1]) /2
            
        normal = 0
        if referenceObject == 1:
            for i in range(len(object2[9])):
                if intersect(object1[1], supportPoint, object2[9][i], object2[9][(i+1) % len(object2[9])]): #vecSum(supportPoint, vecScale(vecDiff(supportPoint, object1[1]), 0.05))
                    normal = normalR(vecDiff(object2[9][(i+1) % len(object2[9])], object2[9][i]))
                    collisionDepth = crossProduct1(vecDiff(object2[9][(i+1) % len(object2[9])], object2[9][i]), vecDiff(supportPoint, object2[9][i])) / magnitude(vecDiff(object2[9][(i+1) % len(object2[9])], object2[9][i]))
                    break
        else:
            for i in range(len(object1[9])):
                if intersect(object2[1], supportPoint, object1[9][i], object1[9][(i+1) % len(object2[9])]):
                    normal = normalL(vecDiff(object1[9][(i+1) % len(object1[9])], object1[9][i]))
                    collisionDepth = crossProduct1(vecDiff(object1[9][(i+1) % len(object1[9])], object1[9][i]), vecDiff(supportPoint, object1[9][i])) / magnitude(vecDiff(object1[9][(i+1) % len(object1[9])], object1[9][i]))
                    break
        if normal == 0:
            normal = vecDiff(object2[1], object1[1])

        collisionDepth = abs(collisionDepth)
        normal = normalize(normal)
        return(normal)

    if (object1[0] == "Circle") & (object2[0] == "Circle"):
        r = object1[2] + object2[2]
        d = math.sqrt((object1[1][0] - object2[1][0])**2 + (object1[1][1] - object2[1][1])**2)
        collisionDepth = r - d
        normal = [object2[1][0] - object1[1][0], object2[1][1] - object1[1][1]]
        normal = normalize(normal)
        supportPoint = [(object1[1][0] + object2[1][0]) / 2, (object1[1][1] + object2[1][1]) / 2]
        #print (normal)
        return normal

    if (object1[0] == "Circle") & (object2[0] == "Polygon"):
        bestDepth = -9999
        for i in range(len(object2[9])):
            p1 = object2[9][i]
            p2 = object2[9][(i+1) % len(object2[9])]
            d = crossProduct1(vecDiff(p2, p1), vecDiff(object1[1], p1)) / magnitude(vecDiff(p2, p1))
            if d > bestDepth:
                bestDepth = d
                closestSide = i

        radius = object1[2]
        if dot(vecDiff(object1[1], object2[9][closestSide]), vecDiff(object2[9][closestSide], object2[9][(closestSide+1) % len(object2[9])])) >= 0:
            collisionDepth = radius - math.sqrt((object1[1][0] - object2[9][closestSide][0])**2 + (object1[1][1] - object2[9][closestSide][1])**2)
            supportPoint = object2[9][closestSide]
            normal = vecDiff(object2[9][closestSide], object1[1])
            normal = normalize(normal)
            return normal

        if dot(vecDiff(object1[1], object2[9][(closestSide+1) % len(object2[9])]), vecDiff(object2[9][(closestSide+1) % len(object2[9])], object2[9][closestSide])) >= 0:
            collisionDepth = radius - math.sqrt((object1[1][0] - object2[9][(closestSide+1) % len(object2[9])][0])**2 + (object1[1][1] - object2[9][(closestSide+1) % len(object2[9])][1])**2)
            supportPoint = object2[9][(closestSide+1) % len(object2[9])]
            normal = vecDiff(object2[9][(closestSide+1) % len(object2[9])], object1[1])
            normal = normalize(normal)
            return normal
        
        collisionDepth = abs(radius - bestDepth)
        normal = normalR(vecDiff(object2[9][(closestSide+1) % len(object2[9])], object2[9][closestSide]))
        normal = normalize(normal)
        supportPoint = vecDiff(object1[1], vecScale(normal,-radius))
        return normal


    if (object1[0] == "Polygon") & (object2[0] == "Circle"):
        normal = findCollisonNormal(object2, object1)
        return vecScale(normal, -1)

    #print("oops")
    return 0

def positionCorrection(object1,object2,collisionDepth,normalVector):
    if collisionDepth > 0.01:
        correctionPercent = 0.2
        correctionScalar = object1[3] / (object2[3] + object1[3])
        return([-normalVector[0] * collisionDepth * correctionScalar * correctionPercent, -normalVector[1] * collisionDepth * correctionScalar * correctionPercent])
    return([0,0])


#addPolygon([960,600],[[-50,50],[50,50],[50,-50],[-50,-50]],10,[0,0],0.2,0.5,10,0.01,0)
addPolygon([960,800],[[-1000,50],[1000,50],[1000,-50],[-1000,-50]],0,[0,0],0.2,0.5,0,0,0)
addPolygon([960,30],[[-1000,50],[1000,50],[1000,-50],[-1000,-50]],0,[0,0],0.2,0.5,0,0,0)
addPolygon([30,0],[[-1000,50],[1000,50],[1000,-50],[-1000,-50]],0,[0,0],0.2,0.5,90,0,0)
addPolygon([1290,0],[[-1000,50],[1000,50],[1000,-50],[-1000,-50]],0,[0,0],0.2,0.5,90,0,0)
addPolygon([300,500],[[-200,30],[200,30],[200,-30],[-200,-30]],0,[0,0],0.2,0.5,90,0.1,0)
addPolygon([800,500],[[-200,30],[200,30],[200,-30],[-200,-30]],0,[0,0],0.2,0.5,90,0.1,0)

def spawnSquare(mousePos):
    addPolygon([mousePos[0], mousePos[1]],[[-50,50],[50,50],[50,-50],[-50,-50]],10,[0,0],0.5,0.5,0,0.00,80000)

def spawnCircle(mousePos):
    addCircle([mousePos[0], mousePos[1]],50,10,[0,0],0.2,0.5)

def spawnTriangle(mousePos):
    addPolygon([mousePos[0], mousePos[1]],[[0,50],[50*math.sin(2*math.pi/3),50*math.cos(2*math.pi/3)],[50*math.sin(4*math.pi/3),50*math.cos(4*math.pi/3)]],10,[0,0],0.2,0.5,180,0.00,120000)

def spawnVerticalRect(mousePos):
    addPolygon([mousePos[0], mousePos[1]],[[-20,60],[20,60],[20,-60],[-20,-60]],6,[0,0],0.2,0.5,0,0.00,100000)

def spawnHorizontalRect(mousePos):
    addPolygon([mousePos[0], mousePos[1]],[[-20,60],[20,60],[20,-60],[-20,-60]],6,[0,0],0.2,0.5,90,0.00,100000)

def spawnCannonBall(mousePos):
    addCircle([mousePos[0], mousePos[1]],40,700,[30,0],0.5,0.2)

while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
        
        if event.type == pygame.MOUSEMOTION:
            mousePos = event.pos

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                sys.exit()
            if event.key == pygame.K_SPACE:
                if spawnShape == "square":
                    spawnShape = "circle"
                elif spawnShape == "circle":
                    spawnShape = "vertical rectangle"
                elif spawnShape == "triangle":
                    spawnShape = "vertical rectangle"
                elif spawnShape == "vertical rectangle":
                    spawnShape = "horizontal rectangle"
                elif spawnShape == "horizontal rectangle":
                    spawnShape = "cannonball"
                elif spawnShape == "cannonball":
                    spawnShape = "square"
            if event.key == pygame.K_c:
                objects = []
                addPolygon([960,1050],[[-1000,50],[1000,50],[1000,-50],[-1000,-50]],0,[0,0],0.5,0.5,0,0,0)
            
        if event.type == pygame.MOUSEBUTTONDOWN:
            if spawnShape == "square": spawnSquare(mousePos)
            if spawnShape == "circle": spawnCircle(mousePos)
            if spawnShape == "triangle": spawnTriangle(mousePos)
            if spawnShape == "vertical rectangle": spawnVerticalRect(mousePos)
            if spawnShape == "horizontal rectangle": spawnHorizontalRect(mousePos)
            if spawnShape == "cannonball": spawnCannonBall(mousePos)

    screen.fill((200,200,200))


    #Gravity
    for i in range(len(objects)):
        if objects[i][3] > 0:
            objects[i][4][1] += 0.1
            
    for i in range(len(objects)):
        if objects[i][0] == "Polygon":
            worldSpace = []
            for j in range(len(objects[i][2])):
                x = objects[i][1][0] + math.cos(-objects[i][7])*objects[i][2][j][0] - math.sin(-objects[i][7])*objects[i][2][j][1]
                y = objects[i][1][1] + math.sin(-objects[i][7])*objects[i][2][j][0] + math.cos(-objects[i][7])*objects[i][2][j][1]
                worldSpace.append([x,y])
            objects[i][9] = worldSpace

    for i in range(len(objects)):
        if objects[i][0] == "Circle":
            pygame.draw.circle(screen, objectcolour, objects[i][1], objects[i][2]+1)
            pygame.draw.circle(screen, (255,255,255), objects[i][1], objects[i][2]+1,3)
            pygame.draw.line(screen,(255,255,255), objects[i][1], (objects[i][1][0] + 20*math.cos(objects[i][7]), objects[i][1][1] + -20*math.sin(objects[i][7])))
        elif objects[i][0] == "Polygon":
            pygame.draw.polygon(screen, objectcolour, objects[i][9])
            pygame.draw.polygon(screen, (255,255,255), objects[i][9],3)
            pygame.draw.line(screen,(255,255,255), objects[i][1], (objects[i][1][0] + 20*math.cos(objects[i][7]), objects[i][1][1] + -20*math.sin(objects[i][7])))

    for l in range(4):
        possibleCollisions = []
        for i in range(len(objects)):
            for j in range(len(objects)):
                if (i!=j):
                    if not [j,i] in possibleCollisions:
                        possibleCollisions.append([i,j])
        
        
        for k in possibleCollisions:
            i = k[0]
            j = k[1]

            if (objects[i][3] > 0) or (objects[j][3] > 0):
                if checkCollision(objects[i],objects[j]):
                    supportPoint = []
                    normalVector = findCollisonNormal(objects[i],objects[j])
                    radius1 = vecDiff(supportPoint, objects[i][1])
                    radius2 = vecDiff(supportPoint, objects[j][1])
                    rvvector1 = crossProduct3(-objects[i][8], radius1)
                    rvvector2 = crossProduct3(objects[j][8], radius2)
                    relativeVelocity = vecSum(vecSum(vecDiff(objects[i][4], objects[j][4]), rvvector1), rvvector2)
                    dotProduct = dot(relativeVelocity, normalVector)
                    if (dotProduct > 0):
                        tangent = vecDiff(relativeVelocity,vecScale(normalVector,dot(relativeVelocity,normalVector)))
                        tangent = normalize(tangent)
                        mu = (objects[i][6] + objects[j][6]) / 2
                        #print(collisionDepth)
                        #print(relativeVelocity)
                        #print("change in x velocity ", -2*normalVector[0]*dotProduct)
                        #print("change in y velocity ", -2*normalVector[1]*dotProduct)
                        #print(normalDirection)
                        elasticity = -1-min(objects[i][5],objects[j][5])
                        impulse = [elasticity*normalVector[0]*dotProduct, elasticity*normalVector[1]*dotProduct]
                        if objects[j][3] == 0:
                            impulse[0] /= 1/objects[i][3]
                            impulse[1] /= 1/objects[i][3]
                        elif objects[i][3] == 0:
                            impulse[0] /= 1/objects[j][3]
                            impulse[1] /= 1/objects[j][3]
                        else:
                            impulse[0] /= 1/objects[i][3] + 1/objects[j][3]
                            impulse[1] /= 1/objects[i][3] + 1/objects[j][3]
                        
                        if objects[i][3] > 0:
                            objects[i][4][0] += 1/objects[i][3] * impulse[0]
                            objects[i][4][1] += 1/objects[i][3] * impulse[1]
                        if objects[j][3] > 0:
                            objects[j][4][0] -= 1/objects[j][3] * impulse[0]
                            objects[j][4][1] -= 1/objects[j][3] * impulse[1]

                        frictionImpulse = -dot(relativeVelocity,tangent)
                        if abs(frictionImpulse) > (magnitude(impulse) * mu):
                            frictionImpulse = vecScale(tangent, -magnitude(impulse) * mu)
                        else: 
                            frictionImpulse = vecScale(tangent, frictionImpulse)

                        if objects[j][3] == 0:
                            frictionImpulse[0] /= 1/objects[i][3] + crossProduct1(radius1,tangent)**2/objects[i][10]
                            frictionImpulse[1] /= 1/objects[i][3] + crossProduct1(radius1,tangent)**2/objects[i][10]
                        elif objects[i][3] == 0:
                            frictionImpulse[0] /= 1/objects[j][3] + crossProduct1(radius2,tangent)**2/objects[j][10]
                            frictionImpulse[1] /= 1/objects[j][3] + crossProduct1(radius2,tangent)**2/objects[j][10]
                        else:
                            frictionImpulse[0] /= 1/objects[i][3] + 1/objects[j][3] + crossProduct1(radius1,tangent)**2/objects[i][10] + crossProduct1(radius2,tangent)**2/objects[j][10]
                            frictionImpulse[1] /= 1/objects[i][3] + 1/objects[j][3] + crossProduct1(radius1,tangent)**2/objects[i][10] + crossProduct1(radius2,tangent)**2/objects[j][10]
                        
                        if objects[i][3] > 0:
                            objects[i][4][0] += frictionImpulse[0] / objects[i][3]
                            objects[i][4][1] += frictionImpulse[1] / objects[i][3]
                            objects[i][8] += 1 / objects[i][10] * crossProduct1(vecDiff(objects[i][1],supportPoint), impulse)
                            objects[i][8] += 1 / objects[i][10] * crossProduct1(vecDiff(objects[i][1],supportPoint), frictionImpulse)
                        if objects[j][3] > 0:
                            objects[j][4][0] -= frictionImpulse[0] / objects[j][3]
                            objects[j][4][0] -= frictionImpulse[1] / objects[j][3]
                            objects[j][8] -= 1 / objects[j][10] * crossProduct1(vecDiff(objects[j][1],supportPoint), impulse)
                            objects[j][8] -= 1 / objects[j][10] * crossProduct1(vecDiff(objects[j][1],supportPoint), frictionImpulse)
                        
                    #if l == 1:
                        #pygame.draw.circle(screen, red, supportPoint, 10)
                        #pygame.draw.line(screen, red, supportPoint, (supportPoint[0] + normalVector[0] * 20, supportPoint[1] + normalVector[1] * 20))

                    if objects[i][3] > 0:
                        positionChange = positionCorrection(objects[i],objects[j],collisionDepth,normalVector)
                        objects[i][1][0] += positionChange[0]
                        objects[i][1][1] += positionChange[1]
                    if objects[j][3] > 0:
                        positionChange = positionCorrection(objects[j],objects[i],collisionDepth,normalVector)
                        objects[j][1][0] -= positionChange[0]
                        objects[j][1][1] -= positionChange[1]
                    #print(positionChange)
      
    for i in range(len(objects)):
        if objects[i][0] == "Circle":
            objects[i][1][0] += objects[i][4][0]
            objects[i][1][1] += objects[i][4][1]
            objects[i][7] = (objects[i][7] + objects[i][8])%(2*math.pi)
        elif objects[i][0] == "Polygon":
            objects[i][1][0] += objects[i][4][0]
            objects[i][1][1] += objects[i][4][1]
            objects[i][7] = (objects[i][7] + objects[i][8])%(2*math.pi)

    display_font.render_to(screen, (200,135), ("Spawn shape: " +(str(spawnShape))), black)

    pygame.display.flip()
    clock.tick(60)