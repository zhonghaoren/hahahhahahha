#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#import os for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
    return count

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    total = 0
    tmp = 0
    eachDist = 0
    secondDist = 0
    maxDist = state.width + state.height + 1
    for box in state.boxes:
      if box not in state.storage:
        eachDist = maxDist
        for storage in state.storage:
          tmp = abs(storage[1] - box[1]) + abs(storage[0] - box[0])
          if eachDist > tmp:
            eachDist = tmp
        total += eachDist
    return total

def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_min_moves has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    
    total = 0
    eachDist = 0
    boxl = []
    boxN = []
    stoN = []
    numB = len(state.boxes)
    numS = len(state.storage)
    
    #check if the graph is too big
    bflag = False
    if state.width * state.height >= 64:
      bflag = True
    
    for storage in state.storage:
      if storage not in state.boxes:
        stoN.append(storage)

    for box in state.boxes:
      if box in state.storage:
        boxl.append(box)
      else:
        boxN.append(box)
    
    #check if there are some boxes in the storage that capture other boxes' 
    #nearest storage   
    for box in boxl:
      bUp = (box[0], box[1] - 1)
      bDown = (box[0], box[1] + 1)
      bLeft = (box[0] - 1, box[1])
      bRight = (box[0] + 1, box[1])
      if ((bUp in boxN and bDown in stoN) or 
         (bDown in stoN and bUp in boxN) or 
         (bLeft in stoN and bRight in boxN) or 
         (bRight in stoN and bLeft in boxN)):
        return float("inf")        

    #set a matrix to store all the possible distances
    matrix = [[0 for c in range(len(stoN))] for r in range(len(boxN))]
    
    if not bflag:
      r = 0
      for box in boxN:
        c = 0 
        #check the dead states
        if checkingDeadState(box, state.width, state.height, state.obstacles, state.robots,boxN, state.boxes):
          return float("inf")

        for storage in stoN:
          tmp = abs(storage[1] - box[1]) + abs(storage[0] - box[0])
          matrix[r][c] = tmp
          c += 1
        r += 1
      
      total += findMinDis(matrix, len(stoN), len(boxN), [], 0)
    else:
      for box in boxN:
        bUp = (box[0], box[1] - 1)
        bDown = (box[0], box[1] + 1)
        bLeft = (box[0] - 1, box[1])
        bRight = (box[0] + 1, box[1])
        #check the dead states
        if checkingDeadState(box, state.width, state.height, state.obstacles, state.robots, boxN, state.boxes):
          return float("inf")
        #when two boxes are connected, the robot is not able to move
        if ((bLeft in boxN and bRight in state.robots) or 
            (bRight in boxN and bLeft in state.robots) or
            (bUp in boxN and bDown in state.robots) or
            (bDown in boxN and bUp in state.robots)):
          return float("inf")

        for storage in state.storage:
          tmp = abs(storage[1] - box[1]) + abs(storage[0] - box[0])
          total += tmp
      
    # when the number of boxes is greater than or eual to the number of robots, we find
    # the shortest distance between each robots and the boxes. when the number
    # of boxes is less than the number of robots, we only find the most shortest distance
    if (len(state.boxes) >= len(state.robots)):
      for bot in state.robots:
        eachDist = float("inf")
        for box in state.boxes:
          tmp = abs(bot[1] - box[1]) + abs(bot[0] - box[0])
          if eachDist > tmp:
            eachDist = tmp
      total += eachDist
    else:
      lst = []
      for bot in state.robots:
        eachDist = float("inf")
        for box in state.boxes:
          tmp = abs(bot[1] - box[1]) + abs(bot[0] - box[0])
          if eachDist > tmp:
            eachDist = tmp
        lst.append(eachDist)
      total += min(lst)
      
    if len(boxN) == 0:
      return 0
    else:
      return total

def findMinDis(matrix, c, r, haveBeen, index):
  '''to find the shortest total distance from all possible distances'''
  '''INPUT: a matrix of all possible distances, colunm numbers, row numbers, the list of index has been to, index'''
  '''OUTPUT: the shortest total distance'''   
  lst = []
  if index == r:
    return 0 
  for i in range(c):
    if i not in haveBeen:
      cpy = haveBeen[:]
      cpy.append(i)
      lst.append(matrix[index][i] + findMinDis(matrix, c, r, cpy, index + 1))
  if not lst:
    return 0
  return min(lst)


def checkingDeadState(box, width, height, obstacles, robots, boxes, allboxes):
  '''to check the dead states'''
  '''INPUT: box, width, height, obstacles, robots, boxes not in storage, all boxes'''
  '''OUTPUT: True if it is in the dead state or False if it is not'''  
  #when the boxes in the corners
  if box == (0,0) or box == (0, height - 1) or box == (width - 1, 0) or box == (width - 1, height -1 ):
      return True    
  bUp = (box[0], box[1] - 1)
  bDown = (box[0], box[1] + 1)
  bLeft = (box[0] - 1, box[1])
  bRight = (box[0] + 1, box[1])
  
  #when the obstacles block the boxes
  if ((bUp in obstacles and bLeft in obstacles) or 
    (bUp in obstacles and bRight in obstacles) or 
    (bDown in obstacles and bLeft in obstacles) or
    (bDown in obstacles and bRight in obstacles)):
    return True
  
  #when the boxes block the boxes  
  if ((bLeft in boxes and bUp in boxes) or 
      (bRight in boxes and bUp in boxes) or
      (bDown in boxes and bRight in boxes) or
      (bDown in boxes and bLeft in boxes)):
    return True  
  
  #when the robots are surrounding by boxes
  for bot in robots:
    rUp = (bot[0], bot[1] - 1)
    rDown = (bot[0], bot[1] + 1)
    rLeft = (bot[0] - 1, bot[1])
    rRight = (bot[0] + 1, bot[1])
    
    if (rUp in allboxes and rDown in allboxes and rRight in allboxes and rLeft in allboxes):
        return True
    if ((bot[0] == 0 and rDown in allboxes and rRight in allboxes and rLeft in allboxes) or 
        (bot[1] == 0 and rDown in allboxes and rRight in allboxes and rUp in allboxes) or 
        (bot[0] == width - 1 and rDown in allboxes and rUp in allboxes and rLeft in allboxes) or
        (bot[1] == height - 1 and rUp in allboxes and rRight in allboxes and rLeft in allboxes)):
      return True
    
  return False



def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return (1 - weight) * sN.gval + weight * sN.hval

def weighted_astar(initail_state, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    
    se = SearchEngine('custom', 'full')
    cost = -1
    r  = None
    
    for i in range(100, -1, -25):
      tmp = i / 100
      final = se.search(initail_state, heur_fn=heur_alternate, timebound=timebound, 
              goal_fn=sokoban_goal_state, fval_function=fval_function, weight=tmp)
      if final != False:
        if (r == None or cost > final.gval):
          cost = final.gval
          r = final
    return r


if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0,40): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    final = se.search(s0, sokoban_goal_state, heur_displaced, timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0,40):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    final = weighted_astar(s0, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 


