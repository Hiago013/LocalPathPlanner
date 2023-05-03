#import numpy as np
#xe, ye = 7.5, 5.5
#n_cell = 13
#x1 = np.sqrt(xe * ye / n_cell ** 2)


from OpenInstance import OpenInstance
from GridWorld import GridWorld
from dijkstra import Dijkstra
from savePath import savePath
from localPathPlanner import MiniGrid
import numpy as np
from qlAgentCursed import qlAgent as qla
import time
import matplotlib.pyplot as plt
from sys import platform
opr=platform
import cv2
import imageio
from dynamic_obstacle import dynamic_obstacle as dynobs
import rospy
from std_msgs.msg import Int64MultiArray
from trajectory_msgs.msg import JointTrajectory


def set_goal_minigrid(current_position:int, goal_position:int, minigrid_size:int):
    tuple_crrt_pstn = grid_world.s2cart(current_position)
    tuple_gl_pstn = grid_world.s2cart(goal_position)
    min_dist = 10000
    for i in range(tuple_crrt_pstn[0], tuple_crrt_pstn[0] + minigrid_size):
        for j in range(tuple_crrt_pstn[1], tuple_crrt_pstn[1] + minigrid_size):
            if abs(i - tuple_gl_pstn[0]) + abs(j - tuple_gl_pstn[1]) < min_dist:
                if grid_world.cart2s((i, j, 0)) not in mini_grid.obstacles:
                    min_dist = abs(i - tuple_gl_pstn[0]) + abs(j - tuple_gl_pstn[1])
                    goal = (i, j, 0)
    return goal

def set_goal_minigrid2(start_grid:tuple, end_grid:tuple, goal_position:int, minigrid_size:int):
    xi, yi, zi = start_grid
    xe, ye, ze = end_grid
    xg, yg, zg = goal_position
    min_dist = 10000
    for i in range(xi, xe + 1):
        for j in range(yi, ye + 1):
            if abs(i - xg) + abs(j - yg) < min_dist:
                if grid_world.cart2s((i, j, 0)) not in mini_grid.obstacles:
                    min_dist = abs(i - xg) + abs(j - yg)
                    goal = (i, j, 0)
    return goal

def set_bounds(startPosition, row, col):
    x, y, z = startPosition

    new_x, new_y, new_z = x - int(GRID_SIZE/2), y - int(GRID_SIZE/2), 0
    if GRID_SIZE % 2 == 0: #If grid size is a even number
        end_x, end_y, end_z = x + int(GRID_SIZE/2) - 1, y + int(GRID_SIZE/2) - 1, 0
    else:
        end_x, end_y, end_z = x + int(GRID_SIZE/2) - 0 , y + int(GRID_SIZE/2)-0 , 0

    if new_x < 0:
        new_x = 0
        end_x = x + GRID_SIZE - 1
    
    if new_y < 0:
        new_y = 0
        end_y = y + GRID_SIZE - 1

    if end_x > row -1:
        new_x = row - GRID_SIZE#x - (x + GRID_SIZE - row)
        end_x = row - 1

    if end_y > col -1:
        new_y = col - GRID_SIZE#y - (y + GRID_SIZE - col)
        end_y = col - 1
    return [(new_x, new_y, new_z), (end_x, end_y, end_z)]


    #mini_grid.setMiniGrid(startPos, (min(startPos[0] + GRID_SIZE - 1, row - 1), min(startPos[1] + GRID_SIZE - 1, col - 1), 0))
    #pass

def remove_dups(lista):
    """Remove os itens duplicados mantendo a ordem original."""
    return list(dict.fromkeys(lista))

def state2cartesian(state):
    y, x = divmod(state, 10)
    return x * 50, y * 50

def cartesian2state(cartesian_point):
    x, y = cartesian_point
    x = x // 50
    y = y // 50
    return 10 * x + y

def move_obstacle(obstacle_position:tuple, obstacles:list, grid_world:GridWorld):
    xo, yo, zo = obstacle_position
    possible_obstacles = []
    for i in range(xo-1, xo+2):
        for j in range(yo-1, yo+2):
            # Check if obstacle is into the environment
            if i >= 0 and j>= 0:
                if i < grid_world.rows and j < grid_world.cols:
                    # Check if obstacle chosen already there
                    # Check if the new obstacle postion is the current agent position
                    if (not grid_world.cart2s((i,j,zo)) in obstacles) and \
                    (not (i,j,zo) == grid_world.get_current_position()):
                        possible_obstacles.append(grid_world.cart2s((i,j,zo)))
    return grid_world.s2cart(np.random.choice(possible_obstacles))

#####

def visualizar(env:MiniGrid, path:list, sub_goal:list, agent_position:int, step:int):
    global SUBGOAL
    if agent_position == sub_goal[SUBGOAL]:
        print(agent_position)
        SUBGOAL += 1
    obstacle = env.obstacles
    points_obstacles = [np.array((state2cartesian(state))) for state in obstacle]
    points_global_goal = np.array((state2cartesian(env.cart2s(env.goal))),dtype=int)
    points_sub_goal = [np.array((state2cartesian(state))) for state in sub_goal]
    points_mini_grid = [np.array((state2cartesian(state))) for state in init_mini_grid]
 

    
    img = np.zeros((500, 500, 3), dtype='uint8')
##      # Desenhar elementos estaticos
    for x, y in points_obstacles:
        cv2.rectangle(img, (int(x), int(y)), (int(x) + 50, int(y) + 50), (0, 0, 255), 5)
    
    cv2.rectangle(img, (points_global_goal[0], points_global_goal[1]),\
                   ( points_global_goal[0]+ 50, points_global_goal[0]+50), (0, 255, 0), 5)
    try:
        x, y = points_sub_goal[SUBGOAL]
        x2, y2 = points_mini_grid[SUBGOAL]

        cv2.rectangle(img, (x, y), (x + 50, y + 50), (0, 150, 0), 5)
        cv2.rectangle(img, (x2, y2), (x2  + 50 * GRID_SIZE, y2 + 50 * GRID_SIZE), (0, 80, 0), 5)
    except:
        pass
     #Takes step after fixed time
    t_end = time.time() + 0# + 0 é o delay entre frame para visualização (nao para o gif)
    while time.time() < t_end:
        continue

    x, y = np.array(state2cartesian(agent_position))
    cv2.rectangle(img, (x, y), (x + 50, y+ 50), [255, 0, 0], 3)
    
    cv2.imshow('Grid_World', img)
    cv2.waitKey(1)
    cv2.imwrite(f"nova_imagem{step}.jpg", img)
    pass

def create_an_image(env:MiniGrid, sub_goal:int, agent_position:int, init_mini_grid:int, step:int):
    obstacle = env.obstacles
    points_obstacles = [np.array((state2cartesian(state))) for state in obstacle]
    points_sub_goal = np.array((state2cartesian(sub_goal)))
    points_mini_grid = np.array((state2cartesian(init_mini_grid)))
 

    
    img = np.zeros((500, 500, 3), dtype='uint8')
##      # Desenhar elementos estaticos
    for x, y in points_obstacles:
        cv2.rectangle(img, (int(x), int(y)), (int(x) + 50, int(y) + 50), (0, 0, 255), 5)

    try:
        x, y = points_sub_goal
        x2, y2 = points_mini_grid

        cv2.rectangle(img, (x, y), (x + 50, y + 50), (0, 255, 255), 5)
        cv2.rectangle(img, (x2, y2), (x2  + 50 * GRID_SIZE, y2 + 50 * GRID_SIZE), (255, 0, 255), 5)
    except:
        pass
     #Takes step after fixed time
    #t_end = time.time() + 0# + 0 é o delay entre frame para visualização (nao para o gif)
    #while time.time() < t_end:
    #    continue

    x, y = np.array(state2cartesian(agent_position))
    cv2.rectangle(img, (x, y), (x + 50, y+ 50), [255, 0, 0], 3)
    
    cv2.imshow('Grid_World', img)
    cv2.waitKey(1)

    cv2.imwrite(f"nova_imagem{step}.jpg", img)
    pass



def create_gif(len_path:int):

    # Lista de caminhos para as imagens
    image_paths = [f'nova_imagem{i}.jpg' for i in range(len_path)]

    # Cria a lista de imagens
    images = []
    for path in image_paths:
        images.append(imageio.imread(path))

    # Salva o GIF
    imageio.mimsave('animation.gif', images, duration= .5)

def local_train(mini_grid: MiniGrid, Num_Ep_local:int, startPos:tuple):
    localAgent=qla()
    localAgent.setEnviroment(mini_grid)
    localAgent.setQtable(row * col, 8)
    localAgent.setEpsilon(1, [1, .1, Num_Ep_local])
    localAgent.setAlpha(1, [0.3, .1, Num_Ep_local])
    localAgent.exploringStarts(1, 1, startPos)
    init=time.time()*1000
    localAgent.train(Num_Ep_local, 10, plot=0)
    fim=time.time()*1000-init
    print('Tempo total de treino:',fim)
    return localAgent
def callback_obs(msg):
    #Conversão de cart para s
    OptMsg=msg.data
    #Verifica mudança pós callback

#Criar Nodo
rospy.init_node('topic_publisher')
#Criar subscriber
sub = rospy.Subscriber('/OptInfo', Int64MultiArray, callback_obs)
#Criar Publisher
pub = rospy.Publisher('/GlobalPath', Int64MultiArray, queue_size=1)
GP=Int64MultiArray()



index = 12
#path = f"LARS_3DMaps\mapaLARS3D\map{index}Crescente.txt"
if opr!='linux':
    print('We are working on a Windows system')
    path = f"mapasICUAS\mapaEscolhido{index}.txt"
    
else:
    print('We are working on a Linux system')
    path = f"src/mapasICUAS/mapaEscolhido{index}.txt"
    
maps = OpenInstance(path)
header, numObstacle, obstacles = maps.run()

row = header[0]
col = header[1]
try:
    height = header[2]
except:
    height = 1

numCell = row * col * height

# kd 0 ou -2
# mapa4 -> kd 0, -2 ou -4

#inicio = time.time() * 1000
startPos = (0, 0, 0) # Posicao inicial do agente no mapa
goalPos = (row-1, col-1, 0) # Objetivo global do agente
grid_world = GridWorld(row, col, height, startPos, -0.4, -0.4, -0.4, 100)
grid_world.set_obstacles(obstacles)
grid_world.get_reward_safety()
path = []
GRID_SIZE = 5
SUBGOAL = 0
sub_goal = []
init_mini_grid = []
real_path = []

obs1 = dynobs(grid_world=grid_world, current_position=(2,2,0), static_obstacles=obstacles)
obs2 = dynobs(grid_world=grid_world, current_position=(7,7,0), static_obstacles=obstacles)
# Configurando minigrid

global OptMsg
OptMsg=[]
#Callback Verificando Obstáculos e Posição do Robô


new_image_index = 0


while startPos != goalPos:
    # Configurar Minigrid
    mini_grid = MiniGrid(row, col, height, startPos, 0, -0.4, -0.4, 100)
    mini_grid.setObstacles(grid_world.get_obstacles())
    init_grid, end_grid = set_bounds(startPos, row, col)
    print(init_grid, end_grid)
    mini_grid.setMiniGrid(init_grid, end_grid)
    mini_grid.setPosition(startPos)
    #goal_minigrid = set_goal_minigrid(grid_world.cart2s(init_grid), grid_world.cart2s(goalPos), GRID_SIZE)
    goal_minigrid = set_goal_minigrid2(init_grid, end_grid, goalPos, GRID_SIZE)
    sub_goal.append(mini_grid.cart2s(goal_minigrid))
    init_mini_grid.append(mini_grid.cart2s(init_grid))
    mini_grid.setGoal(goal_minigrid)
    mini_grid.actionSpace()
    mini_grid.get_reward_safety()

    init_mini_grid1 = mini_grid.cart2s(init_grid) # to mexendo nesse aqui

    # Treinamento Local
    Num_Ep_local = 1000
    localAgent = local_train(mini_grid, Num_Ep_local, startPos)
    #localAgent.getStats(startPos)
    localAgent.enviroment.PrintBestPath(localAgent.Q, 0, localAgent.getBestPath(startPos))
    print(localAgent.getBestPath(startPos))
    path = localAgent.getBestPath(startPos)

    print(grid_world.get_current_position())
    if new_image_index == 0:
        create_an_image(mini_grid, sub_goal=mini_grid.cart2s(goal_minigrid), \
                    agent_position=grid_world.cart2s(grid_world.get_current_position()),\
                    init_mini_grid=init_mini_grid1,
                        step = new_image_index)
        new_image_index += 1
    # Agente seguindo o caminho
    count = 0
    
    while grid_world.get_current_position() != goal_minigrid:
        # new_state, _, _= grid_world.step(localAgent.chooseBestAction(path[count]))
        # del path[0]
        # grid_world.set_current_position(new_state)
        # grid_world.onMap()
        grid_world.set_current_position(OptMsg[0])#Primeiro indice
        #count += 1
        print('\n')
       
        grid_world.set_obstacles(OptMsg[1:])
        grid_world.get_reward_safety()
        grid_world.onMap()
        mini_grid.setObstacles(grid_world.get_obstacles())
        print('\n')
        
        if len(set(grid_world.get_obstacles()+path))!=len(grid_world.get_obstacles()+path):
            #Se len for diferente tem item repitido, pois o set remove eles.
            print('ENTROOOU')
            startPos = grid_world.get_current_position()
            mini_grid = MiniGrid(row, col, height, startPos, 0, -0.4, -0.4, 100)
            mini_grid.setObstacles(grid_world.get_obstacles())
            init_grid, end_grid = set_bounds(startPos, row, col)
            mini_grid.setMiniGrid(init_grid, end_grid)
            mini_grid.setPosition(startPos)
            #goal_minigrid = set_goal_minigrid(grid_world.cart2s(init_grid), grid_world.cart2s(goalPos), GRID_SIZE)
            goal_minigrid = set_goal_minigrid2(init_grid, end_grid, goalPos, GRID_SIZE)
            sub_goal.append(mini_grid.cart2s(goal_minigrid))
            init_mini_grid.append(mini_grid.cart2s(init_grid))
            mini_grid.setGoal(goal_minigrid)
            mini_grid.actionSpace()
            mini_grid.get_reward_safety()

            Num_Ep_local = 1000
            localAgent = local_train(mini_grid, Num_Ep_local, startPos)
            localAgent.enviroment.PrintBestPath(localAgent.Q, 0, localAgent.getBestPath(startPos))
            print(localAgent.getBestPath(startPos))
            path = localAgent.getBestPath(startPos)
            count = 0
            init_mini_grid1 = mini_grid.cart2s(init_grid) # to mexendo nesse aqui
            print('SAAAIIU')
        #Publish ROS
        GP.data =path
        pub.publish(GP)
    mini_grid.setPosition(startPos)
   
SUBGOAL = 0
path = remove_dups(path)
print(path)
print(init_mini_grid)
print(real_path)
#for idx, position in enumerate(path):
#    visualizar(mini_grid, path = path, sub_goal = sub_goal, agent_position=position, step=idx)
create_gif(new_image_index)