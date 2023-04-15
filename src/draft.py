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

def set_bounds(startPosition, row, col):
    x, y, z = startPosition
    new_x, new_y, new_z = x, y, z
    end_x, end_y, end_z = x + GRID_SIZE - 1, y + GRID_SIZE-1, 0
    if x + GRID_SIZE -1 > row -1:
        new_x = x - (x + GRID_SIZE - row)
        end_x = row - 1
    if y + GRID_SIZE -1 > col -1:
        new_y = y - (y + GRID_SIZE - col)
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

#####

def visualizar(env:MiniGrid, path:list, sub_goal:list, agent_position:int, step:int):
    global SUBGOAL
    if agent_position == sub_goal[SUBGOAL]:
        print(agent_position)
        SUBGOAL += 1
    obstacle = env.obstacles
    points_obstacles = [np.array((state2cartesian(state))) for state in obstacle]
    points_global_goal = np.array((state2cartesian(env.cart2s(env.goal))))
    points_sub_goal = [np.array((state2cartesian(state))) for state in sub_goal]
    points_mini_grid = [np.array((state2cartesian(state))) for state in init_mini_grid]
  

    
    img = np.zeros((500, 500, 3), dtype='uint8')
##      # Desenhar elementos estaticos
    for point in points_obstacles:
        cv2.rectangle(img, point, point + 50, (0, 0, 255), 5)
    
    cv2.rectangle(img, points_global_goal, points_global_goal + 50, (0, 255, 0), 5)
    try:
        cv2.rectangle(img, points_sub_goal[SUBGOAL], points_sub_goal[SUBGOAL] + 50, (0, 150, 0), 5)
        cv2.rectangle(img, points_mini_grid[SUBGOAL], points_mini_grid[SUBGOAL] + 50 * GRID_SIZE, (0, 80, 0), 5)
    except:
        pass
     #Takes step after fixed time
    t_end = time.time() + 0# + 0 é o delay entre frame para visualização (nao para o gif)
    while time.time() < t_end:
        continue

    agent_point = np.array(state2cartesian(agent_position))
    cv2.rectangle(img, agent_point, agent_point + 50, [255, 0, 0], 3)
    
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
sub_goal = []
init_mini_grid = []
real_path = []

# Configurando minigrid


while startPos != goalPos:
    # Configurar Minigrid
    mini_grid = MiniGrid(row, col, height, startPos, -0.4, -0.4, -0.4, 100)
    mini_grid.setObstacles(obstacles)
    init_grid, end_grid = set_bounds(startPos, row, col)
    mini_grid.setMiniGrid(init_grid, end_grid)
    mini_grid.setPosition(startPos)
    goal_minigrid = set_goal_minigrid(grid_world.cart2s(init_grid), grid_world.cart2s(goalPos), GRID_SIZE)
    sub_goal.append(mini_grid.cart2s(goal_minigrid))
    init_mini_grid.append(mini_grid.cart2s(init_grid))
    mini_grid.setGoal(goal_minigrid)
    mini_grid.actionSpace()
    mini_grid.get_reward_safety()

    # Treinamento Local
    Num_Ep_local = 1000
    localAgent = local_train(mini_grid, Num_Ep_local, startPos)
    #localAgent.getStats(startPos)
    localAgent.enviroment.PrintBestPath(localAgent.Q, 0, localAgent.getBestPath(startPos))
    print(localAgent.getBestPath(startPos))
    path = localAgent.getBestPath(startPos)
    

    # Agente seguindo o caminho
    count = 0
    while grid_world.get_current_position() != goal_minigrid:
        new_state, _, _= grid_world.step(localAgent.chooseBestAction(path[count]))
        grid_world.set_current_position(new_state)
        grid_world.onMap()
        count += 1
        print('\n')
        if np.random.rand() > .5: # Aparece um obstaculo aleatorio 50% das vezes na frente do robo
            if path[-1] != grid_world.cart2s(new_state): # Se não estiver no ultimo ultimo estado do caminho
                count -= 1
                print('Entrou')
                startPos = grid_world.get_current_position()
                grid_world.set_obstacles(grid_world.get_obstacles() + [path[count + 2]])
                grid_world.get_reward_safety()
                grid_world.onMap()

                mini_grid = MiniGrid(row, col, height, startPos, -0.4, -0.4, -0.4, 100)
                mini_grid.setObstacles(grid_world.get_obstacles())
                init_grid, end_grid = set_bounds(startPos, row, col)
                mini_grid.setMiniGrid(init_grid, end_grid)
                mini_grid.setPosition(startPos)
                goal_minigrid = set_goal_minigrid(grid_world.cart2s(init_grid), grid_world.cart2s(goalPos), GRID_SIZE)
                sub_goal.append(mini_grid.cart2s(goal_minigrid))
                init_mini_grid.append(mini_grid.cart2s(init_grid))
                mini_grid.setGoal(goal_minigrid)
                mini_grid.actionSpace()
                mini_grid.get_reward_safety()

                Num_Ep_local = 1000
                localAgent = local_train(mini_grid, Num_Ep_local, startPos)
                #localAgent.getStats(startPos)
                localAgent.enviroment.PrintBestPath(localAgent.Q, 0, localAgent.getBestPath(startPos))
                print(localAgent.getBestPath(startPos))
                path = localAgent.getBestPath(startPos)
                count = 0
                print('Saiu')
                
        real_path.append(new_state)
        

    mini_grid.setPosition(startPos)
    startPos = goal_minigrid

SUBGOAL = 0
path = remove_dups(path)
print(path)
print(init_mini_grid)
print(real_path)
#for idx, position in enumerate(path):
#    visualizar(mini_grid, path = path, sub_goal = sub_goal, agent_position=position, step=idx)
#create_gif(len(path))
