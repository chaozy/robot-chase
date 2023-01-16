import numpy as np
import matplotlib.pylab as plt

X, Y, YAW = 0, 1, 2

def get_relative_pose(robot, target):
    relative = robot.copy()
    t1 = -robot[YAW]
    delta = target[:2] - robot[:2]
    relative[X] = np.cos(t1) * delta[X] - np.sin(t1) * delta[Y]
    relative[Y] = np.sin(t1) * delta[X] + np.cos(t1) * delta[Y]
    
    return relative

def reach_goal(pose, goal_pose, max_speed):
  v = np.copy(get_relative_pose(pose, goal_pose))[:2]
  return cap(v, max_speed=max_speed)

def avoid_obsts(pose, obsts, max_speed):
  v = np.zeros(2, dtype=np.float32)
  d, closest, radius = 1000000, None, -1
  for i in range(len(obsts)):
    relative_pose = get_relative_pose(pose, obsts[i].pose)
    tmp_d = np.linalg.norm(relative_pose)
    if tmp_d < d:
      d = tmp_d
      closest = relative_pose
      radius = obsts[i].radius
      
  angle = np.arctan2(-closest[Y], -closest[X])    
  if d > radius + 0.15: # 0.3 is the radius of the robot
    potential = 5 * np.exp(-8 * d + 8 * radius)
    v[X], v[Y] = potential * max_speed * np.cos(angle), potential * max_speed * np.sin(angle)
  else:
    v[X], v[Y] = max_speed * np.cos(angle), max_speed * np.sin(angle)
  return v

def cap(v, max_speed):
  n = np.linalg.norm(v)
  if n > max_speed:
    return v / n * max_speed
  return v

def get_velocity(pose, goal_pose, obsts, max_speed):
  v = reach_goal(pose, goal_pose, max_speed) + avoid_obsts(pose, obsts, max_speed)
  return cap(v, max_speed=max_speed)

WALL_OFFSET = 2.0
def plot_field(pose, goal_pose, obsts, max_speed):
  fig, ax = plt.subplots()
  X, Y = np.meshgrid(np.linspace(-2, 2, 30),
                     np.linspace(-2, 2, 30))
  U = np.zeros_like(X)
  V = np.zeros_like(X)
  for i in range(len(X)):
    for j in range(len(X[0])):
      velocity = get_velocity(np.array([X[i, j], Y[i, j], 0]), goal_pose, obsts, max_speed)
      # velocity = reach_goal(np.array([X[i, j], Y[i, j], 0]), goal_pose, max_speed)
      U[i, j] = velocity[0]
      V[i, j] = velocity[1]
  plt.quiver(X, Y, U, V, units='width')
  
  plt.plot([-WALL_OFFSET, WALL_OFFSET], [-WALL_OFFSET, -WALL_OFFSET], 'k')
  plt.plot([-WALL_OFFSET, WALL_OFFSET], [WALL_OFFSET, WALL_OFFSET], 'k')
  plt.plot([-WALL_OFFSET, -WALL_OFFSET], [-WALL_OFFSET, WALL_OFFSET], 'k')
  plt.plot([WALL_OFFSET, WALL_OFFSET], [-WALL_OFFSET, WALL_OFFSET], 'k')
  
  ax.add_artist(plt.Circle(np.array([0, 0]), 0.25, color='gray'))
  ax.add_artist(plt.Circle(np.array([1, 0]), 0.25, color='gray'))
  ax.add_artist(plt.Circle(np.array([-1, 0]), 0.25, color='gray'))



  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-.5 - WALL_OFFSET, WALL_OFFSET + .5])
  plt.ylim([-.5 - WALL_OFFSET, WALL_OFFSET + .5])
  plt.savefig("potential_field")
  plt.show()
   
                                                  