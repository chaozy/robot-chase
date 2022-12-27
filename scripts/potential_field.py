import numpy as np

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
  if d > radius: # 0.3 is the radius of the robot
    potential = -5 * np.exp(-8 * d + 8 * radius)
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
   
                                                  