import numpy as np
import pygame as pg
import time

# initialize pygame
pg.init()

# define basic things
width, height = 1280, 720
screen = pg.display.set_mode((width, height))
clock = pg.time.Clock()

# hide mouse
pg.mouse.set_visible(False)
pg.event.set_grab(True)

# rotation matrices
def rotXmat(ang):
    return np.array([
        [1, 0, 0],
        [0, np.cos(ang), -np.sin(ang)],
        [0, np.sin(ang), np.cos(ang)]
    ])
def rotYmat(ang):
    return np.array([
        [np.cos(ang), 0, np.sin(ang)],
        [0, 1, 0],
        [-np.sin(ang), 0, np.cos(ang)]
    ])

# moving matrices
def forward(yaw):
    return np.array([
        np.sin(yaw),
        0,
        np.cos(yaw)
    ])
def right(yaw):
    return np.array([
        np.cos(yaw),
        0,
        -np.sin(yaw)
    ])

# skeleton of a block
blockskl = np.array([
    [-0.5, 0, -0.5],
    [ 0.5, 0, -0.5],
    [ 0.5, 0,  0.5],
    [-0.5, 0,  0.5],
    [-0.5, 1, -0.5],
    [ 0.5, 1, -0.5],
    [ 0.5, 1,  0.5],
    [-0.5, 1,  0.5]
])

# edges of a block
blockedges = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 4],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7]
]

# class that deines a point
class Point:
    def __init__(self, world, pos):
        self.world = world
        self.pos = np.array(pos, float)
        self.screenpos = np.zeros(2)
    def get_camera_pos(self):
        npos = self.pos - self.world.camera.pos
        R = rotYmat(self.world.camera.rot[1]) @ rotXmat(self.world.camera.rot[0])
        npos = R.T @ npos
        return npos
    def get_persp_proj_pos(self):
        npos = self.get_camera_pos()
        if npos[2] <= 0: return None
        nx = (self.world.camera.f * npos[0]) / npos[2]
        ny = (self.world.camera.f * npos[1]) / npos[2]
        npos = np.array([nx, ny])
        return npos
    def get_screen_pos(self):
        npos = self.get_persp_proj_pos()
        if npos is None: return None
        u = (width / 2) + npos[0]
        v = (height / 2) - npos[1]
        npos = np.array([u, v])
        self.screenpos = npos
        return npos
    def draw(self):
        npos = self.get_screen_pos()
        if npos is None: return
        pg.draw.circle(screen, "white", npos, 3)

# class that defines a block
class Block:
    def __init__(self, world, pos):
        self.world = world
        self.pos = np.array(pos, float)
        self.pts = blockskl + self.pos
        self.pts = [Point(self.world, pt) for pt in self.pts]
    def draw(self):
        for pt in self.pts: pt.draw()
        for i1, i2 in blockedges:
            pos1 = self.pts[i1].screenpos
            pos2 = self.pts[i2].screenpos
            if pos1 is None or pos2 is None: break
            pg.draw.line(screen, "gray", pos1, pos2, 2)

# class that defines the camera
class Camera:
    def __init__(self, world):
        self.world = world
        self.pos = np.zeros(3)
        self.rot = np.zeros(3)
        self.fov = np.deg2rad(80)
        self.f = height / (2 * np.tan(self.fov / 2))
        self.sensitivity = 0.0015
        self.speed = 5
    def update(self, dt):
        vel = np.zeros(3)
        keys = pg.key.get_pressed()
        if keys[pg.K_w]: vel += forward(self.rot[1])
        if keys[pg.K_s]: vel -= forward(self.rot[1])
        if keys[pg.K_a]: vel -= right(self.rot[1])
        if keys[pg.K_d]: vel += right(self.rot[1])
        if keys[pg.K_SPACE]: vel[1] += 1
        if keys[pg.K_LSHIFT]: vel[1] -= 1
        velmag = np.linalg.norm(vel)
        if velmag != 0: vel /= velmag
        self.pos += vel * self.speed * dt
        mx, my = pg.mouse.get_rel()
        self.rot[1] += mx * self.sensitivity
        self.rot[0] += my * self.sensitivity
        self.rot[0] = np.clip(self.rot[0], -np.pi/2, np.pi/2)

# class that defines the world
class World:
    def __init__(self):
        self.camera = Camera(self)
        self.blocks = []
    def add(self, pos):
        self.blocks.append(Block(self, pos))
    def draw(self):
        for block in self.blocks: block.draw()

# time before frame end
lt = time.time()
# exit trigger
xt = False

# create the world
world = World()

# add points
world.add([1, 0, 5])
world.add([1, 1, 5])
world.add([3, 0, 2])
world.add([2, 0, 2])
world.add([1, 0, 2])

# main loop
while True:
    # calculate delta time
    dt = time.time() - lt
    # update time before frame end
    lt = time.time()
    # check for events
    for event in pg.event.get():
        if event.type == pg.QUIT: xt = True
    # exit if triggered
    if xt: break
    # update camera
    world.camera.update(dt)
    # refresh the screen
    screen.fill((0, 0, 0))
    # draw the world
    world.draw()
    # update the screen
    pg.display.flip()
    # set fps
    clock.tick(60)

# exit at the end
pg.quit()