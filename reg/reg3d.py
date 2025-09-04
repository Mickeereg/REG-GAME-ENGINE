import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import pybullet as p
import math
from pathlib import Path
from PIL import Image

class Camera3D:
    def __init__(self):
        self.position = np.array([0.0, 2.0, 5.0], dtype=np.float32)
        self.front = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        self.right = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        
        self.yaw = -90.0
        self.pitch = 0.0
        self.move_speed = 0.1
        self.mouse_sensitivity = 0.1
        
    def update_vectors(self):
        front = np.array([
            math.cos(math.radians(self.yaw)) * math.cos(math.radians(self.pitch)),
            math.sin(math.radians(self.pitch)),
            math.sin(math.radians(self.yaw)) * math.cos(math.radians(self.pitch))
        ])
        self.front = front / np.linalg.norm(front)
        self.right = np.cross(self.front, np.array([0.0, 1.0, 0.0]))
        self.right = self.right / np.linalg.norm(self.right)
        self.up = np.cross(self.right, self.front)

class GameObject3D:
    def __init__(self, position=(0,0,0), size=(1,1,1), mass=1.0, color=(1,1,1)):
        self.position = np.array(position, dtype=np.float32)
        self.size = np.array(size, dtype=np.float32)
        self.color = color
        self.mass = mass
        self.physics_id = None
        self.velocity = np.zeros(3, dtype=np.float32)
        
    def init_physics(self, physics_client):
        half_extents = [s/2 for s in self.size]
        self.collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            physicsClientId=physics_client
        )
        self.physics_id = p.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=self.collision_shape,
            basePosition=self.position,
            physicsClientId=physics_client
        )
        
    def update_from_physics(self, physics_client):
        if self.physics_id is not None:
            pos, _ = p.getBasePositionAndOrientation(
                self.physics_id,
                physicsClientId=physics_client
            )
            self.position = np.array(pos)
            
    def draw(self):
        glPushMatrix()
        glTranslatef(*self.position)
        glColor3f(*self.color)
        
        # Draw cube
        glBegin(GL_QUADS)
        # Front
        glVertex3f(-self.size[0]/2, -self.size[1]/2, self.size[2]/2)
        glVertex3f(self.size[0]/2, -self.size[1]/2, self.size[2]/2)
        glVertex3f(self.size[0]/2, self.size[1]/2, self.size[2]/2)
        glVertex3f(-self.size[0]/2, self.size[1]/2, self.size[2]/2)
        # Back
        glVertex3f(-self.size[0]/2, -self.size[1]/2, -self.size[2]/2)
        glVertex3f(-self.size[0]/2, self.size[1]/2, -self.size[2]/2)
        glVertex3f(self.size[0]/2, self.size[1]/2, -self.size[2]/2)
        glVertex3f(self.size[0]/2, -self.size[1]/2, -self.size[2]/2)
        # Top
        glVertex3f(-self.size[0]/2, self.size[1]/2, -self.size[2]/2)
        glVertex3f(-self.size[0]/2, self.size[1]/2, self.size[2]/2)
        glVertex3f(self.size[0]/2, self.size[1]/2, self.size[2]/2)
        glVertex3f(self.size[0]/2, self.size[1]/2, -self.size[2]/2)
        # Bottom
        glVertex3f(-self.size[0]/2, -self.size[1]/2, -self.size[2]/2)
        glVertex3f(self.size[0]/2, -self.size[1]/2, -self.size[2]/2)
        glVertex3f(self.size[0]/2, -self.size[1]/2, self.size[2]/2)
        glVertex3f(-self.size[0]/2, -self.size[1]/2, self.size[2]/2)
        # Right
        glVertex3f(self.size[0]/2, -self.size[1]/2, -self.size[2]/2)
        glVertex3f(self.size[0]/2, self.size[1]/2, -self.size[2]/2)
        glVertex3f(self.size[0]/2, self.size[1]/2, self.size[2]/2)
        glVertex3f(self.size[0]/2, -self.size[1]/2, self.size[2]/2)
        # Left
        glVertex3f(-self.size[0]/2, -self.size[1]/2, -self.size[2]/2)
        glVertex3f(-self.size[0]/2, -self.size[1]/2, self.size[2]/2)
        glVertex3f(-self.size[0]/2, self.size[1]/2, self.size[2]/2)
        glVertex3f(-self.size[0]/2, self.size[1]/2, -self.size[2]/2)
        glEnd()
        
        glPopMatrix()

class Reg3D:
    def __init__(self, width=800, height=600, title="REG 3D"):
        pygame.init()
        self.width = width
        self.height = height
        pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
        pygame.display.set_caption(title)
        
        # Initialize 3D
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        
        # Setup lighting
        glLightfv(GL_LIGHT0, GL_POSITION, (5, 5, 5, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1.0))
        
        # Setup perspective
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, (width / height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        
        # Initialize physics
        self.physics_client = p.connect(p.DIRECT)
        p.setGravity(0, -9.81, 0)
        
        # Create ground plane
        self.ground_shape = p.createCollisionShape(
            p.GEOM_PLANE,
            physicsClientId=self.physics_client
        )
        self.ground_body = p.createMultiBody(
            0, self.ground_shape,
            physicsClientId=self.physics_client
        )
        
        # Game objects
        self.objects = []
        self.camera = Camera3D()
        
        # Update and draw callbacks
        self.update_func = None
        self.draw_func = None
        
        self.running = True
        self.clock = pygame.time.Clock()
        
    def add_object(self, obj):
        obj.init_physics(self.physics_client)
        self.objects.append(obj)
        
    def update(self, func):
        self.update_func = func
        return func
        
    def draw(self, func):
        self.draw_func = func
        return func
        
    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                    
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            self.camera.position += self.camera.front * self.camera.move_speed
        if keys[pygame.K_s]:
            self.camera.position -= self.camera.front * self.camera.move_speed
        if keys[pygame.K_a]:
            self.camera.position -= self.camera.right * self.camera.move_speed
        if keys[pygame.K_d]:
            self.camera.position += self.camera.right * self.camera.move_speed
        if keys[pygame.K_SPACE]:
            self.camera.position += self.camera.up * self.camera.move_speed
        if keys[pygame.K_LSHIFT]:
            self.camera.position -= self.camera.up * self.camera.move_speed
            
        # Mouse look
        mouse_rel = pygame.mouse.get_rel()
        self.camera.yaw += mouse_rel[0] * self.camera.mouse_sensitivity
        self.camera.pitch -= mouse_rel[1] * self.camera.mouse_sensitivity
        self.camera.pitch = max(-89.0, min(89.0, self.camera.pitch))
        self.camera.update_vectors()
        
    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Apply camera transform
        target = self.camera.position + self.camera.front
        gluLookAt(*self.camera.position, *target, *self.camera.up)
        
        # Draw ground
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5)
        glBegin(GL_QUADS)
        glVertex3f(-50, 0, -50)
        glVertex3f(-50, 0, 50)
        glVertex3f(50, 0, 50)
        glVertex3f(50, 0, -50)
        glEnd()
        glPopMatrix()
        
        # Update physics
        p.stepSimulation(physicsClientId=self.physics_client)
        
        # Update and draw objects
        for obj in self.objects:
            obj.update_from_physics(self.physics_client)
            obj.draw()
            
        # Call user draw function
        if self.draw_func:
            self.draw_func()
            
        pygame.display.flip()
        
    def loop(self):
        pygame.mouse.set_visible(False)
        pygame.event.set_grab(True)
        
        while self.running:
            self.handle_input()
            
            if self.update_func:
                self.update_func(1.0 / 60.0)  # Fixed time step
                
            self.render()
            self.clock.tick(60)
            
        pygame.quit()
        p.disconnect(physicsClientId=self.physics_client) 