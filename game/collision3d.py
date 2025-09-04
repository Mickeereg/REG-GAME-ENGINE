import sys
import os
import numpy as np
import math
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
import pybullet as p
from PIL import Image

# Add the parent directory to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from reg.reg3d import Reg3D, GameObject3D

# Initialize the 3D engine
reg = Reg3D(title="3D Collision Game")

# Enable lighting
glEnable(GL_LIGHTING)
glEnable(GL_LIGHT0)
glEnable(GL_COLOR_MATERIAL)
glEnable(GL_DEPTH_TEST)
glEnable(GL_TEXTURE_2D)

# Set up lighting
glLightfv(GL_LIGHT0, GL_POSITION, (0, 10, 0, 1))
glLightfv(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1))
glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1))
glLightfv(GL_LIGHT0, GL_SPECULAR, (1, 1, 1, 1))

# Create an invisible player
class InvisiblePlayer(GameObject3D):
    def draw(self):
        pass  # Don't draw anything

player = InvisiblePlayer(
    position=(0, 1.7, 0),  # Start at eye level
    size=(0.5, 0.5, 0.5),  # Collision size
    mass=1.0,
    color=(0.0, 0.5, 1.0)  # Color doesn't matter since it won't be drawn
)
reg.add_object(player)

# Create moving cars
class Car(GameObject3D):
    def __init__(self, position, size, color, speed, path):
        super().__init__(position=position, size=size, mass=1.0, color=color)
        self.speed = speed
        self.path = path
        self.current_path_index = 0
        self.target_position = path[0]
        
    def update(self, dt):
        # Calculate direction to target
        direction = np.array(self.target_position) - np.array(self.position)
        distance = np.linalg.norm(direction)
        
        if distance < 0.5:  # If close to target, move to next point
            self.current_path_index = (self.current_path_index + 1) % len(self.path)
            self.target_position = self.path[self.current_path_index]
            direction = np.array(self.target_position) - np.array(self.position)
            distance = np.linalg.norm(direction)
        
        if distance > 0.1:  # If not at target, move towards it
            direction = direction / distance
            force = direction * self.speed
            p.applyExternalForce(
                self.physics_id,
                -1,
                force.tolist(),
                [0, 0, 0],
                flags=p.WORLD_FRAME,
                physicsClientId=reg.physics_client
            )

# Create ground (street)
ground = GameObject3D(
    position=(0, -0.5, 0),
    size=(100, 1, 100),
    mass=0,
    color=(0.2, 0.2, 0.2)  # Dark gray for asphalt
)
reg.add_object(ground)

# Create city buildings and urban elements
city_elements = [
    # Tall office building
    GameObject3D(position=(10, 15, -10), size=(8, 30, 8), mass=0, color=(0.7, 0.7, 0.8)),  # Main building
    GameObject3D(position=(10, 30.5, -10), size=(6, 1, 6), mass=0, color=(0.6, 0.6, 0.7)),  # Roof structure
    
    # Modern glass skyscraper
    GameObject3D(position=(-15, 20, 15), size=(10, 40, 10), mass=0, color=(0.4, 0.6, 0.8)),  # Glass building
    
    # Apartment complex
    GameObject3D(position=(20, 10, 20), size=(15, 20, 8), mass=0, color=(0.8, 0.7, 0.6)),  # Main building
    GameObject3D(position=(20, 20.5, 20), size=(16, 1, 9), mass=0, color=(0.7, 0.6, 0.5)),  # Roof
    
    # Small buildings and shops
    GameObject3D(position=(-8, 3, -8), size=(6, 6, 6), mass=0, color=(0.9, 0.8, 0.7)),
    GameObject3D(position=(5, 2.5, 5), size=(4, 5, 4), mass=0, color=(0.8, 0.8, 0.7)),
    GameObject3D(position=(-20, 4, -5), size=(8, 8, 8), mass=0, color=(0.75, 0.75, 0.8)),
    
    # City infrastructure
    # Street lamps
    GameObject3D(position=(5, 2, -5), size=(0.2, 4, 0.2), mass=0, color=(0.3, 0.3, 0.3)),
    GameObject3D(position=(5, 4, -5), size=(0.5, 0.2, 0.5), mass=0, color=(1.0, 1.0, 0.7)),
    GameObject3D(position=(-5, 2, 5), size=(0.2, 4, 0.2), mass=0, color=(0.3, 0.3, 0.3)),
    GameObject3D(position=(-5, 4, 5), size=(0.5, 0.2, 0.5), mass=0, color=(1.0, 1.0, 0.7)),
    
    # Bus stops
    GameObject3D(position=(8, 1.5, 0), size=(3, 3, 1), mass=0, color=(0.4, 0.4, 0.4)),
    GameObject3D(position=(-8, 1.5, 0), size=(3, 3, 1), mass=0, color=(0.4, 0.4, 0.4)),
    
    # Barriers and walls
    GameObject3D(position=(15, 1, -15), size=(0.3, 2, 10), mass=0, color=(0.6, 0.6, 0.6)),
    GameObject3D(position=(-15, 1, 15), size=(10, 2, 0.3), mass=0, color=(0.6, 0.6, 0.6)),
    
    # Parking structures
    GameObject3D(position=(25, 3, 0), size=(10, 6, 20), mass=0, color=(0.5, 0.5, 0.5)),
    GameObject3D(position=(25, 9, 0), size=(10, 6, 20), mass=0, color=(0.5, 0.5, 0.5)),
    
    # Park elements
    GameObject3D(position=(0, 0.5, -20), size=(15, 0.2, 15), mass=0, color=(0.2, 0.5, 0.2)),  # Grass
    GameObject3D(position=(0, 1.5, -20), size=(1, 3, 1), mass=0, color=(0.3, 0.2, 0.1)),  # Tree trunk
    GameObject3D(position=(0, 4, -20), size=(3, 2, 3), mass=0, color=(0.1, 0.4, 0.1)),  # Tree top
    
    # Construction site
    GameObject3D(position=(-25, 5, -25), size=(8, 10, 8), mass=0, color=(0.8, 0.6, 0.3)),  # Unfinished building
    GameObject3D(position=(-25, 10.5, -25), size=(0.5, 15, 0.5), mass=0, color=(0.7, 0.1, 0.1)),  # Crane
]

for element in city_elements:
    reg.add_object(element)

# Add some road markings (white lines on the ground)
road_markings = [
    GameObject3D(position=(0, -0.45, 0), size=(0.3, 0.05, 20), mass=0, color=(1.0, 1.0, 1.0)),  # Center line
    GameObject3D(position=(10, -0.45, 0), size=(0.3, 0.05, 20), mass=0, color=(1.0, 1.0, 1.0)),  # Side line
    GameObject3D(position=(-10, -0.45, 0), size=(0.3, 0.05, 20), mass=0, color=(1.0, 1.0, 1.0)),  # Side line
]

for marking in road_markings:
    reg.add_object(marking)

# Create cars with AI
cars = []
# Define car paths (circular routes)
car_paths = [
    [(20, 0.5, 0), (20, 0.5, 20), (0, 0.5, 20), (0, 0.5, 0)],  # Square path
    [(-20, 0.5, 0), (-20, 0.5, -20), (0, 0.5, -20), (0, 0.5, 0)],  # Square path
    [(10, 0.5, 10), (-10, 0.5, 10), (-10, 0.5, -10), (10, 0.5, -10)],  # Square path
]

# Create cars with different colors and paths
car_colors = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)]  # Red, Blue, Green
for i, path in enumerate(car_paths):
    car = Car(
        position=path[0],
        size=(2, 1, 4),  # Car size
        color=car_colors[i],
        speed=500,  # Car speed
        path=path
    )
    reg.add_object(car)
    cars.append(car)

# Physics variables
JUMP_FORCE = 800
MOVE_SPEED = 1000  # Increased movement speed
AIR_CONTROL = 0.5
GROUND_FRICTION = 0.8
AIR_RESISTANCE = 0.95

# Camera settings
CAMERA_DISTANCE = 0.0  # First person view
CAMERA_HEIGHT = 0.25   # Camera at center of collision box
CAMERA_SMOOTHNESS = 0.1  # More responsive camera for FPS

# Character rotation
character_rotation = 0.0
mouse_sensitivity = 0.2  # Mouse sensitivity for looking around

# Sky color - City daytime sky
glClearColor(0.6, 0.8, 1.0, 1.0)  # Light blue sky

def update_camera():
    global character_rotation
    # Get player position
    player_pos = player.position
    
    # Calculate target camera position (at center of collision box)
    target_pos = np.array([
        player_pos[0],
        player_pos[1] + CAMERA_HEIGHT,
        player_pos[2]
    ])
    
    # Set camera position to player's center
    reg.camera.position = target_pos
    
    # Calculate camera direction based on player rotation
    reg.camera.front = np.array([
        -math.sin(math.radians(character_rotation)),
        0,
        -math.cos(math.radians(character_rotation))
    ])
    reg.camera.front = reg.camera.front / np.linalg.norm(reg.camera.front)
    reg.camera.update_vectors()

@reg.update
def update(dt):
    global character_rotation
    keys = pygame.key.get_pressed()
    
    # Get current velocity
    vel, _ = p.getBaseVelocity(player.physics_id, physicsClientId=reg.physics_client)
    
    # Check if player is on ground
    contacts = p.getContactPoints(
        physicsClientId=reg.physics_client,
        bodyA=player.physics_id
    )
    is_grounded = False
    for contact in contacts:
        contact_normal = contact[7]
        if contact_normal[1] > 0.7:
            is_grounded = True
            break
    
    # Calculate forward and right vectors based on camera rotation
    forward = np.array([
        -math.sin(math.radians(character_rotation)),
        0,
        -math.cos(math.radians(character_rotation))
    ])
    
    right = np.array([
        math.cos(math.radians(character_rotation)),
        0,
        -math.sin(math.radians(character_rotation))
    ])
    
    # Initialize movement direction
    move_direction = np.zeros(3)
    
    # Add movement based on input
    if keys[pygame.K_w]:  # Forward
        move_direction += forward
    if keys[pygame.K_s]:  # Backward
        move_direction -= forward
    if keys[pygame.K_d]:  # Right
        move_direction += right
    if keys[pygame.K_a]:  # Left
        move_direction -= right
    
    # Normalize movement direction if not zero
    if np.any(move_direction):
        move_direction = move_direction / np.linalg.norm(move_direction)
        
    # Calculate movement force
    move_force = move_direction * MOVE_SPEED
    
    # Apply movement force
    if np.any(move_force):
        p.applyExternalForce(
            player.physics_id,
            -1,
            move_force.tolist(),
            [0, 0, 0],
            flags=p.WORLD_FRAME,
            physicsClientId=reg.physics_client
        )
    
    # Handle rotation with mouse
    mouse_rel = pygame.mouse.get_rel()
    character_rotation -= mouse_rel[0] * mouse_sensitivity  # Mouse sensitivity
    
    # Apply friction
    if is_grounded:
        vel = [v * GROUND_FRICTION for v in vel]
    else:
        vel = [v * AIR_RESISTANCE for v in vel]
    
    # Update velocity
    p.resetBaseVelocity(
        player.physics_id,
        vel,
        [0, 0, 0],
        physicsClientId=reg.physics_client
    )
    
    # Jumping
    if keys[pygame.K_SPACE] and is_grounded:
        jump_force = [0, JUMP_FORCE, 0]
        p.applyExternalForce(
            player.physics_id,
            -1,
            jump_force,
            [0, 0, 0],
            flags=p.WORLD_FRAME,
            physicsClientId=reg.physics_client
        )
    
    # Update camera position
    update_camera()
    
    # Update cars
    for car in cars:
        car.update(dt)

@reg.draw
def draw():
    # Draw a simple sun
    glPushMatrix()
    glTranslatef(0, 20, 0)
    glColor3f(1.0, 0.9, 0.0)
    glBegin(GL_POLYGON)
    for i in range(32):
        angle = 2 * math.pi * i / 32
        x = math.cos(angle)
        y = math.sin(angle)
        glVertex3f(x, y, 0)
    glEnd()
    glPopMatrix()

if __name__ == "__main__":
    reg.loop() 