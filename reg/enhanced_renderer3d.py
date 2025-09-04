import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
from PIL import Image
import pybullet as p
import pyrr
import math
from pathlib import Path
from ai_system import AIAgent, AIState

class Camera:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 5.0], dtype=np.float32)
        self.front = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        self.right = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        
        self.yaw = -90.0
        self.pitch = 0.0
        self.move_speed = 0.1
        self.mouse_sensitivity = 0.1
        
    def update_camera_vectors(self):
        front = np.array([
            math.cos(math.radians(self.yaw)) * math.cos(math.radians(self.pitch)),
            math.sin(math.radians(self.pitch)),
            math.sin(math.radians(self.yaw)) * math.cos(math.radians(self.pitch))
        ])                                                                                                                                                                                                                          
        
        self.front = front / np.linalg.norm(front)
        self.right = np.cross(self.front, np.array([0.0, 1.0, 0.0]))
        self.right = self.right / np.linalg.norm(self.right)
        self.up = np.cross(self.right, self.front)
        
    def get_view_matrix(self):
        target = self.position + self.front
        return pyrr.matrix44.create_look_at(self.position, target, self.up)

class PhysicsObject:
    def __init__(self, physics_client, mass, position):
        self.physics_client = physics_client
        self.mass = mass
        self.position = np.array(position, dtype=np.float32)
        self.health = 100
        
        # Create collision shape and body
        self.collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        self.body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=self.collision_shape,
            basePosition=position
        )
        
    def get_position(self):
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        self.position = np.array(pos)
        return pos

class EnhancedRenderer3D:
    def __init__(self, width=1024, height=768):
        pygame.init()
        self.width = width
        self.height = height
        pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
        pygame.mouse.set_visible(False)
        pygame.event.set_grab(True)
        
        # Initialize camera
        self.camera = Camera()
        
        # Initialize physics
        self.physics_client = p.connect(p.DIRECT)
        p.setGravity(0, -9.81, 0)
        
        # Setup OpenGL
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Advanced lighting setup
        glLightfv(GL_LIGHT0, GL_POSITION, (5, 5, 5, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))
        
        # Setup perspective
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, (width / height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        
        # Load textures
        self.textures = {}
        self.load_texture("crate", "assets/crate.jpg")
        
        # Create physics objects
        self.physics_objects = [
            PhysicsObject(self.physics_client, 1.0, [0, 5, 0]),
            PhysicsObject(self.physics_client, 1.0, [1, 7, 0]),
            PhysicsObject(self.physics_client, 1.0, [-1, 6, 0])
        ]
        
        # Create AI agents with groups
        group1 = []
        group2 = []
        
        agent1 = AIAgent(self.physics_client, [3, 0, 3], target=self.physics_objects[0], group=group1)
        agent2 = AIAgent(self.physics_client, [-3, 0, -3], target=self.physics_objects[1], group=group1)
        agent3 = AIAgent(self.physics_client, [3, 0, -3], target=self.physics_objects[2], group=group2)
        agent4 = AIAgent(self.physics_client, [-3, 0, 3], target=self.physics_objects[0], group=group2)
        
        group1.extend([agent1, agent2])
        group2.extend([agent3, agent4])
        
        self.ai_agents = [agent1, agent2, agent3, agent4]
        
        # Create obstacles
        self.obstacles = []
        self.create_obstacles()
        
        # Create ground
        self.ground_shape = p.createCollisionShape(p.GEOM_PLANE)
        self.ground_body = p.createMultiBody(0, self.ground_shape)
        
        # Initialize particle system
        self.particles = []
        
    def load_texture(self, name, path):
        if not Path(path).exists():
            print(f"Warning: Texture {path} not found. Creating placeholder texture.")
            # Create a placeholder texture
            img_data = np.zeros((64, 64, 3), dtype=np.uint8)
            img_data[::2, ::2] = [255, 0, 0]  # Red checkerboard pattern
            img_data[1::2, 1::2] = [255, 0, 0]
        else:
            img = Image.open(path)
            img_data = np.array(img)
        
        texture_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture_id)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_data.shape[1], img_data.shape[0], 
                     0, GL_RGB, GL_UNSIGNED_BYTE, img_data)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        self.textures[name] = texture_id
        
    def draw_textured_cube(self, position):
        glPushMatrix()
        glTranslatef(*position)
        
        glBindTexture(GL_TEXTURE_2D, self.textures["crate"])
        
        # Define vertices, texture coordinates, and normals
        vertices = [
            # Front face
            (-0.5, -0.5,  0.5), (0.5, -0.5,  0.5), (0.5,  0.5,  0.5), (-0.5,  0.5,  0.5),
            # Back face
            (-0.5, -0.5, -0.5), (-0.5,  0.5, -0.5), (0.5,  0.5, -0.5), (0.5, -0.5, -0.5),
            # Top face
            (-0.5,  0.5, -0.5), (-0.5,  0.5,  0.5), (0.5,  0.5,  0.5), (0.5,  0.5, -0.5),
            # Bottom face
            (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, -0.5,  0.5), (-0.5, -0.5,  0.5),
            # Right face
            (0.5, -0.5, -0.5), (0.5,  0.5, -0.5), (0.5,  0.5,  0.5), (0.5, -0.5,  0.5),
            # Left face
            (-0.5, -0.5, -0.5), (-0.5, -0.5,  0.5), (-0.5,  0.5,  0.5), (-0.5,  0.5, -0.5),
        ]
        
        tex_coords = [(0, 0), (1, 0), (1, 1), (0, 1)] * 6
        
        glBegin(GL_QUADS)
        for i in range(24):
            glTexCoord2f(*tex_coords[i])
            glVertex3f(*vertices[i])
        glEnd()
        
        glPopMatrix()
        
    def draw_ground(self):
        glPushMatrix()
        glTranslatef(0, -2, 0)
        
        glBegin(GL_QUADS)
        glColor3f(0.5, 0.5, 0.5)  # Gray color
        glVertex3f(-10, 0, -10)
        glVertex3f(-10, 0, 10)
        glVertex3f(10, 0, 10)
        glVertex3f(10, 0, -10)
        glEnd()
        
        glPopMatrix()
        
    def draw_ai_agent(self, agent):
        glPushMatrix()
        position = agent.get_position()
        glTranslatef(*position)
        
        # Draw the AI agent as a colored cube
        color = agent.get_color()
        glColor3f(*color)
        
        # Define vertices for the agent cube (smaller than physics objects)
        vertices = [
            # Front face
            (-0.3, -0.3,  0.3), (0.3, -0.3,  0.3), (0.3,  0.3,  0.3), (-0.3,  0.3,  0.3),
            # Back face
            (-0.3, -0.3, -0.3), (-0.3,  0.3, -0.3), (0.3,  0.3, -0.3), (0.3, -0.3, -0.3),
            # Top face
            (-0.3,  0.3, -0.3), (-0.3,  0.3,  0.3), (0.3,  0.3,  0.3), (0.3,  0.3, -0.3),
            # Bottom face
            (-0.3, -0.3, -0.3), (0.3, -0.3, -0.3), (0.3, -0.3,  0.3), (-0.3, -0.3,  0.3),
            # Right face
            (0.3, -0.3, -0.3), (0.3,  0.3, -0.3), (0.3,  0.3,  0.3), (0.3, -0.3,  0.3),
            # Left face
            (-0.3, -0.3, -0.3), (-0.3, -0.3,  0.3), (-0.3,  0.3,  0.3), (-0.3,  0.3, -0.3),
        ]
        
        glBegin(GL_QUADS)
        for vertex in vertices:
            glVertex3f(*vertex)
        glEnd()
        
        glPopMatrix()
        
    def create_obstacles(self):
        obstacle_positions = [
            [2, 0, 2],
            [-2, 0, -2],
            [2, 0, -2],
            [-2, 0, 2]
        ]
        
        for pos in obstacle_positions:
            collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.0, 0.5])
            body_id = p.createMultiBody(
                baseMass=0,  # Static obstacle
                baseCollisionShapeIndex=collision_shape,
                basePosition=pos
            )
            self.obstacles.append(body_id)
            
    def add_particle(self, position, color, lifetime=1.0):
        self.particles.append({
            'position': np.array(position),
            'velocity': np.random.randn(3) * 0.1,
            'color': color,
            'lifetime': lifetime,
            'age': 0
        })
        
    def update_particles(self, delta_time):
        # Update particle positions and remove dead particles
        self.particles = [p for p in self.particles if p['age'] < p['lifetime']]
        
        for particle in self.particles:
            particle['position'] += particle['velocity'] * delta_time
            particle['velocity'][1] -= 9.81 * delta_time  # Gravity
            particle['age'] += delta_time
            
    def draw_particles(self):
        glPointSize(3.0)
        glBegin(GL_POINTS)
        
        for particle in self.particles:
            alpha = 1.0 - (particle['age'] / particle['lifetime'])
            glColor4f(*particle['color'], alpha)
            glVertex3f(*particle['position'])
            
        glEnd()
        
    def draw_health_bar(self, position, health, offset_y=1.0):
        if health <= 0:
            return
            
        glPushMatrix()
        glTranslatef(*position)
        
        # Make health bar face camera
        modelview = glGetFloatv(GL_MODELVIEW_MATRIX)
        glLoadIdentity()
        glTranslatef(position[0], position[1] + offset_y, position[2])
        glRotatef(-math.degrees(math.atan2(modelview[2][0], modelview[0][0])), 0, 1, 0)
        
        # Draw health bar background
        glColor3f(1.0, 0.0, 0.0)
        glBegin(GL_QUADS)
        glVertex3f(-0.5, 0, 0)
        glVertex3f(0.5, 0, 0)
        glVertex3f(0.5, 0.1, 0)
        glVertex3f(-0.5, 0.1, 0)
        glEnd()
        
        # Draw health bar fill
        glColor3f(0.0, 1.0, 0.0)
        health_width = (health / 100.0) - 0.5
        glBegin(GL_QUADS)
        glVertex3f(-0.5, 0, 0)
        glVertex3f(health_width, 0, 0)
        glVertex3f(health_width, 0.1, 0)
        glVertex3f(-0.5, 0.1, 0)
        glEnd()
        
        glPopMatrix()
        
    def draw_obstacle(self, body_id):
        pos, orn = p.getBasePositionAndOrientation(body_id)
        
        glPushMatrix()
        glTranslatef(*pos)
        
        # Draw obstacle as a tall box
        glColor3f(0.6, 0.6, 0.6)  # Gray color
        
        vertices = [
            # Front face
            (-0.5, -1.0,  0.5), (0.5, -1.0,  0.5), (0.5,  1.0,  0.5), (-0.5,  1.0,  0.5),
            # Back face
            (-0.5, -1.0, -0.5), (-0.5,  1.0, -0.5), (0.5,  1.0, -0.5), (0.5, -1.0, -0.5),
            # Top face
            (-0.5,  1.0, -0.5), (-0.5,  1.0,  0.5), (0.5,  1.0,  0.5), (0.5,  1.0, -0.5),
            # Bottom face
            (-0.5, -1.0, -0.5), (0.5, -1.0, -0.5), (0.5, -1.0,  0.5), (-0.5, -1.0,  0.5),
            # Right face
            (0.5, -1.0, -0.5), (0.5,  1.0, -0.5), (0.5,  1.0,  0.5), (0.5, -1.0,  0.5),
            # Left face
            (-0.5, -1.0, -0.5), (-0.5, -1.0,  0.5), (-0.5,  1.0,  0.5), (-0.5,  1.0, -0.5),
        ]
        
        glBegin(GL_QUADS)
        for vertex in vertices:
            glVertex3f(*vertex)
        glEnd()
        
        glPopMatrix()
        
    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_r:  # Reset physics objects
                    for i, obj in enumerate(self.physics_objects):
                        p.resetBasePositionAndOrientation(
                            obj.body_id,
                            [i-1, 5, 0],
                            p.getQuaternionFromEuler([0, 0, 0])
                        )
                elif event.key == pygame.K_h:  # Heal all objects
                    for obj in self.physics_objects:
                        obj.health = 100
                    for agent in self.ai_agents:
                        agent.health = 100
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click - damage nearest object
                    self._damage_nearest_object(20)
                elif event.button == 3:  # Right click - heal nearest object
                    self._heal_nearest_object(20)
            elif event.type == pygame.MOUSEMOTION:
                x_offset, y_offset = event.rel
                self.camera.yaw += x_offset * self.camera.mouse_sensitivity
                self.camera.pitch -= y_offset * self.camera.mouse_sensitivity
                self.camera.pitch = max(-89.0, min(89.0, self.camera.pitch))
                self.camera.update_camera_vectors()
        
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
            self.camera.position[1] += self.camera.move_speed
        if keys[pygame.K_LSHIFT]:
            self.camera.position[1] -= self.camera.move_speed
            
        return True
        
    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Apply camera transformation
        view_matrix = self.camera.get_view_matrix()
        glMultMatrixf(view_matrix)
        
        # Enable blending for particles
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        # Update physics and AI
        p.stepSimulation()
        delta_time = 1.0 / 60.0
        
        for agent in self.ai_agents:
            agent.update(delta_time)
            # Add particles based on AI state
            if agent.state in [AIState.ATTACK, AIState.FLEE]:
                self.add_particle(
                    agent.get_position(),
                    (1.0, 0.0, 0.0) if agent.state == AIState.ATTACK else (1.0, 0.5, 0.0),
                    0.5
                )
                
        # Update and draw particles
        self.update_particles(delta_time)
        self.draw_particles()
        
        # Draw ground
        self.draw_ground()
        
        # Draw obstacles
        for obstacle_id in self.obstacles:
            self.draw_obstacle(obstacle_id)
        
        # Draw physics objects with health bars
        for obj in self.physics_objects:
            position = obj.get_position()
            self.draw_textured_cube(position)
            self.draw_health_bar(position, obj.health)
        
        # Draw AI agents with health bars
        for agent in self.ai_agents:
            self.draw_ai_agent(agent)
            self.draw_health_bar(agent.get_position(), agent.health, 0.5)
        
        glDisable(GL_BLEND)
        pygame.display.flip()
        
    def run(self):
        running = True
        clock = pygame.time.Clock()
        
        while running:
            running = self.handle_input()
            self.render()
            clock.tick(60)
        
        p.disconnect()
        pygame.quit()

    def _find_nearest_object(self, max_distance=10.0):
        # Convert mouse position to ray
        mx, my = pygame.mouse.get_pos()
        mx = (mx / self.width) * 2 - 1
        my = 1 - (my / self.height) * 2
        
        # Get ray from camera
        ray_start = self.camera.position
        ray_end = ray_start + self.camera.front * max_distance
        
        # Check physics objects and AI agents
        nearest_obj = None
        min_dist = float('inf')
        
        all_objects = [(obj, obj.body_id) for obj in self.physics_objects]
        all_objects.extend([(agent, agent.body_id) for agent in self.ai_agents])
        
        for obj, body_id in all_objects:
            result = p.rayTest(ray_start, ray_end)[0]
            if result[0] == body_id:
                hit_pos = result[3]
                dist = np.linalg.norm(np.array(hit_pos) - ray_start)
                if dist < min_dist:
                    min_dist = dist
                    nearest_obj = obj
                    
        return nearest_obj
        
    def _damage_nearest_object(self, damage):
        obj = self._find_nearest_object()
        if obj and hasattr(obj, 'health'):
            obj.health = max(0, obj.health - damage)
            self.add_particle(
                obj.get_position(),
                (1.0, 0.0, 0.0),
                0.5
            )
            
    def _heal_nearest_object(self, amount):
        obj = self._find_nearest_object()
        if obj and hasattr(obj, 'health'):
            obj.health = min(100, obj.health + amount)
            self.add_particle(
                obj.get_position(),
                (0.0, 1.0, 0.0),
                0.5
            )

if __name__ == "__main__":
    renderer = EnhancedRenderer3D()
    renderer.run() 