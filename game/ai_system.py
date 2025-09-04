import numpy as np
from enum import Enum
import random
import pybullet as p
from pygame import mixer
import math
from pathlib import Path
import time

class AIState(Enum):
    IDLE = 1
    PATROL = 2
    CHASE = 3
    AVOID = 4
    ATTACK = 5
    FLEE = 6
    GROUP = 7

class BehaviorNode:
    def __init__(self):
        self.parent = None
        self.children = []
        
    def add_child(self, child):
        child.parent = self
        self.children.append(child)
        
    def run(self, ai_agent):
        pass

class SequenceNode(BehaviorNode):
    def run(self, ai_agent):
        for child in self.children:
            if not child.run(ai_agent):
                return False
        return True

class SelectorNode(BehaviorNode):
    def run(self, ai_agent):
        for child in self.children:
            if child.run(ai_agent):
                return True
        return False

class ActionNode(BehaviorNode):
    def __init__(self, action_func):
        super().__init__()
        self.action_func = action_func
        
    def run(self, ai_agent):
        return self.action_func()

class AIAgent:
    def __init__(self, physics_client, position, target=None, group=None):
        self.physics_client = physics_client
        self.position = np.array(position, dtype=np.float32)
        self.target = target
        self.group = group if group is not None else []
        self.state = AIState.IDLE
        self.speed = 0.05
        self.detection_radius = 5.0
        self.avoid_radius = 2.0
        self.attack_radius = 1.0
        self.health = 100
        self.damage = 10
        self.last_attack_time = 0
        self.attack_cooldown = 2.0  # seconds
        self.behavior_tree = self._create_behavior_tree()
        
        # Create physics body for AI agent
        self.collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.3, 0.3, 0.3]
        )
        self.body_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=self.collision_shape,
            basePosition=position
        )
        
        # Patrol points with height variation
        self.patrol_points = [
            np.array([3, 1, 3]),
            np.array([-3, 0, 3]),
            np.array([3, 2, -3]),
            np.array([-3, 1, -3])
        ]
        self.current_patrol_index = 0
        
        # Initialize sound effects
        mixer.init()
        self.sounds = {
            'attack': mixer.Sound('assets/attack.wav') if Path('assets/attack.wav').exists() else None,
            'chase': mixer.Sound('assets/chase.wav') if Path('assets/chase.wav').exists() else None,
            'flee': mixer.Sound('assets/flee.wav') if Path('assets/flee.wav').exists() else None
        }
        self.last_sound_time = 0
        self.sound_cooldown = 1.0  # seconds
        
    def _create_behavior_tree(self):
        root = SelectorNode()
        
        # Flee sequence (highest priority)
        flee_sequence = SequenceNode()
        flee_sequence.add_child(ActionNode(self._should_flee))
        flee_sequence.add_child(ActionNode(self._flee))
        
        # Attack sequence
        attack_sequence = SequenceNode()
        attack_sequence.add_child(ActionNode(self._can_attack))
        attack_sequence.add_child(ActionNode(self._attack))
        
        # Chase sequence
        chase_sequence = SequenceNode()
        chase_sequence.add_child(ActionNode(self._check_target_nearby))
        chase_sequence.add_child(ActionNode(self._chase_target))
        
        # Group behavior sequence
        group_sequence = SequenceNode()
        group_sequence.add_child(ActionNode(self._should_group))
        group_sequence.add_child(ActionNode(self._group_behavior))
        
        # Patrol sequence
        patrol_sequence = SequenceNode()
        patrol_sequence.add_child(ActionNode(self._should_patrol))
        patrol_sequence.add_child(ActionNode(self._patrol))
        
        # Add all sequences to root
        root.add_child(flee_sequence)
        root.add_child(attack_sequence)
        root.add_child(chase_sequence)
        root.add_child(group_sequence)
        root.add_child(patrol_sequence)
        root.add_child(ActionNode(self._idle))
        
        return root
        
    def update(self, delta_time):
        self.behavior_tree.run(self)
        self._update_physics_position()
        
    def _update_physics_position(self):
        current_pos, current_orn = p.getBasePositionAndOrientation(self.body_id)
        self.position = np.array(current_pos)
        
    def _check_target_nearby(self):
        if self.target is None:
            return False
        distance = np.linalg.norm(np.array(self.target.get_position()) - self.position)
        return distance < self.detection_radius
        
    def _chase_target(self):
        if self.target is None:
            return False
        self.state = AIState.CHASE
        self.play_sound('chase')
        target_pos = np.array(self.target.get_position())
        direction = target_pos - self.position
        if np.linalg.norm(direction) < 0.001:  # Avoid division by zero
            return True
        direction = direction / np.linalg.norm(direction)
        new_position = self.position + direction * self.speed
        new_position = self._avoid_obstacles(new_position)
        p.resetBasePositionAndOrientation(
            self.body_id,
            new_position.tolist(),
            p.getQuaternionFromEuler([0, 0, 0])
        )
        return True
        
    def _should_patrol(self):
        return self.state in [AIState.IDLE, AIState.PATROL]
        
    def _patrol(self):
        self.state = AIState.PATROL
        target_point = self.patrol_points[self.current_patrol_index]
        direction = target_point - self.position
        distance = np.linalg.norm(direction)
        
        if distance < 0.1:
            self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
            return True
            
        direction = direction / distance
        new_position = self.position + direction * self.speed
        p.resetBasePositionAndOrientation(
            self.body_id,
            new_position.tolist(),
            p.getQuaternionFromEuler([0, 0, 0])
        )
        return True
        
    def _idle(self):
        self.state = AIState.IDLE
        return True
        
    def _should_flee(self):
        return self.health < 30
        
    def _flee(self):
        if self.target is None:
            return False
            
        self.state = AIState.FLEE
        self.play_sound('flee')
        
        # Flee in opposite direction of target
        target_pos = np.array(self.target.get_position())
        direction = self.position - target_pos
        if np.linalg.norm(direction) < 0.001:
            return True
            
        direction = direction / np.linalg.norm(direction)
        new_position = self.position + direction * self.speed * 1.5  # Faster when fleeing
        new_position = self._avoid_obstacles(new_position)
        
        p.resetBasePositionAndOrientation(
            self.body_id,
            new_position.tolist(),
            p.getQuaternionFromEuler([0, 0, 0])
        )
        return True
        
    def _can_attack(self):
        if self.target is None:
            return False
            
        current_time = time.time()
        if current_time - self.last_attack_time < self.attack_cooldown:
            return False
            
        distance = np.linalg.norm(np.array(self.target.get_position()) - self.position)
        return distance < self.attack_radius
        
    def _attack(self):
        self.state = AIState.ATTACK
        self.play_sound('attack')
        self.last_attack_time = time.time()
        
        # Implement attack logic here (e.g., reduce target health)
        if hasattr(self.target, 'health'):
            self.target.health -= self.damage
            
        return True
        
    def _should_group(self):
        return len(self.group) > 0
        
    def _group_behavior(self):
        self.state = AIState.GROUP
        
        # Calculate center of group
        group_center = np.mean([member.position for member in self.group], axis=0)
        
        # Move towards group center while maintaining distance
        direction = group_center - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 2.0:  # Too far from group
            direction = direction / distance
            new_position = self.position + direction * self.speed
        elif distance < 1.0:  # Too close to group
            direction = -direction / distance
            new_position = self.position + direction * self.speed
        else:
            return True
            
        new_position = self._avoid_obstacles(new_position)
        p.resetBasePositionAndOrientation(
            self.body_id,
            new_position.tolist(),
            p.getQuaternionFromEuler([0, 0, 0])
        )
        return True
        
    def play_sound(self, sound_key):
        current_time = time.time()
        if (current_time - self.last_sound_time >= self.sound_cooldown and 
            self.sounds[sound_key] is not None):
            self.sounds[sound_key].play()
            self.last_sound_time = current_time
            
    def _avoid_obstacles(self, desired_position):
        # Cast rays to detect obstacles
        num_rays = 8
        ray_length = 1.0
        avoidance_force = np.zeros(3)
        
        for i in range(num_rays):
            angle = 2 * math.pi * i / num_rays
            ray_dir = np.array([math.cos(angle), 0, math.sin(angle)])
            ray_start = self.position
            ray_end = ray_start + ray_dir * ray_length
            
            result = p.rayTest(ray_start, ray_end)[0]
            if result[0] >= 0:  # Hit something
                hit_fraction = result[2]
                hit_distance = ray_length * hit_fraction
                if hit_distance < self.avoid_radius:
                    avoidance_force -= ray_dir * (self.avoid_radius - hit_distance)
        
        if np.linalg.norm(avoidance_force) > 0:
            avoidance_force = avoidance_force / np.linalg.norm(avoidance_force)
            desired_position += avoidance_force * self.speed
            
        return desired_position
        
    def get_position(self):
        return self.position.tolist()
        
    def get_color(self):
        colors = {
            AIState.IDLE: (0.2, 0.2, 0.8),    # Blue
            AIState.PATROL: (0.2, 0.8, 0.2),  # Green
            AIState.CHASE: (0.8, 0.2, 0.2),   # Red
            AIState.AVOID: (0.8, 0.8, 0.2),   # Yellow
            AIState.ATTACK: (1.0, 0.0, 0.0),  # Bright Red
            AIState.FLEE: (1.0, 0.5, 0.0),    # Orange
            AIState.GROUP: (0.5, 0.0, 0.5)    # Purple
        }
        return colors[self.state] 