import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

from reg.main import Reg
from reg.gameobject import GameObject
from reg.line import Line
from reg.sprite import Sprite
from reg.text import Text
from reg.input import Input
from reg.audio import Audio
from reg.animation import Animation
from reg.light import Light
from reg.particle import ParticleSpawner
from reg.reg3d import Reg3D, GameObject3D, Camera3D

# Make 3D classes available at the top level
__all__ = [
    'Reg', 'GameObject', 'Line', 'Sprite', 'Text', 'Input',
    'Audio', 'Animation', 'Light', 'ParticleSpawner',
    'Reg3D', 'GameObject3D', 'Camera3D'
]
