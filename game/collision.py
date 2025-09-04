from reg import *

reg = Reg(title="reg", width=800, height=600, bg_color="black")

# Game Objects
square = GameObject(window=reg, shape="rect", width=20, height=30, x=0, y=0, color="orange")
square.center()

ground = GameObject(window=reg, shape="rect", width=5000, height=100, x=0, y=0, color="gray")
ground.center()
ground.y = 500

# Character setup
character = Sprite(window=reg, image="assets/adventurer.png", x=0, y=0, width=200, height=200)
character.center()
animation = Animation("assets/idle1", 0.14)
run_anim = Animation("assets/normal-run", 0.14)

character.add_animation(animation, "idle")
character.add_animation(run_anim, "run")

# Physics variables
speed = 5
jump_strength = -350  
gravity = 800  
character.velocity_y = 0  

@reg.draw
def draw():
    square.draw()
    ground.draw()
    character.draw()

@reg.update
def update(dt):
    reg.input.update()
    if reg.input.get_key_pressed(reg.input.keys["D"]):
        character.play_animation("run")
        character.x += 60 * speed * dt

    elif reg.input.get_key_pressed(reg.input.keys["A"]):
        character.play_animation("run", "x")
        character.x -= 60 * speed * dt

    else:
        character.play_animation("idle")

    # Jumping logic (Only trigger on key press)
    if reg.input.get_key_down(reg.input.keys["SPACE"]) and character.collide(ground):
        character.velocity_y = jump_strength  # Jump impulse

    # Apply gravity
    character.velocity_y += gravity * dt
    character.y += character.velocity_y * dt

    # Prevent character from falling through the ground
    if character.collide(ground):
        character.y = ground.y - character.height  # Snap to ground
        character.velocity_y = 0  # Reset velocity

    # Square falls due to gravity
    if not square.collide(ground):
        square.y += 150 * dt

reg.loop()
