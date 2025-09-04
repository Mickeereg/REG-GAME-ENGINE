from reg import *
import random

# General Setup
reg = Reg("Pong")

bg_color = (0, 0, 0)
light_gray = (200, 200, 200)

score1 = 0
score2 = 0

class Paddle(GameObject):
    def __init__(self, x, y):
        super().__init__(
            window = reg,
            shape = "rect",
            width = 10,
            height = 140,
            x = x,
            y = y,
            color = light_gray
        )

        self.player_pos = reg.height / 2 - 70
        self.opponent_speed = 7

    def player_movement(self):
        self.y = self.player_pos

        if self.top <= 0:
            self.top = 0
        if self.bottom >= reg.height:
            self.bottom = reg.height

    def opponent_movement(self):
        if self.top < ball.y:
            self.top += self.opponent_speed
        if self.bottom > ball.y:
            self.bottom -= self.opponent_speed

        if self.top <= 0:
            self.top = 0
        if self.bottom >= reg.height:
            self.bottom = reg.height

class Ball(GameObject):
    def __init__(self):
        super().__init__(
            window = reg,
            shape = "ellipse",
            width = 20,
            height = 20,
            x = reg.width / 2 - 15,
            y = reg.height / 2 - 15,
            color = light_gray
        )

        self.ball_speed_x = 7 * random.choice((1, -1))
        self.ball_speed_y = 7 * random.choice((1, -1))

    def ball_movement(self, dt):
        global score1
        global score2
        # Animate the ball
        self.x += 50 * self.ball_speed_x * dt
        self.y += 50 * self.ball_speed_y * dt

        # Collisions
        if self.top <= 0 or self.bottom >= reg.height:
            self.ball_speed_y *= -1
        if self.left <= 0:
            self.ball_reset()
            score2 += 1
        elif self.right >= reg.width:
            self.ball_reset()
            score1 += 1

        if self.colliderect(player) or self.colliderect(opponent):
            self.ball_speed_x *= -1

    def ball_reset(self):
        self.center()
        self.ball_speed_y *= random.choice((1, -1))
        self.ball_speed_x *= random.choice((1, -1))

ball = Ball()
player = Paddle(reg.width - 20, reg.height / 2 - 70)
opponent = Paddle(10, reg.height / 2 - 70)

speed = 7

@reg.draw
def draw():
    # Visuals
    reg.window.fill(bg_color)
    player.draw()
    opponent.draw()
    ball.draw()
    score1_text = Text(reg, str(score1), 24, None, "white", 10)
    score2_text = Text(reg, str(score2), 24, None, "white", reg.width - 20)
    score1_text.draw()
    score2_text.draw()
    line = Line(reg, light_gray, (reg.width / 2, 0), (reg.width / 2, reg.height))

@reg.update
def update(dt):
    # Handling inputs
    if reg.input.get_key_pressed(reg.input.keys["UP"]):
        player.player_pos -= 50 * speed * dt

    if reg.input.get_key_pressed(reg.input.keys["DOWN"]):
        player.player_pos += 50 * speed * dt

    # Ball movement
    ball.ball_movement(dt)
    
    # Player movement
    player.player_movement()

    # Opponent movement
    opponent.opponent_movement()

reg.loop()