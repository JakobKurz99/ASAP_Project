import math
import random
import pygame

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY = (128, 128, 128)

SCALE = 21

# Pfad zur Plan-Ausgabedatei und Problemdateien
import os
import sys
cur_dir = os.path.dirname(__file__)
plan_file = os.path.join(cur_dir, "plan.txt")
problem1_file = os.path.join(cur_dir, "../problems/problem1.pddl")
problem2_file = os.path.join(cur_dir, "../problems/problem2.pddl")
world_file = os.path.join(cur_dir, "../worlds/gridworld.sdf")

def set_model_state(model_name, x, y):
    """Setzt die Position eines Objects in pygame."""
    update_ball_visualization(x, y)

def normalize_angle(angle):
    """Normalisiert einen Winkel auf den Bereich [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
    
def calculate_kick_angle(ball_x, ball_y, goal_x, goal_y, object_positions):
    """Berechnet den Winkel, um den Ball zum Tor zu schießen."""
    # Draw circle around ball
    pygame.draw.circle(pygame.display.get_surface(), RED, (math.ceil(ball_x * SCALE), math.ceil(ball_y * SCALE)), 10*SCALE, 1)  # Adjusted to center within the grid cell
    # Berechne den Winkel zum Tor
    angle = math.atan2(goal_y - ball_y, goal_x - ball_x)

    draw_angle(ball_x, ball_y, angle, color=RED)
    draw_angle(ball_x, ball_y, angle + math.radians(20), color=RED)
    draw_angle(ball_x, ball_y, angle - math.radians(20), color=RED)

    # Check if static robots are in the way
    free_shot = 0
    while free_shot<3:
        max_angle_obj = []
        min_angle_obj = []
        for obj_cell in object_positions:
            obj_x, obj_y = obj_cell
            # Calculate distance from ball to object
            distance_to_obj = []
            distance_to_obj.append(math.sqrt((ball_x - (obj_x))**2 + (ball_y - (obj_y))**2))
            distance_to_obj.append(math.sqrt((ball_x - (obj_x+1))**2 + (ball_y - (obj_y))**2))
            distance_to_obj.append(math.sqrt((ball_x - (obj_x))**2 + (ball_y - (obj_y+1))**2))
            distance_to_obj.append(math.sqrt((ball_x - (obj_x+1))**2 + (ball_y - (obj_y+1))**2))
            min_distance_obj = min(distance_to_obj)
            if min_distance_obj <= 10:
                angle_to_obj = []
                angle_to_obj.append(normalize_angle(math.atan2(obj_y - ball_y, obj_x - ball_x) - angle))
                angle_to_obj.append(normalize_angle(math.atan2(obj_y - ball_y, obj_x + 1 - ball_x) - angle))
                angle_to_obj.append(normalize_angle(math.atan2(obj_y + 1 - ball_y, obj_x - ball_x) - angle))
                angle_to_obj.append(normalize_angle(math.atan2(obj_y + 1 - ball_y, obj_x + 1 - ball_x) - angle))
                # if any of the absolute angles is smaller than 20°, the object is in the way 
                if any(abs(a) < math.radians(20) for a in angle_to_obj):
                    max_angle_obj.append(max(angle_to_obj))
                    min_angle_obj.append(min(angle_to_obj))
        if max_angle_obj and min_angle_obj:
            free_shot += 1
            draw_angle(ball_x, ball_y, max(max_angle_obj) + angle, color=BLUE)
            draw_angle(ball_x, ball_y, min(min_angle_obj) + angle, color=BLACK)
            print(f"Max angle: {max(max_angle_obj)}, Min angle: {min(min_angle_obj)}")
            if abs(max(max_angle_obj)) > abs(min(min_angle_obj)):
                angle += min(min_angle_obj) - math.radians(20)
            else:
                angle += max(max_angle_obj) + math.radians(20)
        else:
            draw_angle(ball_x, ball_y, angle, color=GREEN)
            draw_angle(ball_x, ball_y, angle + math.radians(20), color=GREEN)
            draw_angle(ball_x, ball_y, angle - math.radians(20), color=GREEN)
            break
    print(f"Attempts: {free_shot}")
    return angle

def calculate_kick_distance(ball_x, ball_y, goal_x, goal_y):
    if ball_x <= 3:
        # calculate distance to goal posts
        d1 = math.sqrt((goal_x - ball_x)**2 + (goal_y - 2 - ball_y)**2)
        d2 = math.sqrt((goal_x - ball_x)**2 + (goal_y + 2 - ball_y)**2)
        d = max(d1, d2)+0.1
        return min(d, 10)
    return 10

def kick_ball(robot_x, robot_y, ball_x, ball_y, goal_x, goal_y, object_positions, current_position=None):
    """Bewegt den Ball iterativ entlang des Bewegungsvektors bis zur maximalen Distanz oder dem Tor."""
    # Falls eine aktuelle Position übergeben wurde, setze sie als Startpunkt
    if current_position:
        ball_x, ball_y = current_position

    angle = calculate_kick_angle(ball_x, ball_y, goal_x, goal_y, object_positions)

    # Generiere zufälliges Rauschen für den Schuss
    noise =  generate_random_noise() #random.uniform(-20, 20)
    angle += math.radians(noise)  # Füge das Rauschen zum Winkel hinzu

    # Normiere den Bewegungsvektor auf die maximale Distanz von 10 Metern
    max_distance = calculate_kick_distance(ball_x, ball_y, goal_x, goal_y)
    dx = math.cos(angle) * max_distance
    dy = math.sin(angle) * max_distance

    #print(f"Berechne Bewegungsvektor: dx={dx}, dy={dy}")

    # Iterative Bewegung entlang des Vektors
    current_x, current_y = ball_x, ball_y
    steps = 100
    for i in range(1, steps+1):  # Maximal 10 Schritte (entspricht 10 Meter in 0.1-Meter-Schritten)
        last_x, last_y = current_x, current_y
        current_x += dx / steps  # Teile den Vektor in 100 kleine Schritte
        current_y += dy / steps

        # Runde auf das Grid für Planungszwecke
        grid_x = math.floor(current_x)
        grid_y = math.floor(current_y)

        #print(f"Schritt {i}: Ball bewegt sich zu x={current_x:.2f}, y={current_y:.2f} (Grid: x={grid_x}, y={grid_y})")

        # Reflektion prüfen und durchführen
        if current_x <= 0 and not (18 <= current_y < 22):  # Reflektion an der linken Wand außerhalb des Torbereichs
            #print(f"Ball trifft linke Wand bei x={current_x}. Reflektiere.")
            dx *= -1  # Reflektion entlang der x-Achse
        elif current_x >= 60:  # Reflektion an der rechten Wand
            #print(f"Ball trifft rechte Wand bei x={current_x}. Reflektiere.")
            dx *= -1  # Reflektion entlang der x-Achse
        if current_y <= 0 or current_y >= 40:  # Reflektion an der oberen oder unteren Wand
            #print(f"Ball trifft horizontale Wand bei y={current_y}. Reflektiere.")
            dy *= -1  # Reflektion entlang der y-Achse

        # Prüfe Reflektion an stationären Objekten
        for obj_cell in object_positions:
            obj_x, obj_y = obj_cell
            if obj_x <= current_x <= obj_x + 1 and obj_y <= current_y <= obj_y + 1:
                #print(f"Ball trifft Objekt bei {obj_cell}. Reflektiere.")
                # Reflektion an x-Seiten des Objekts
                if last_x <= obj_x < current_x:
                    dx *= -1
                elif last_x >= obj_x + 1 > current_x:
                    dx *= -1
                # Reflektion an y-Seiten des Objekts
                if last_y <= obj_y < current_y:
                    dy *= -1
                elif last_y >= obj_y + 1 > current_y:
                    dy *= -1

        # Prüfe, ob der Ball das Tor erreicht
        if -1 <= grid_x < 0 and 18 <= grid_y < 22:
            #print(f"Ball erreicht Torzelle bei x={grid_x}, y={grid_y}. Bewegung wird gestoppt.")
            set_model_state("ball", current_x, current_y)  # Exakte Werte für Gazebo
            return grid_x, grid_y, True, (current_x, current_y)  # Aktuelle Position hinzufügen

        # Bewege den Ball schrittweise in Gazebo
        set_model_state("ball", current_x, current_y)

    # Wenn der Ball die maximale Distanz zurückgelegt hat, ohne das Tor zu erreichen
    #print(f"Ball hat maximale Distanz erreicht bei x={grid_x}, y={grid_y}. Kein Tor.")
    set_model_state("ball", current_x, current_y)  # Exakte Werte für Gazebo
    return grid_x, grid_y, False, (current_x, current_y)  # Aktuelle Position hinzufügen

def generate_random_noise():
    """Generiert ein zufälliges Richtungsrauschen gemäß der Bedingung."""
    if random.uniform(0, 1) < 0.9:  # 90 % Wahrscheinlichkeit
        # Wähle ein Rauschen außerhalb des Bereichs [-15, 15]
        return random.uniform(-20, -15) if random.choice([True, False]) else random.uniform(15, 20)
    else:
        # Wähle ein Rauschen innerhalb des Bereichs [-15, 15]
        return random.uniform(-15, 15)
    
def generate_positions_for_entities():
    """Generiert ganzzahlige Koordinaten für Ball, Roboter und fünf Objekte."""
    # Definiere den Wertebereich für die X- und Y-Koordinaten
    x_range = (0, 19)
    y_range = (15, 24)

    # Generiere Koordinaten für jede Entität
    ball_position = (random.randint(*x_range), random.randint(*y_range))
    robot_position = (random.randint(*x_range), random.randint(*y_range))
    object_positions = [
        (random.randint(*x_range), random.randint(*y_range)) for _ in range(5)
    ]

    # Speichere alle Positionen in einem Dictionary
    positions = {
        "ball": ball_position,
        "robot": robot_position,
        "object1": object_positions[0],
        "object2": object_positions[1],
        "object3": object_positions[2],
        "object4": object_positions[3],
        "object5": object_positions[4],
    }

    return positions


def get_obj_positions(object_cells):
    """Konvertiert die Objektzellen in die entsprechenden Koordinaten."""
    obj_positions = []
    for cell in object_cells:
        x, y = map(int, cell.replace("cell_", "").split("_"))
        obj_positions.append((x, y))
    return obj_positions

def visualize(positions):
    """
    Visualizes the grid world using Pygame.
    :param positions: Dictionary with objects and their grid positions, e.g., {"ball": (5, 5), "robot": (10, 10)}.
    """
    pygame.init()
    width = 60 * SCALE
    height = 40 * SCALE
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Gridworld")

    # Set background to white
    screen.fill(WHITE)
    pygame.draw.rect(screen, GREEN, (0, 18*SCALE, 0.2*SCALE, 4*SCALE))

    # Draw the grid (10x10 cells)
    for x in range(0, width+1, SCALE):
        pygame.draw.line(screen, BLACK, (x, 0), (x, height))
    for y in range(0, height+1, SCALE):
        pygame.draw.line(screen, BLACK, (0, y), (width, y))

    # Draw objects
    for obj, pos in positions.items():
        #print(f"Drawing {obj} at {pos}")
        x, y = pos
        if obj == "ball":
            color = RED
            pygame.draw.circle(screen, color, (x * SCALE, y * SCALE), 0.2*SCALE)  # Adjusted to center within the grid cell
        elif obj == "robot":
            color = GREEN
            pygame.draw.rect(screen, color, (x * SCALE, y * SCALE, SCALE, SCALE))
        else:
            color = BLUE
            pygame.draw.rect(screen, color, (x * SCALE, y * SCALE, SCALE, SCALE))

    # Update the display
    pygame.display.flip()

def update_ball_visualization(ball_x, ball_y):
    """
    Updates the visualization of the ball in Pygame.
    :param ball_x: X-coordinate of the ball.
    :param ball_y: Y-coordinate of the ball.
    """
    # wait
    pygame.time.wait(20)
    screen = pygame.display.get_surface()
    pygame.draw.circle(screen, RED, (math.ceil(ball_x * SCALE), math.ceil(ball_y * SCALE)), 0.2*SCALE)  # Adjusted to center within the grid cell
    pygame.display.flip()

def draw_angle(ball_x, ball_y, angle, color=GREY):
    """
    Draws a line representing the angle of the ball's movement.
    :param ball_x: X-coordinate of the ball.
    :param ball_y: Y-coordinate of the ball.
    :param angle: Angle of the ball's movement.
    """
    screen = pygame.display.get_surface()
    dx = math.cos(angle) * 10
    dy = math.sin(angle) * 10
    pygame.draw.line(screen, color, (ball_x * SCALE, ball_y * SCALE), ((ball_x + dx) * SCALE, (ball_y + dy) * SCALE))
    pygame.display.flip()

def main():
    # Example positions for objects
    positions = {
        "robot": (0, 0),
        "ball": (15.5, 25.5),
        "object1": (1, 1),
        "object2": (2, 2), # (15, 23)
        "object3": (3, 3), # (15, 27)
        "object4": (11, 23), # (13, 25)
        "object5": (12, 25), # (17, 25)
    }
    positions = generate_positions_for_entities()

    positions["robot"] = (0, 0)
    positions["ball"] = (random.uniform(40, 59), random.uniform(10, 29))

    # Initialize Pygame visualization
    visualize(positions)

    # Kick the ball towards the goal
    goal = False
    while not goal:
        # Get the current position of the ball
        ball_x, ball_y = positions["ball"]
        robot_x, robot_y = positions["robot"]
        goal_x, goal_y = 0, 20
        obstacle_positions = [positions["object1"], positions["object2"], positions["object3"], positions["object4"], positions["object5"]]
        x, y, goal, current_position = kick_ball(robot_x, robot_y, ball_x, ball_y, goal_x, goal_y, obstacle_positions)
        positions["ball"] = current_position

    # Keep the window open and handle quit events
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
    pygame.quit()

if __name__ == "__main__":
    main()


