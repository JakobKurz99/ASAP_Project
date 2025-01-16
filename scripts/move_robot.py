import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math
import random
import subprocess

# Pfad zur Plan-Ausgabedatei und Problemdateien
plan_file = "/home/jakob/workspace_rosplan/src/project_package/scripts/plan.txt"
problem1_file = "/home/jakob/workspace_rosplan/src/project_package/problems/problem1.pddl"
problem2_file = "/home/jakob/workspace_rosplan/src/project_package/problems/problem2.pddl"
world_file = "/home/jakob/workspace_rosplan/src/project_package/worlds/gridworld.sdf"

def set_model_state(model_name, x, y, z=0.5):
    """Setzt die Position eines Modells in Gazebo."""
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 1

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

import math

def kick_ball(robot_x, robot_y, ball_x, ball_y, goal_x, goal_y, object_cells, current_position=None):
    """Bewegt den Ball iterativ entlang des Bewegungsvektors bis zur maximalen Distanz oder dem Tor."""
    # Falls eine aktuelle Position übergeben wurde, setze sie als Startpunkt
    if current_position:
        ball_x, ball_y = current_position

    dx = goal_x - ball_x
    dy = goal_y - ball_y
    angle = math.atan2(dy, dx)

    # Generiere zufälliges Rauschen für den Schuss
    noise = generate_random_noise()
    angle += math.radians(noise)  # Füge das Rauschen zum Winkel hinzu

    # Normiere den Bewegungsvektor auf die maximale Distanz von 10 Metern
    max_distance = 10
    dx = math.cos(angle) * max_distance
    dy = math.sin(angle) * max_distance

    print(f"Berechne Bewegungsvektor: dx={dx}, dy={dy}")

    # Iterative Bewegung entlang des Vektors
    current_x, current_y = ball_x, ball_y
    for i in range(1, 101):  # Maximal 10 Schritte (entspricht 10 Meter in 0.1-Meter-Schritten)
        current_x += dx / 100  # Teile den Vektor in 100 kleine Schritte
        current_y += dy / 100

        # Runde auf das Grid für Planungszwecke
        grid_x = math.floor(current_x)
        grid_y = math.floor(current_y)

        print(f"Schritt {i}: Ball bewegt sich zu x={current_x:.2f}, y={current_y:.2f} (Grid: x={grid_x}, y={grid_y})")

        # Reflektion prüfen und durchführen
        if grid_x == 0 and not (18 <= grid_y < 22):  # Reflektion an der linken Wand außerhalb des Torbereichs
            print(f"Ball trifft linke Wand bei x={grid_x}. Reflektiere.")
            dx *= -1  # Reflektion entlang der x-Achse
        elif grid_x == 60:  # Reflektion an der rechten Wand
            print(f"Ball trifft rechte Wand bei x={grid_x}. Reflektiere.")
            dx *= -1  # Reflektion entlang der x-Achse
        if grid_y <= 0 or grid_y >= 40:  # Reflektion an der oberen oder unteren Wand
            print(f"Ball trifft horizontale Wand bei y={grid_y}. Reflektiere.")
            dy *= -1  # Reflektion entlang der y-Achse

        # Prüfe Reflektion an stationären Objekten
        for obj_cell in object_cells:
            obj_x, obj_y = map(int, obj_cell.replace("cell_", "").split('_'))
            if obj_x <= current_x < obj_x + 1 and obj_y <= current_y < obj_y + 1:
                print(f"Ball trifft Objekt bei {obj_cell}. Reflektiere.")
                if obj_x <= grid_x < obj_x + 1:  # Reflektion an x-Seiten des Objekts
                    dx *= -1
                if obj_y <= grid_y < obj_y + 1:  # Reflektion an y-Seiten des Objekts
                    dy *= -1

        # Prüfe, ob der Ball das Tor erreicht
        if -1 <= grid_x < 0 and 18 <= grid_y <= 22:
            print(f"Ball erreicht Torzelle bei x={grid_x}, y={grid_y}. Bewegung wird gestoppt.")
            set_model_state("ball", current_x, current_y)  # Exakte Werte für Gazebo
            return grid_x, grid_y, True, (current_x, current_y)  # Aktuelle Position hinzufügen

        # Bewege den Ball schrittweise in Gazebo
        set_model_state("ball", current_x, current_y)

    # Wenn der Ball die maximale Distanz zurückgelegt hat, ohne das Tor zu erreichen
    print(f"Ball hat maximale Distanz erreicht bei x={grid_x}, y={grid_y}. Kein Tor.")
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

def update_problem_files(robot_x, robot_y, ball_x, ball_y):
    """Aktualisiert die Problemdateien basierend auf neuen Positionen."""
    # Aktualisiere `problem1.pddl`
    with open(problem1_file, "r") as f:
        lines = f.readlines()

    lines[4707] = f"(robot_at kenny g{robot_x}_{robot_y})\n"
    lines[4708] = f"(object_at ball g{ball_x}_{ball_y})\n"
    lines[4711] = f"(robot_at kenny g{ball_x}_{ball_y})\n"
        
    with open(problem1_file, "w") as f:
        f.writelines(lines)

    # Aktualisiere `problem2.pddl`
    with open(problem2_file, "r") as f:
        lines = f.readlines()

    lines[9] = f"(robot_at kenny cell_{robot_x}_{robot_y})\n"
    if ball_x < 59:
        lines[11852] = f"  (goal_reachable cell_{ball_x + 1}_{ball_y} cell_{ball_x}_{ball_y})\n"
    else:
        lines[11852] = f"  (goal_reachable cell_{ball_x - 1}_{ball_y} cell_{ball_x}_{ball_y})\n"

    with open(problem2_file, "w") as f:
        f.writelines(lines)

def update_world_and_problem2(positions):
    """Aktualisiert die Simulationsumgebung UNd Problemdatei basierend auf neuen Positionen."""
    ball_x, ball_y = positions["ball"]
    robot_x, robot_y = positions["robot"]
    object1_x, object1_y = positions["object1"]
    object2_x, object2_y = positions["object2"]
    object3_x, object3_y = positions["object3"]
    object4_x, object4_y = positions["object4"]
    object5_x, object5_y = positions["object5"]
    
    # Aktualisiere `gridworld.sdf`
    with open(world_file, "r") as f:
        lines = f.readlines()

    lines[62694] = f"      <pose>{object1_x} {object1_y} 0.5 0 0 0</pose>\n"
    lines[62721] = f"      <pose>{object2_x} {object2_y} 0.5 0 0 0</pose>\n"
    lines[62748] = f"      <pose>{object3_x} {object3_y} 0.5 0 0 0</pose>\n"
    lines[62775] = f"      <pose>{object4_x} {object4_y} 0.5 0 0 0</pose>\n"
    lines[62802] = f"      <pose>{object5_x} {object5_y} 0.5 0 0 0</pose>\n"
    lines[62829] = f"      <pose>{robot_x} {robot_y} 0.5 0 0 0</pose>\n"
    lines[62874] = f"      <pose>{ball_x + 0.5} {ball_y + 0.5} 0.5 0 0 0</pose>\n"

    with open(world_file, "w") as f:
        f.writelines(lines)

    # Aktualisiere `problem2.pddl`
    with open(problem2_file, "r") as f:
        lines = f.readlines()

    # Speichere alte Positionen und aktualisiere die object_at Zeilen
    old_positions = {}
    for i, obj in enumerate(["o1", "o2", "o3", "o4", "o5"], start=10):
        line = lines[i].strip()
        if line.startswith(f"(object_at {obj}"):
            # Extrahiere alte cell_ Positionen
            cell_value = line.split()[-1][:-1]  # Entferne die schließende Klammer
            old_positions[obj] = cell_value

            # Hole neue Positionen aus `positions` und aktualisiere die Zeile
            new_x, new_y = positions[f"object{int(obj[1])}"]
            new_cell = f"cell_{int(new_x)}_{int(new_y)}"
            lines[i] = f"(object_at {obj} {new_cell})\n"

    # Aktualisiere die accessible Zeilen mit den alten Positionen
    for i, (obj, old_cell) in enumerate(old_positions.items()):
        # Suche die Zeile mit der neuen Position in accessible und ersetze sie mit der alten Position
        new_x, new_y = positions[f"object{int(obj[1])}"]
        new_cell = f"cell_{int(new_x)}_{int(new_y)}"
        old_accessible = f"(accessible {new_cell})"
        new_accessible = f"(accessible {old_cell})"

        lines = [line.replace(old_accessible, new_accessible) for line in lines]

    # Schreibe die aktualisierte Datei zurück
    with open(problem2_file, "w") as f:
        f.writelines(lines)

def generate_positions_for_entities():
    """Generiert ganzzahlige Koordinaten für Ball, Roboter und fünf Objekte."""
    # Definiere den Wertebereich für die X- und Y-Koordinaten
    x_range = (0, 59)
    y_range = (0, 39)

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

def get_object_cells(problem_file):
    """Liest die Objektpositionen aus der problem2.pddl-Datei."""
    object_cells = []
    try:
        with open(problem_file, "r") as f:
            lines = f.readlines()
        # Zeilen 10 bis 14 enthalten die Objektpositionen
        for i in range(10, 15):
            line = lines[i].strip()
            if line.startswith("(object_at"):
                # Extrahiere die Zellkoordinaten, z.B. "cell_38_20"
                cell = line.split()[-1][:-1]  # Entferne die abschließende Klammer
                object_cells.append(cell)
    except FileNotFoundError:
        print(f"Fehler: Die Datei '{problem_file}' wurde nicht gefunden.")
    except Exception as e:
        print(f"Fehler beim Lesen der Objektpositionen: {e}")
    return object_cells


def execute_planner():
    """Führt den Planer aus."""
    planner_command = [
        "/home/jakob/workspace_rosplan/src/rosplan/rosplan_planning_system/common/bin/popf",
        "/home/jakob/workspace_rosplan/src/project_package/domains/domain2.pddl",
        "/home/jakob/workspace_rosplan/src/project_package/problems/problem2.pddl",
    ]
    with open(plan_file, "w") as plan_output:
        subprocess.run(planner_command, stdout=plan_output)

if __name__ == "__main__":
    rospy.init_node('robot_mover', anonymous=True)

    # Lese die Objektzellen aus der Datei
    object_cells = get_object_cells(problem2_file)
    print(f"Objektzellen geladen: {object_cells}")

    # Initiale Position des Balls
    ball_x, ball_y = 30, 22  # Beispielhafte Startkoordinaten im Gitter
    current_position = (ball_x + 0.5, ball_y + 0.5)  # Setze initiale Position auf die Mitte der Zelle

    while True:
        execute_planner()

        parsing_started = False
        last_target_position = None

        try:
            with open(plan_file, "r") as f:
                for line in f:
                    if not parsing_started:
                        if line.startswith("0.000:"):
                            parsing_started = True
                        else:
                            continue

                    try:
                        parts = line.strip().split()
                        if len(parts) < 6:  # Stelle sicher, dass genug Teile vorhanden sind
                            continue

                        # Extrahiere die Zielzelle
                        end_cell = parts[4].strip(')')  # Entferne die schließende Klammer
                        _, x_end, y_end = end_cell.split('_')  # Extrahiere x und y
                        x_end, y_end = int(x_end), int(y_end)
                        last_target_position = (x_end, y_end)

                        print(f"Bewege Roboter zu x: {x_end}, y: {y_end}")
                        x_end = x_end + 0.5
                        y_end = y_end + 0.5
                        success = set_model_state("robot", x_end, y_end)
                        if not success:
                            print(f"Fehler: Konnte Position x: {x_end}, y: {y_end} nicht setzen.")
                        rospy.sleep(0.1)
                    except Exception as e:
                        print(f"Fehler beim Verarbeiten der Zeile: {line.strip()}")
                        print(e)

            if last_target_position:
                robot_x, robot_y = last_target_position
                goal_x, goal_y = -1, 20

                print(f"Neue Ballposition für Kicksimulation: current_position={current_position}")

                # Kick den Ball und übergebe die Objektzellen
                new_ball_x, new_ball_y, is_goal, current_position = kick_ball(
                    robot_x, robot_y, ball_x, ball_y, goal_x, goal_y, object_cells, current_position
                )

                # Wenn der Ball im Tor ist, starte den Reset-Prozess
                if is_goal:
                    print("Ball ist im Tor! Starte Reset.")
                    positions = generate_positions_for_entities()
                    new_ball_x, new_ball_y = positions["ball"]
                    robot_x, robot_y = positions["robot"]
                    current_position = (new_ball_x + 0.5, new_ball_y + 0.5)  # Neue Position nach Reset
                    update_problem_files(robot_x, robot_y, new_ball_x, new_ball_y)
                    update_world_and_problem2(positions)
                    print("Reset abgeschlossen. Programm beendet.")
                    break  # Programm erfolgreich beenden

                print(f"Roboterposition für planner x: {robot_x}, y: {robot_y}")
                print(f"Neue Ballposition für planner x: {new_ball_x}, y: {new_ball_y}")
                # Aktualisiere Problemdateien nur, wenn kein Tor erzielt wurde
                update_problem_files(robot_x, robot_y, new_ball_x, new_ball_y)

            else:
                print("Fehler: Keine Zielposition gefunden, um den Ball zu schießen.")

        except FileNotFoundError:
            print(f"Fehler: Die Plan-Datei '{plan_file}' wurde nicht gefunden.")
