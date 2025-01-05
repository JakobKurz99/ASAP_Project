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

def kick_ball(robot_x, robot_y, ball_x, ball_y, goal_x, goal_y):
    """Simuliert einen Schuss des Balls in Richtung des Tors."""
    dx = goal_x - ball_x
    dy = goal_y - ball_y
    angle = math.atan2(dy, dx)

    # Füge Rauschen zum Schusswinkel hinzu
    angle += math.radians(random.uniform(-10, 10))

    # Berechne die neue Ballposition basierend auf einer maximalen Distanz von 10 Metern
    max_distance = 10
    new_x = ball_x + max_distance * math.cos(angle)
    new_y = ball_y + max_distance * math.sin(angle)

    # Runden der Werte gemäß den Anforderungen
    new_x = round(new_x) if abs(new_x - int(new_x)) < 0.5 else math.ceil(new_x)
    new_y = round(new_y) if abs(new_y - int(new_y)) < 0.5 else math.ceil(new_y)

    # Wenn der Ball im Tor ist oder die Grenze überschreitet, stoppe ihn
    if new_x <= -0.5:
        print(f"Ball erreicht das Torbereich bei x: {new_x}, y: {new_y}. Stoppe Bewegung.")
        new_x = 0  # Angenommene Stoppposition für das Tor
        new_y = max(18, min(new_y, 22))  # Im Torbereich zwischen 18 und 22 halten

    print(f"Schieße Ball von x: {ball_x}, y: {ball_y} zu x: {new_x}, y: {new_y}")
    success = set_model_state("ball", new_x, new_y)
    if not success:
        print("Fehler beim Setzen der Ballposition.")

    return new_x, new_y

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
    lines[9417] = f"(robot_at kenny cell_{ball_x}_{ball_y})\n"

    with open(problem2_file, "w") as f:
        f.writelines(lines)

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
                        if len(parts) < 5:
                            continue
                        
                        _, _, start_cell, end_cell = parts[:4]
                        _, x_end, y_end = end_cell.split('_')
                        x_end, y_end = int(x_end), int(y_end)
                        last_target_position = (x_end, y_end)

                        print(f"Bewege Roboter zu x: {x_end}, y: {y_end}")
                        success = set_model_state("robot", x_end, y_end)
                        if not success:
                            print(f"Fehler: Konnte Position x: {x_end}, y: {y_end} nicht setzen.")
                        rospy.sleep(0.1)
                    except Exception as e:
                        print(f"Fehler beim Verarbeiten der Zeile: {line.strip()}")
                        print(e)

            if last_target_position:
                robot_x, robot_y = last_target_position
                ball_x, ball_y = last_target_position
                goal_x, goal_y = 0, 20
                new_ball_x, new_ball_y = kick_ball(robot_x, robot_y, ball_x, ball_y, goal_x, goal_y)
                update_problem_files(robot_x, robot_y, new_ball_x, new_ball_y)

                # Prüfe, ob der Ball im Tor ist
                if new_ball_x < 0 and 18 < new_ball_y < 22:
                    print("Ball ist im Tor!")
                    break
            else:
                print("Fehler: Keine Zielposition gefunden, um den Ball zu schießen.")
        except FileNotFoundError:
            print(f"Fehler: Die Plan-Datei '{plan_file}' wurde nicht gefunden.")
