(define (domain GridPlanning)
  (:requirements :strips :typing :negative-preconditions)
  (:types robot object grid_box)
  (:predicates
    (right_of ?x - grid_box ?y - grid_box)       ; Definiert, ob ?x rechts von ?y ist
    (left_of ?x - grid_box ?y - grid_box)        ; Definiert, ob ?x links von ?y ist
    (up_of ?x - grid_box ?y - grid_box)          ; Definiert, ob ?x über ?y ist
    (down_of ?x - grid_box ?y - grid_box)        ; Definiert, ob ?x unter ?y ist
    (robot_at ?r - robot ?y - grid_box)          ; Roboter befindet sich auf ?y
    (object_at ?obj - object ?y - grid_box)      ; Objekt befindet sich auf ?y
    (free ?r - robot)                            ; Roboter ist frei (nicht blockiert)
    (holding ?r - robot ?obj - object)           ; Roboter hält ein Objekt
    (accessible ?cell - grid_box)                ; Zelle ist zugänglich
    (goal_reachable ?goal - grid_box ?from - grid_box) ; Ziel ist erreichbar
    (is_edge_right ?cell - grid_box)             ; ?cell ist am rechten Rand
  )
  
  ; Bewegungsaktionen
  (:action move_right
    :parameters (?r - robot ?from - grid_box ?to - grid_box)
    :precondition (and (robot_at ?r ?from) (right_of ?from ?to) (free ?r) (accessible ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  (:action move_left
    :parameters (?r - robot ?from - grid_box ?to - grid_box)
    :precondition (and (robot_at ?r ?from) (left_of ?from ?to) (free ?r) (accessible ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  (:action move_up
    :parameters (?r - robot ?from - grid_box ?to - grid_box)
    :precondition (and (robot_at ?r ?from) (up_of ?from ?to) (free ?r) (accessible ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  (:action move_down
    :parameters (?r - robot ?from - grid_box ?to - grid_box)
    :precondition (and (robot_at ?r ?from) (down_of ?from ?to) (free ?r) (accessible ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  ; Normale Zielerreichung für alle Nicht-Randkacheln
  (:action set_goal_reachable
    :parameters (?r - robot ?right - grid_box ?goal - grid_box)
    :precondition (and 
      (accessible ?right)
      (robot_at ?r ?right)
      (right_of ?goal ?right)
    )
    :effect (goal_reachable ?right ?goal)
  )

  ; Zielerreichung speziell für rechte Randkacheln
  (:action set_goal_reachable_edge
    :parameters (?r - robot ?left - grid_box ?goal - grid_box)
    :precondition (and 
      (accessible ?goal)
      (robot_at ?r ?left)
      (left_of ?goal ?left)
      (is_edge_right ?goal)  ; Ziel ist eine Randkachel
    )
    :effect (goal_reachable ?left ?goal)
  )
)
