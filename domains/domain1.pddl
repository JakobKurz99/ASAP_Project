(define (domain GridPlanning)
    (:requirements :strips :typing)
    (:types 
        robot 
        grid_box)
    (:predicates
        (beside ?x - grid_box ?y - grid_box)
        (robot_at ?r - robot ?y - grid_box)
    )
    (:action move_forward
        :parameters (?r - robot ?from - grid_box ?to - grid_box)
        :precondition (and (robot_at ?r ?from) (beside ?from ?to))
        :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
    )
    (:action move_backward
        :parameters (?r - robot ?from - grid_box ?to - grid_box)
        :precondition (and (robot_at ?r ?from) (beside ?to ?from))
        :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
    )
)
