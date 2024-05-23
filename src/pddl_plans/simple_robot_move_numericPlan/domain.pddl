(define (domain simple_robot_move_domain)

(:requirements :strips :typing :action-costs 
:negative-preconditions :conditional-effects
:fluents)

(:types
    static_object - object
    location - object
    robot - object
    possible_move - object
)
(:predicates 
    (at ?r - robot ?l - location)
    (possible_move ?from - location ?to - location)
    (visited ?l - location)
)
(:functions
    (distance ?from ?to - location)
    (energy-cost ?rico - robot ?from ?to - location)
    (total-cost)
    (battery)
)
(:action move
    :parameters (?r - robot 
        ?start_loc - location ?end_loc - location)
    :precondition (
        and(at ?r ?start_loc)
        (not(visited ?end_loc))
    )
    :effect (
        and(
        when (at ?r ?start_loc)(at ?r ?end_loc))
        (at ?r ?end_loc)
        (not(at ?r ?start_loc))
        (visited ?end_loc)
        (increase (total-cost) (distance ?start_loc ?end_loc))
        (decrease (battery) (energy-cost ?r ?start_loc ?end_loc))
    )
)
)
