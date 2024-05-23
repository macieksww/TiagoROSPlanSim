;Header and description

(define (domain simple_robot_move_domain)

(:requirements :strips :typing :negative-preconditions)

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
(:action move
    :parameters (?r - robot 
        ?start_loc - location ?end_loc - location)
    :precondition (
        and(at ?r ?start_loc)
        (not(visited ?end_loc))
    )

    :effect (
        and(at ?r ?end_loc)
        (not(at ?r ?start_loc))
        (visited ?end_loc)
        )
)
)
