;Header and description

(define (domain simple_robot_move_domain)

(:requirements :typing :fluents :durative-actions
 :negative-preconditions)


(:types
    static_object - object
    location - object
    robot - object
    charger - object
)
(:predicates 
    (at-location ?r - robot ?l - location)
    (visited ?r - robot ?l - location)
    (next ?l1 - location ?l2 - location)
    (at-location-charger ?c - charger ?l - location)
    (is-charging ?r - robot ?c - charger)
    (is-moving ?r - robot ?l1 - location ?l2 - location)

)
(:functions
    (distance ?from ?to - location)
    (per-meter-cost ?r - robot)
    (total-distance ?r - robot)
    (battery-level ?r - robot)
    (battery-capacity ?r - robot)
    (speed ?r - robot)
    (charging-speed ?r - robot)
    (motion-time ?from ?to - location)
    (covered-distance ?from ?to - location)
)

; MOVE

(:action start-move
    :parameters (?r - robot 
        ?start-loc - location
        ?end-loc - location)

    :precondition (
        and 
        (at-location ?r ?start-loc)
        ; (not(visited ?r ?end-loc))
        (not(is-moving ?r ?start-loc ?end-loc))
        ; (>= (battery-level ?r) (* (distance ?start-loc ?end-loc) (per-meter-cost ?r)))
    )
    :effect (
        and
        (not(at-location ?r ?start-loc))
        (is-moving ?r ?start-loc ?end-loc)
    )
)

(:process in-motion
    :parameters (?r - robot
        ?start-loc - location
        ?end-loc - location)

    :precondition (
        and
        (is-moving ?r ?start-loc ?end-loc)
    )
    :effect( 
        and
        (decrease (battery-level ?r) (* #t (per-meter-cost ?r)))
        (increase (total-distance) (* #t (speed ?r)))
        (increase (covered-distance ?start-loc ?start-loc) (* #t (speed ?r)))
    )
)

; (:action end-move
;     :parameters (?r - robot 
;         ?start-loc - location
;         ?end-loc - location)

;     :precondition (
;         and 
;         (is-moving ?r ?start-loc ?end-loc)
;         (>= (covered-distance ?start-loc ?end-loc)(distance ?start-loc ?end-loc))
;     )
;     :effect (
;         and
;         (not(is-moving ?r ?start-loc ?end-loc))
;         (at-location ?r ?end-loc)
;         (visited ?r ?end-loc)
;         (assign (motion-time ?start-loc ?end-loc) 0)
;         (assign (covered-distance ?start-loc ?end-loc) 0)
;         (assign (total-distance) 10)
;     )
; )

(:event end-move
    :parameters (?r - robot 
        ?start-loc - location
        ?end-loc - location)

    :precondition (and
        (is-moving ?r ?start-loc ?end-loc)
        (>= (covered-distance ?start-loc ?end-loc)(distance ?start-loc ?end-loc))
    )
    :effect (
        and
        (assign (motion-time ?start-loc ?end-loc) 0)
        (assign (covered-distance ?start-loc ?end-loc) 0)
        (not(is-moving ?r ?start-loc ?end-loc))
        (at-location ?r ?end-loc)
        (visited ?r ?end-loc)
    )
)

)