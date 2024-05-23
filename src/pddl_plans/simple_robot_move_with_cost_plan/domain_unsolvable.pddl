;Header and description

(define (domain simple_robot_move_domain)

(:requirements :typing :fluents :durative-actions
 :negative-preconditions :action-costs)


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

)
(:functions
    (distance ?from ?to - location)
    (per-meter-cost ?r - robot)
    (total-distance ?r - robot)
    (battery-level ?r - robot)
    (battery-capacity ?r - robot)
    (speed ?r - robot)
    (charging-speed ?r - robot)
)

; ACTIONS

(:durative-action move
    :parameters (?r - robot 
        ?start-loc - location
        ?end-loc - location)

    :duration
        (= ?duration (* (distance ?start-loc ?end-loc) (speed ?r)))

    :condition (
        and 
        (at start(at-location ?r ?start-loc))
        (at start (not(visited ?r ?end-loc)))
        (at start (>= (battery-level ?r) (* (distance ?start-loc ?end-loc) (per-meter-cost ?r))))
        ; (at start)
        ; (over all (next ?start_loc ?end_loc))
    )
    :effect (
        and
        (decrease (battery-level ?r) (* #t (per-meter-cost ?r)))
        ; (decrease (battery-level ?r) (* #t 1))
        (at end(at-location ?r ?end-loc))
        (at end(visited ?r ?end-loc))
        (at start(not(at-location ?r ?start-loc)))
        ; (at start (decrease (battery-level ?r) (* (distance ?start-loc ?end-loc) (per-meter-cost ?r))))
        (at start (increase (total-distance ?r) (distance ?start-loc ?end-loc)))
    )
)

; (:durative-action recharge
;     :parameters (?r - robot 
;         ?charger-loc - location
;         ?c - charger)

;     :duration
;         (= ?duration (*(-(battery-capacity ?r)(battery-level ?r))(charging-speed ?r ?c)))

;     :condition (
;         and 
;         (at start (at-location ?r ?charger-loc))
;         (at start (at-location-charger ?c ?charger-loc))
;     )
;     :effect (
;         ; and (at end(assign (battery-level ?r) (battery-capacity ?r)))
;         and (increase (battery-level ?r) (* #t 1.0))
;     )
; )

; (:action dock-at-charger
;     :parameters (?r - robot 
;         ?charger-loc - location
;         ?c - charger)

;     :precondition (
;         and(at-location ?r ?charger-loc)
;         (at-location ?c ?charger-loc)
;         ; (needs-recharge ?r)
;         (not(is-charging ?r ?c))
;     )

;     :effect (
;         and(at-location ?r ?charger-loc)
;         (at-location ?c ?charger-loc)
;         (is-charging ?r ?c)
;     )
; )

; (:action undock-from-charger
;     :parameters (?r - robot 
;         ?charger-loc - location
;         ?c - charger)

;     :precondition (
;         (is-charging ?r ?c)
;     )

;     :effect (
;         and(at-location ?r ?charger-loc)
;         (at-location ?c ?charger-loc)
;         (not(is-charging ?r ?c))
;     )
; )

; ; PROCESSES

; (:process charging
;     :parameters (?r - robot 
;         ?c - charger)

;     :precondition (
;         (is-charging ?r ?c)
;     )

;     :effect (
;         and(
;             (increase (battery-level ?r) (* #t charging-speed))
;         )
;     )
; )

; )