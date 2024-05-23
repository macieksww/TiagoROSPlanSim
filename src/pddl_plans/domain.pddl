(define (domain rover-domain)
    (:requirements :durative-actions :fluents :duration-inequalities)
    (:types
    static_object - object
    location - object
    robot - object
    possible_move - object
    )

    (:functions
        (battery-amount ?r - rover)
        (sample-amount ?r - rover)
        (recharge-rate ?r - rover)
        (battery-capacity)
        (sample-capacity)
        (distance-travelled)
    )
    (:predicates
        (at_location ?r - robot ?l - location)
	)

    (:durative-action move
        :parameters
            (?r - robot
             ?start_point - location
             ?end_point - location)

        :duration
            (= ?duration 5)

        :condition
	        (and
            	(at start (?end_point))
	            (at start (?start_point))
                (at start (?r))
	            (at start (at_location ?r ?start_point))
	            (over all (possible_move ?start_point ?end_point))
	            ; (at start (at_location ?rover ?from-waypoint))
	            ; (at start (> (battery-amount ?rover) 8)))

        :effect
	        (and
	            (at end (at_location ?r ?end_point))
	            ; (at end (been-at ?rover ?to-waypoint))
	            (at start (not (at_location ?r ?start_point)))
	            ; (at start (decrease (battery-amount ?rover) 8))
                ; (at end (increase (distance-travelled) 5))
                )
	)
)
