(define (problem simple_robot_move_problem) (:domain simple_robot_move_domain)
(:objects 
    Rico - robot
    Kitchen - location
    Bathroom - location
    LivingRoom - location
    RicoChargerStation - location
    RicoCharger - charger
)
(:init
    (at-location Rico RicoChargerStation)

    (not(is-moving Rico Kitchen Bathroom))
    (not(is-moving Rico Kitchen LivingRoom))
    (not(is-moving Rico Kitchen RicoChargerStation))
    (not(is-moving Rico Kitchen Kitchen))
    (not(is-moving Rico Bathroom Bathroom))
    (not(is-moving Rico Bathroom LivingRoom))
    (not(is-moving Rico Bathroom RicoChargerStation))
    (not(is-moving Rico Bathroom Kitchen))
    (not(is-moving Rico LivingRoom RicoChargerStation))
    (not(is-moving Rico LivingRoom Kitchen))
    (not(is-moving Rico LivingRoom Bathroom))
    (not(is-moving Rico LivingRoom LivingRoom))

    (not(visited Rico Kitchen))
    (not(visited Rico Bathroom))
    (not(visited Rico LivingRoom))

    (not(is-charging Rico RicoCharger))

    (= (distance RicoChargerStation Kitchen) 20)
    (= (distance RicoChargerStation Bathroom) 4)
    (= (distance RicoChargerStation LivingRoom) 3)

    (= (distance Bathroom Kitchen) 7)
    (= (distance Bathroom RicoChargerStation) 1)
    (= (distance Bathroom LivingRoom) 2)

    (= (distance Kitchen Bathroom) 13)
    (= (distance Kitchen LivingRoom) 4)
    (= (distance Kitchen RicoChargerStation) 5)

    (= (distance LivingRoom Bathroom) 2)
    (= (distance LivingRoom Kitchen) 6)
    (= (distance LivingRoom RicoChargerStation) 10)

    (= (distance LivingRoom LivingRoom) 0)
    (= (distance RicoChargerStation RicoChargerStation) 0)
    (= (distance Bathroom Bathroom) 0)
    (= (distance Kitchen Kitchen) 0)

    (= (covered-distance RicoChargerStation Kitchen) 20)
    (= (covered-distance RicoChargerStation Bathroom) 4)
    (= (covered-distance RicoChargerStation LivingRoom) 3)

    (= (covered-distance Bathroom Kitchen) 7)
    (= (covered-distance Bathroom RicoChargerStation) 1)
    (= (covered-distance Bathroom LivingRoom) 2)

    (= (covered-distance Kitchen Bathroom) 13)
    (= (covered-distance Kitchen LivingRoom) 4)
    (= (covered-distance Kitchen RicoChargerStation) 5)

    (= (covered-distance LivingRoom Bathroom) 2)
    (= (covered-distance LivingRoom Kitchen) 6)
    (= (covered-distance LivingRoom RicoChargerStation) 10)

    (= (covered-distance LivingRoom LivingRoom) 0)
    (= (covered-distance RicoChargerStation RicoChargerStation) 0)
    (= (covered-distance Bathroom Bathroom) 0)
    (= (covered-distance Kitchen Kitchen) 0)

    (= (total-distance Rico) 0)
    (= (per-meter-cost Rico) 1)
    (= (battery-level Rico) 100)
    (= (battery-capacity Rico) 100)
    (= (speed Rico) 1)
    (= (charging-speed Rico) 1)

)

(:goal 
    (and 
        (visited Rico Kitchen)
        (visited Rico Bathroom)
        (visited Rico LivingRoom)
        (at-location Rico RicoChargerStation)
    )
)
; (:metric minimize (total-distance))
; (:metric maximize (battery-level Rico))
)
