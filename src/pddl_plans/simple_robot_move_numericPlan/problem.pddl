(define (problem simple_robot_move_problem) (:domain simple_robot_move_domain)
(:objects 
    Rico - robot
    Kitchen - location
    Bathroom - location
    LivingRoom - location
    RicoCharger - location
)
(:init
    (at Rico RicoCharger)
    (= (distance RicoCharger Kitchen) 20)
    (= (distance RicoCharger Bathroom) 4)
    (= (distance RicoCharger LivingRoom) 3)
    (= (distance Bathroom Kitchen) 7)
    (= (distance Bathroom RicoCharger) 1)
    (= (distance Bathroom LivingRoom) 2)
    (= (distance Kitchen Bathroom) 13)
    (= (distance Kitchen LivingRoom) 4)
    (= (distance Kitchen RicoCharger) 5)
    (= (distance LivingRoom Bathroom) 2)
    (= (distance LivingRoom Kitchen) 6)
    (= (distance LivingRoom RicoCharger) 10)
    (= (total-cost) 0)

    (= (energy-cost Rico RicoCharger Kitchen) 20)
    (= (energy-cost Rico RicoCharger Bathroom) 4)
    (= (energy-cost Rico RicoCharger LivingRoom) 3)
    (= (energy-cost Rico Bathroom Kitchen) 7)
    (= (energy-cost Rico Bathroom RicoCharger) 1)
    (= (energy-cost Rico Bathroom LivingRoom) 2)
    (= (energy-cost Rico Kitchen Bathroom) 13)
    (= (energy-cost Rico Kitchen LivingRoom) 4)
    (= (energy-cost Rico Kitchen RicoCharger) 5)
    (= (energy-cost Rico LivingRoom Bathroom) 2)
    (= (energy-cost Rico LivingRoom Kitchen) 6)
    (= (energy-cost Rico LivingRoom RicoCharger) 10)
    (= (battery) 100)
)

(:goal 
    (and 
        (visited Kitchen)
        (visited RicoCharger)
        (visited Bathroom)
        (visited LivingRoom)
    )
)
; (:metric minimize (total-cost))
(:metric maximize (battery))
)
