(define (problem task)
(:domain simple_robot_move_domain)
(:objects
    kitchen bathroom livingroom ricocharger - location
    rico - robot
)
(:init
    (at rico ricocharger)



)
(:goal (and
    (visited kitchen)
    (visited bathroom)
    (visited livingroom)
    (visited ricocharger)
))
)
