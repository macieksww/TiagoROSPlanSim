(define (problem task)
(:domain simple_robot_move_domain)
(:objects
    kitchen bathroom livingroom ricochargerstation - location
    rico - robot
    ricocharger - charger
)
(:init
    (at-location rico ricochargerstation)

    (not (visited rico kitchen))
    (not (visited rico bathroom))
    (not (visited rico livingroom))



    (not (is-charging rico ricocharger))

    (not (is-moving rico kitchen bathroom))
    (not (is-moving rico kitchen livingroom))
    (not (is-moving rico kitchen ricochargerstation))
    (not (is-moving rico kitchen kitchen))
    (not (is-moving rico bathroom bathroom))
    (not (is-moving rico bathroom livingroom))
    (not (is-moving rico bathroom ricochargerstation))
    (not (is-moving rico bathroom kitchen))
    (not (is-moving rico livingroom ricochargerstation))
    (not (is-moving rico livingroom kitchen))
    (not (is-moving rico livingroom bathroom))
    (not (is-moving rico livingroom livingroom))

    (= (distance ricochargerstation kitchen) 20)
    (= (distance ricochargerstation bathroom) 4)
    (= (distance ricochargerstation livingroom) 3)
    (= (distance bathroom kitchen) 7)
    (= (distance bathroom ricochargerstation) 1)
    (= (distance bathroom livingroom) 2)
    (= (distance kitchen bathroom) 13)
    (= (distance kitchen livingroom) 4)
    (= (distance kitchen ricochargerstation) 5)
    (= (distance livingroom bathroom) 2)
    (= (distance livingroom kitchen) 6)
    (= (distance livingroom ricochargerstation) 10)
    (= (distance livingroom livingroom) 0)
    (= (distance ricochargerstation ricochargerstation) 0)
    (= (distance bathroom bathroom) 0)
    (= (distance kitchen kitchen) 0)

    (= (per-meter-cost rico) 1)

    (= (total-distance rico) 0)

    (= (battery-level rico) 100)

    (= (battery-capacity rico) 100)

    (= (speed rico) 1)

    (= (charging-speed rico) 1)


    (= (covered-distance ricochargerstation kitchen) 20)
    (= (covered-distance ricochargerstation bathroom) 4)
    (= (covered-distance ricochargerstation livingroom) 3)
    (= (covered-distance bathroom kitchen) 7)
    (= (covered-distance bathroom ricochargerstation) 1)
    (= (covered-distance bathroom livingroom) 2)
    (= (covered-distance kitchen bathroom) 13)
    (= (covered-distance kitchen livingroom) 4)
    (= (covered-distance kitchen ricochargerstation) 5)
    (= (covered-distance livingroom bathroom) 2)
    (= (covered-distance livingroom kitchen) 6)
    (= (covered-distance livingroom ricochargerstation) 10)
    (= (covered-distance livingroom livingroom) 0)
    (= (covered-distance ricochargerstation ricochargerstation) 0)
    (= (covered-distance bathroom bathroom) 0)
    (= (covered-distance kitchen kitchen) 0)

)
(:goal (and
    (visited rico kitchen)
    (visited rico bathroom)
    (visited rico livingroom)
    (at-location rico ricochargerstation)
))
)
