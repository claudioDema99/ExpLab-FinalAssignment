(define (domain robotFega)
(:requirements :strips :typing :adl :fluents :durative-actions)
  (:types 
    robot
    position
  )

  ;; Predicates
  (:predicates
    (at_robot ?r - robot ?p0 - position)
    (connected_pos ?pos1 ?pos2 - position)
  )

  ;; Numeric Fluents
  (:functions
    ;;(visibility ?m - marker)
  )

  ;; Durative Actions
  (:durative-action go_to_marker
    :parameters (?r - robot ?from ?to - position)
    :duration (= ?duration 1)
    :condition (and
      (at start(at_robot ?r ?from))
      (over all(connected_pos ?from ?to))
      ;;(over all(= (visibility ?m) 0))
    )
    :effect (and
      (at end(at_robot ?r ?to))
      ;;(at end(= (visibility ?m) 1))
    )
  )
)

