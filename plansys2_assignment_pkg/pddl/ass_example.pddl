(define (domain rosbotAruco)
(:requirements :strips :typing :adl :fluents :durative-actions)
  (:types 
    marker 
    position
  )

  ;; Predicates
  (:predicates
    (at_robot ?p0 - position)
    (at_marker ?m - marker ?p1 - position)
    (connected_pos ?pos1 - position ?pos2 - position)
    (reached ?m - marker)
  )

  ;; Numeric Fluents
  (:functions
    ;;(visibility ?m - marker)
  )

  ;; Durative Actions
  (:durative-action go_to_marker
    :parameters (?m - marker ?from ?to - position)
    :duration (= ?duration 1)
    :condition (and
      (at start(at_robot ?from))
      (over all(at_marker ?m ?to))
      (over all(connected_pos ?from ?to))
      ;;(over all(= (visibility ?m) 0))
    )
    :effect (and
      (at end(at_robot ?to))
      (at end(not (at_robot ?from)))
      ;;(at end(= (visibility ?m) 1))
    )
  )

  (:durative-action find_marker
    :parameters (?m - marker ?pos - position)
    :duration (= ?duration 1)
    :condition (and
      (over all(at_robot ?pos))
      (over all(at_marker ?m ?pos))
      ;;(at start(= (visibility ?m) 1))
    )
    :effect
      (at end (reached ?m))
      ;;(at end(= (visibility ?m) 2))
  )
)
