(define (domain robotFega)
(:requirements :strips :typing :adl :fluents :durative-actions)
  (:types 
    robot
    position
  )

  ;; Predicates
  (:predicates
    (at-robot ?r - robot ?p0 - position)
    (connected-pos ?pos1 ?pos2 - position)
  )

  ;; Numeric Fluents
  (:functions
    ;;(visibility ?m - marker)
  )

  ;; Durative Actions
  (:durative-action go-to-marker
    :parameters (?r - robot ?from ?to - position)
    :duration (= ?duration 1)
    :condition (and
      (at start(at-robot ?r ?from))
      (over all(connected-pos ?from ?to))
      ;;(over all(= (visibility ?m) 0))
    )
    :effect (and
      (at end(at-robot ?r ?to))
      ;;(at end(= (visibility ?m) 1))
    )
  )
)
