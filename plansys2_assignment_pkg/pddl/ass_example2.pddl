(define (domain robotFega)
(:requirements :strips :typing :adl :fluents :durative-actions)
  (:types 
    marker 
    position
  )

  ;; Predicates
  (:predicates
    (at-robot ?p0 - position)
    (at-marker ?m - marker ?p1 - position)
    (connected-pos ?pos1 - position ?pos2 - position)
  )

  ;; Numeric Fluents
  (:functions
    ;;(visibility ?m - marker)
  )

  ;; Durative Actions
  (:durative-action go-to-marker
    :parameters (?m - marker ?from ?to - position)
    :duration (= ?duration 1)
    :condition (and
      (at start(at-robot ?from))
      (at start(at-marker ?m ?to))
      (over all(connected-pos ?from ?to))
      ;;(over all(= (visibility ?m) 0))
    )
    :effect (and
      (at end(at-robot ?to))
      ;;(at end(= (visibility ?m) 1))
    )
  )

  (:durative-action find-marker
    :parameters (?m - marker ?pos - position)
    :duration (= ?duration 1)
    :condition (and
      (at start(at-robot ?pos))
      (at start(at-marker ?m ?pos))
      ;;(at start(= (visibility ?m) 1))
    )
    :effect
      (at end(not(at-robot ?pos)))
      ;;(at end(= (visibility ?m) 2))
  )
)
