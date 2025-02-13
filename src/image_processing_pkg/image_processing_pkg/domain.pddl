(define (domain blocks)
    (:requirements :strips :typing)
    (:types
        block position
    )
    (:predicates
        (at ?b - block ?p - position)
        (adjacent ?p1 - position ?p2 - position)
    )
    (:action swap
        :parameters (?b1 - block ?b2 - block ?p1 - position ?p2 - position)
        :precondition (and
            (at ?b1 ?p1)
            (at ?b2 ?p2)
            (adjacent ?p1 ?p2)
        )
        :effect (and
            (not (at ?b1 ?p1))
            (not (at ?b2 ?p2))
            (at ?b1 ?p2)
            (at ?b2 ?p1)
        )
    )
)