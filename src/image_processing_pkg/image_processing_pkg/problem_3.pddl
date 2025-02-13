(define (problem block-problem-3)
    (:domain blocks)
    (:objects
        red green blue - block
        left mid right - position
    )
    (:init
        (at green left)
        (at red mid)
        (at blue right)
        (adjacent left mid)
        (adjacent mid left)
        (adjacent mid right)
        (adjacent right mid)
    )
    (:goal
        (and
            (at red left)
            (at green mid)
            (at blue right)
        )
    )
)