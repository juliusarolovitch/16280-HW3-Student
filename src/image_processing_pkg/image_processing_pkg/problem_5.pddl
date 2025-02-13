(define (problem block-problem-5)
    (:domain blocks)
    (:objects
        red green blue - block
        left mid right - position
    )
    (:init
        (at blue left)
        (at red mid)
        (at green right)
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