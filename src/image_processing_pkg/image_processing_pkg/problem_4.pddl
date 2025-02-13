(define (problem block-problem-4)
  (:domain blocks)
  (:objects
    red green blue - block
    left mid right - position
  )
  (:init
    (at green left)
    (at blue mid)
    (at red right)
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