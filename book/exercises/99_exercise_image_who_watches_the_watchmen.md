
# Exercise: Who watches the watchmen? (optional) {#exercise-watchmen status=draft}

## Skills learned

* Awareness that this is a losing game.

## Instructions

A good exercise is writing `image-ops-tester` yourself.

However, we already gave you a copy of `image-ops-tester`, which you
used in the previous step, so there is not much of a challenge. So, let's go one level up, and consider...

## `image-ops-tester-tester` specification

Write a program `image-ops-tester-tester` that tests whether
a program conforms to the specification of a `image-ops-tester`
given in [](#image-ops-tester-specification).

The `image-ops-tester-tester` program is called as follows:

    $ image-ops-tester-tester ![candidate-image-ops-tester]

This must return:

- 0 if the candidate conforms to the specification;
- 1 if it doesn't;
- another error code if other errors arise.

## Testing it works with `image-ops-tester-tester-tester`

We provide you with a helpful program called `image-ops-tester-tester-tester`
that makes sure that a candidate script conforms to the specification of
an `image-ops-tester-tester`.

Use it as follows:

    $ image-ops-tester-tester-tester ![candidate image-ops-tester-tester]

This should return 0 if everything is ok, or different than 0 otherwise.

## Bottom line {nonumber=1}

Even if things are tested, you can never be sure that the tests themselves work.

<!-- TODO: Validation and testing -->
