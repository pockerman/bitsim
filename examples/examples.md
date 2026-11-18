\page bitrl_examples bitrl examples

# bitrl examples

Each example lives in `examples/example_N/` and is built with the library when
`ENABLE_EXAMPLES` is `ON`. Some need optional dependencies; the page of each example
says which. See \ref bitrl_install for how to build bitrl.

## Reinforcement learning environments

The Gymnasium environments run on the <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a>
server, which the examples talk to over HTTP.

- \ref bitrl_example_1
- \ref bitrl_example_2
- \ref bitrl_example_3
- \ref bitrl_example_4

## Rigid body simulation with Chrono

These need bitrl built with `ENABLE_CHRONO=ON`.

- \ref bitrl_example_11
- \ref bitrl_example_12
- \ref bitrl_example_13
- \ref bitrl_example_14
- \ref bitrl_example_24

## State estimation

- \ref bitrl_example_21
- \ref bitrl_example_22

## Robotics course

The lessons build on each other, so read them in order.

- \ref bitrl_example_25
- \ref bitrl_example_26
- \ref bitrl_example_27
- \ref bitrl_example_28
- \ref bitrl_example_29
- \ref bitrl_example_30
- \ref bitrl_example_31
- \ref bitrl_example_32
- \ref bitrl_example_33
