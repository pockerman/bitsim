\page bitrl_example_connect2 Connect2

Connect2 is a two player game on a board of four cells: the players take turns to put a stone in
an empty cell. It is small enough to search completely, which makes it a test bed for self-play
algorithms such as AlphaZero. In bitrl's version a player wins with two stones on the board: it
counts the stones, and does not check that they are next to each other. This
example plays it with \ref bitrl::envs::connect2::Connect2 "Connect2".

## What you need

Nothing beyond bitrl itself. See \ref bitrl_install.

## The environment

`step(action)` puts a stone of the player to move in cell `action`, and the time step carries the
valid moves left as an extra, `valid_moves`, a `std::vector<uint_t>`. Playing a cell that is taken,
or one that does not exist, throws `std::logic_error`. `move(player, action)` plays for a given
player, 1 or 2; the observation shows their stones as 1 and 2.

## Running the example

@code{.sh}
./connect2
@endcode

The example plays a cell, tries an invalid one and a taken one, wins, and then plays a game with
both players.

## The code

\include connect2.cpp
