# Co-simulation: coupled subsystems, each with its own solver, that meet every
# macro step and exchange forces and motions; loosely, or tightly, repeating a
# step until they agree. The subsystems can be threads or processes on several
# machines.
BITSIM_ADD_EXAMPLE(two_mass_oscillator SMOKE_TEST)
BITSIM_ADD_EXAMPLE(distributed_towing SMOKE_TEST)
