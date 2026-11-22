\page bitsim_example_producer_consumer A sensor thread and a queue

A sensor driver, an MQTT client or a camera usually receives its data on a thread of its own,
while the controller runs on another. The data has to pass from one thread to the other without
being lost or read half written. \ref bitsim::utils::concurrency::ConcurrentQueue
"ConcurrentQueue" does that: one thread pushes, another pops, and the pop waits for data when there
is none. bitsim's \ref bitsim::network::MqttSubscriber "MqttSubscriber" keeps the messages it
receives in one.

Here a simulated gyroscope produces a turn rate every 5 ms for 2 s on its own thread, and an
estimator integrates the rates into a heading on the main thread.

## What you need

Nothing beyond bitsim itself. See \ref bitsim_install.

## The queue

@code{.cpp}
ConcurrentQueue<GyroReading> queue;

// the sensor thread
queue.push({k, k * PERIOD, rate});
...
queue.close(); // nothing more is coming

// the estimator
auto reading = queue.pop_for(20ms); // std::nullopt if nothing came in 20 ms
@endcode

| | |
| --- | --- |
| `push(x)` | adds `x` at the back; returns false, and drops `x`, once the queue is closed |
| `pop()` | waits until there is an item, or the queue is closed and empty |
| `pop_for(timeout)` | waits at most `timeout` |
| `try_pop()` | does not wait |
| `close()` | wakes every thread that waits; they get what is left, then `std::nullopt` |

## Running the example

@code{.sh}
./producer_consumer
@endcode

## Output

@code
1. A consumer that keeps up
  fast estimator: 400 readings in 2.00 s, heading 1.0006 rad (true 1.0000), at most 0 readings waiting, 0 waits timed out

2. A consumer that falls behind
  slow estimator: 400 readings in 3.24 s, heading 1.0006 rad (true 1.0000), at most 152 readings waiting, 0 waits timed out

3. Draining what is there
  try_pop took 5 readings without waiting; after close(), push() gives false
@endcode

The fast estimator takes each reading as it arrives, so the queue never holds more than the
reading being taken. The slow one needs 8 ms per reading, longer than the 5 ms between them: the
readings pile up, up to 152 of them, and the estimate is 1.2 s late, but no reading is lost and the
heading is the same. A real controller that falls behind like that should drain the queue with
`try_pop()` and use the newest reading, as part 3 does, rather than work through stale ones.

The waits of `pop_for` would time out if the sensor stopped sending; here it never does. When it
is done, `close()` tells the estimator, which stops once the queue is empty.

## Things to try

1. Stop the sensor after 1 s without closing the queue. How does the estimator notice?
2. Run two estimators on two threads, popping from the same queue. Which readings does each get,
   and what is the heading of each?
3. Make the slow estimator drain the queue and integrate only the newest reading, scaled by the
   time since the last one it used.

## The code

\include producer_consumer.cpp
