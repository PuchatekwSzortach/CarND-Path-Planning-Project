# Path Planning project - writeup

In this document I'll shortly describe major components of my system.
At basic level it can be split into two parts - trajectories generation and cost generation, so that best trajectory is selected.

### Trajectories generation

Every second model generates a set of new trajectories reusing last half a second of previous trajectory, with very first trajectory being set manually. About 5 different speeds, ranging from target speed of 45mph to half that are used, as well as one d-position for each lane.

Final states are computed as follows:
- for s-coordinate - initial state and desired velocity are provided, system calculates accelertion needed to achieve that velocity, then scales it down if required by ride smoothness constraints. Finally position at end state is computed. 
- for d-coordinate - as for s-coordinates, except desired end position rather than speed is provided as input

A small caveat is that for roads with sharp turns desired velocity is set a bit lower. Due to inaccuracies in xy-sd coordinates translations, vehicles moving in outer lanes of some sharper turns are scored as going over speed limit even when their s-trajectories in fact are within limits. Therefore I needed to bring down speed a bit on sharp turns. This would ideally be handled with a cost function, but turned out to be easier to solve in trajectory generation step.

Once initial and all final states are prepared, minimum jerk trajectories are computed.
These trajectories are then smoothly blended with reused part of previous trajectory.

### Cost function

Cost function is made of following terms:
- target speed cost
- collision cost
- safety distance cost
- lane change cost
- speeding cost

#### Target speed cost
This term encourages vehicles to drive near target velocity, penalizing them for not doing so.

#### Collision cost and safety distance cost
These are the real brains of the car, allowing it to decide when to change a lane or brake. Their logics are identical, but impose somewhat different costs. Collision cost puts huge cost on any trajectory leading to collisions. Safety distance encourages trajectories that keep generous spacing from other vehicles. Both functions take into account such nuances as 'you don't need to worry about vehicles driving behind you in your own lane (it's their job not to hit you), vehicles ahed of you going faster than you don't pose big risk', etc.

#### Lane change cost
A small cost that encourages vehicle to stick with lane choice it made during previous planning. This helps to prevent vehicle from initiating a lane change and then switching to a different lane with similar low cost.

#### Speeding cost
Penalizes vehicle for going over speed limit.

A video of my model in action can be found at:
https://www.youtube.com/watch?v=FWN1N7YFPTQ

I kept plenty of debugging output in my code - it helps to understand model's reasoning.
