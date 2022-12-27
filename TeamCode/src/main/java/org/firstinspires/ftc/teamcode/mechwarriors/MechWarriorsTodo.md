1. Update behaviors to no longer take in the name string. Automatically compute this instead (in the constructor).
2. Update ReverseHeading for inverse heading/speed values?
3. Update autonomous opmode to add selectable delay time (in seconds)
   1. Add a Wait behavior as the first behavior and set the wait length with this value
   2. Test it first to see what happens. Then will need to figure out how to handle button debounce
4. Create new Behavior: ConcurrentActions
   1. Takes one or more behaviors, stores them in a list, and runs them concurrently
   2. Does not complete until all contained behaviors are done
5. Update DriveHeading/RaiseLift/LowerLift behaviors to have acceleration/deceleration
   1. Could use ElapsedTimer to determine acceleration/deceleration. Always attempt to get to max speed.
   2. Could also use PID