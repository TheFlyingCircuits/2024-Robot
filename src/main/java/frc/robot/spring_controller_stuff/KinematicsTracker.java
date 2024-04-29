package frc.robot.spring_controller_stuff;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

public class KinematicsTracker {
        public double position;
        public double velocity;
        public double acceleration;

        private double prevPosition;
        private double prevVelocity;
        private Timer timer = new Timer();

        // TODO: make this an average over time instead of over iteration count?
        private LinearFilter velocityFilter;
        private LinearFilter accelFilter;

        public KinematicsTracker(double initialPosition, int samplesPerMovingAverage) {
            this.position = initialPosition;
            this.velocity = 0;
            this.acceleration = 0;

            this.prevPosition = initialPosition;
            this.prevVelocity = 0;
            timer.restart();

            velocityFilter = LinearFilter.movingAverage(samplesPerMovingAverage);
            accelFilter = LinearFilter.movingAverage(samplesPerMovingAverage);
        }

        public KinematicsTracker(double initialPosition) {
            // Default moving average size of 3 iterations.
            // 8 iterations was found to introduce too much delay to the measurements,
            // as evinced by increased oscillation about the setpoint in control loops.
            this(initialPosition, 3);
        }

        public void update(double newPosition) {
            // Save information from last iteration
            // before updating for this iteration
            this.prevPosition = this.position;
            this.prevVelocity = this.velocity;

            double deltaT = timer.get();
            timer.restart();

            this.position = newPosition;

            double rawVelocity = (position - prevPosition) / deltaT;
            this.velocity = velocityFilter.calculate(rawVelocity);

            double rawAcceleration = (velocity - prevVelocity) / deltaT;
            this.acceleration = accelFilter.calculate(rawAcceleration);

            // Would either of these be more accurate/useful?
            // position = (0.5 * acceleration * deltaT * deltaT) + (prevVelocity * deltaT) + prevPosition;
            // acceleration = ( (velocity * velocity) - (prevVelocity * prevVelocity) ) / (2 * (position - prevPosition))
        }
    }
