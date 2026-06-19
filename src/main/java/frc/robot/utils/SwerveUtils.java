package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveUtils {
    public static double stepTowards(double current, double target, double stepsize) {
        if (Math.abs(current - target) <= stepsize) {
            return target;
        } else if (target < current) {
            return current - stepsize;
        } else {
            return current + stepsize;
        }
    }

    public static double rotateToPose(Pose2d pose1, Pose2d pose2) {
        double xDiff = pose2.getX() - pose1.getX();
        double yDiff = pose2.getY() - pose1.getY();

        return Math.atan2(yDiff, xDiff);
    }

    public static double stepTowardsCircular(double current, double target, double stepsize) {
        current = wrapAngle(current);
        target = wrapAngle(target);
        double stepDirection = Math.signum(target - current);
        double difference = Math.abs(current - target);
        if (difference <= stepsize) {
            return target;
        } else if (difference > Math.PI) {
            if (current + 2 * Math.PI - target < stepsize
                    || target + 2 * Math.PI - current < stepsize) {
                return target;
            } else {
                return wrapAngle(
                        current - stepDirection * stepsize);
            }
        } else {
            return current + stepDirection * stepsize;
        }
    }

    public static double angleDifference(double angleA, double angleB) {
        double difference = Math.abs(angleA - angleB);
        return (difference > Math.PI) ? 2 * Math.PI - difference : difference;
    }

    public static double wrapAngle(double angle) {
        double twoPi = 2 * Math.PI;
        if (angle == twoPi) {
            return 0.0;
        } else if (angle > twoPi) {
            return angle - twoPi * Math.floor(angle / twoPi);
        } else if (angle < 0.0) {
            return angle + twoPi * (Math.floor(-angle / twoPi) + 1);
        } else {
            return angle;
        }
    }
}
