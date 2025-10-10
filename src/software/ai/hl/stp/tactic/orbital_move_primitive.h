#pragma once

#include "proto/primitive/primitive_types.h"
#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/trajectory/trajectory_planner.h"
#include "software/world/world.h"

class OrbitalMovePrimitive : public Primitive
{
   public:
    /**
     * Create an Orbital Move Primitive Message
     * This primitive moves the robot to a destination while continuously facing the ball
     *
     * @param robot Robot running this primitive
     * @param destination Destination position of the robot
     * @param ball_position Position of the ball to face during movement
     * @param max_allowed_speed_mode Max allowed speed the robot can move at
     * @param obstacle_avoidance_mode How aggressively to avoid obstacles
     * @param dribbler_mode Dribbler mode during this primitive
     * @param ball_collision_type Ball collision type specifying if collision with the
     * ball is allowed
     * @param auto_chip_or_kick Whether auto chip or kick is enabled and the target
     * distance/speed
     * @param cost_override optionally override the cost of the move primitive, defaults
     * to the total duration of reaching the destination (ignoring obstacles)
     */
    OrbitalMovePrimitive(const Robot &robot, const Point &destination, 
                        const Point &ball_position,
                        const TbotsProto::MaxAllowedSpeedMode &max_allowed_speed_mode,
                        const TbotsProto::ObstacleAvoidanceMode &obstacle_avoidance_mode,
                        const TbotsProto::DribblerMode &dribbler_mode,
                        const TbotsProto::BallCollisionType &ball_collision_type,
                        const AutoChipOrKick &auto_chip_or_kick,
                        std::optional<double> cost_override = std::nullopt);

    ~OrbitalMovePrimitive() override = default;

    /**
     * Gets the primitive proto message
     *
     * @param world Current state of the world
     * @param motion_constraints Motion constraints to consider
     * @param robot_trajectories A map of the all friendly robots' known trajectories
     * @param obstacle_factory Obstacle factory to use for generating obstacles
     * @return A pair of the found trajectory (optional) and the primitive proto message
     */
    std::pair<std::optional<TrajectoryPath>, std::unique_ptr<TbotsProto::Primitive>>
    generatePrimitiveProtoMessage(
        const World &world,
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const std::map<RobotId, TrajectoryPath> &robot_trajectories,
        const RobotNavigationObstacleFactory &obstacle_factory) override;

    /**
     * Fill the obstacle list and path visualization with the obstacles and path
     * of this primitive
     *
     * @param obstacle_list_out Reference to the ObstacleList proto to add obstacles to
     * @param path_visualization_out Reference to the PathVisualization proto to add path
     */
    void getVisualizationProtos(
        TbotsProto::ObstacleList &obstacle_list_out,
        TbotsProto::PathVisualization &path_visualization_out) const override;

   private:
    /**
     * Helper for filling the `obstacles` vector with the obstacles that the primitive
     * should avoid
     *
     * @param world Current state of the world
     * @param motion_constraints Motion constraints
     * @param robot_trajectories A map of the friendly robots' known trajectories
     * @param obstacle_factory Obstacle factory to use
     */
    void updateObstacles(const World &world,
                         const std::set<TbotsProto::MotionConstraint> &motion_constraints,
                         const std::map<RobotId, TrajectoryPath> &robot_trajectories,
                         const RobotNavigationObstacleFactory &obstacle_factory);

    /**
     * Calculate the angle to face the ball from the given position
     *
     * @param position Position to calculate angle from
     * @return Angle to face the ball
     */
    Angle calculateAngleToFaceBall(const Point &position) const;

    Robot robot;
    Point destination;
    Point ball_position;

    TbotsProto::DribblerMode dribbler_mode;
    AutoChipOrKick auto_chip_or_kick;

    TbotsProto::BallCollisionType ball_collision_type;
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
    TbotsProto::ObstacleAvoidanceMode obstacle_avoidance_mode;

    // List of all obstacles that the robot should avoid
    std::vector<ObstaclePtr> obstacles;
    // List of only the motion constraint obstacles that the robot should avoid
    std::vector<ObstaclePtr> field_obstacles;

    BangBangTrajectory2D trajectory;
    std::optional<TrajectoryPath> traj_path;
    BangBangTrajectory1DAngular angular_trajectory;
    TrajectoryPlanner planner;

    constexpr static unsigned int NUM_TRAJECTORY_VISUALIZATION_POINTS = 10;
};
