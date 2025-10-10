#include "software/ai/hl/stp/tactic/orbital_move_primitive.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/geom/algorithms/end_in_obstacle_sample.h"

OrbitalMovePrimitive::OrbitalMovePrimitive(
    const Robot &robot, const Point &destination, const Point &ball_position,
    const TbotsProto::MaxAllowedSpeedMode &max_allowed_speed_mode,
    const TbotsProto::ObstacleAvoidanceMode &obstacle_avoidance_mode,
    const TbotsProto::DribblerMode &dribbler_mode,
    const TbotsProto::BallCollisionType &ball_collision_type,
    const AutoChipOrKick &auto_chip_or_kick, std::optional<double> cost_override)
    : robot(robot),
      destination(destination),
      ball_position(ball_position),
      dribbler_mode(dribbler_mode),
      auto_chip_or_kick(auto_chip_or_kick),
      ball_collision_type(ball_collision_type),
      max_allowed_speed_mode(max_allowed_speed_mode),
      obstacle_avoidance_mode(obstacle_avoidance_mode)
{
    if (cost_override.has_value())
    {
        estimated_cost = cost_override.value();
    }
    else
    {   
        // TODO: Implement trajectory.generate and angular_trajectory.generate 
        // estimated_cost should be the max of their time
        // for orbital, we have dynamic angle calculations..
        estimated_cost = 0.0;
    }
}

std::pair<std::optional<TrajectoryPath>, std::unique_ptr<TbotsProto::Primitive>>
OrbitalMovePrimitive::generatePrimitiveProtoMessage(
    const World &world, const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const std::map<RobotId, TrajectoryPath> &robot_trajectories,
    const RobotNavigationObstacleFactory &obstacle_factory)
{

    // takes current world state and genearates trajectory path and protobuf message
    // that tells robot how to move

    // return a pair:
    //  TrajectoryPath: planned path robot follows
    //  TbotProto::Primitive: the control message sent to the robot


    // updateObstacles where we generate obstacle avoiding trajectory
    //      - populates obstacles vector w all obstacles robot should avoid
    //      - includes other robots, field boundaries, motion constraints, ect

    // KinematicConstraints constraints
    //      - converts speed mode enum to actual speed values
    //      - creates constraints for maximum speed, acceleration, deceleration

    // navigable_area: where robot can move .. boundaries

    // updated_start_position
    // - if stuck, find nearest point out, if destination is obstacle
    // adjust to new place --> ensure robot can reach target


    // trajectory planning: planner.findTrajectory
    // - find path from current pos to destination

    // TbotsProto::TrajectoryPathParams2D
    // - 2D trajectory proto parameters.. proto message

    // TrajectoryParamsAngular1D
    // - angular trajectory proto population
    // - ORBITAL PRIMITIVE: create angular trajectory parameters
    // set start angle (curr robot orientation) and final angle (target orientation)

    auto primitive_proto = std::make_unique<TbotsProto::Primitive>();
    return std::make_pair(std::nullopt, std::move(primitive_proto));
}

void OrbitalMovePrimitive::updateObstacles(
    const World &world, const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const std::map<RobotId, TrajectoryPath> &robot_trajectories,
    const RobotNavigationObstacleFactory &obstacle_factory)
{
    // TODO: Implement obstacle updating logic
    // would be nearly identical to MovePrimitive method
    // ball handling is different:
        // if (ball_collision_type == TbotsProto::AVOID)
        // {
        //     obstacles.push_back(
        //         obstacle_factory.createFromBallPosition(world.ball().position()));
        // }
    // dont AVOID the ball
    // don't treat as ball as an obstacle to avoid since we are orbiting around
    // the ball... get close to it??
    // maintain visual contact w ball
    // USED FOR DRIBBLING WHICH REQUIRES BEING NEAR THE BALL
}

void OrbitalMovePrimitive::getVisualizationProtos(
    TbotsProto::ObstacleList &obstacle_list_out,
    TbotsProto::PathVisualization &path_visualization_out) const
{
    // TODO: 
    // generates visualization data for debugging and monitoring
    // obstacle and path visualization
    // OBSTACLE: 
    //      iterates through obstacles robot is avoiding
    //      shows what obstacles robot is avoiding
    // PATH:
    //      creates through NUM_TRAJECTORY_VISUALIZATION_POINTS 
    //      samples trajectory at evenly spaced time intervals then
    //      converts positions to a protobuf point
    //      shows planned path the robot follows
    // orbital needs to add extra visualizations to show orbital behaviour
    // this includes ball pos, robot orientation, orbital relationship??
}

Angle OrbitalMovePrimitive::calculateAngleToFaceBall(const Point &position) const
{
    // calculates the angle the robot should be to face the ball from ANY
    // given position
    // ball_pos - pos gives us vector form robot pos to ball pos
    // .orientation() converts vector to angle
    //  return (ball_position - position).orientation();
    //     Ball (ball_position)
    //          ↑
    //          |  ← This vector: (ball_position - position)
    //          |
    //      Robot (position)
    return (ball_position - position).orientation();
}
