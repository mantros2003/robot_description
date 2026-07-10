#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <vector>

namespace gazebo {

// Structure to hold the state of each obstacle
struct ObstacleState {
	physics::ModelPtr model;
	ignition::math::Vector3d current_pos;
	ignition::math::Vector3d target_pos;
	ignition::math::Vector3d start_pos;
	double speed;
	bool moving_to_target;
};

class MovingObstacles : public WorldPlugin {

public:

	void Load(physics::WorldPtr world, sdf::ElementPtr) {
		world_ = world;
        last_time_ = world_->SimTime();

        // Define the square boundaries for randomization
        double min_bound = -4.0;
        double max_bound = 4.0;

        // Find all models with "obstacle" in the name
        for (const auto& model : world_->Models()) {
            std::string model_name = model->GetName();
            
            if (model_name.find("obstacle") != std::string::npos) {
                ObstacleState obs;
                obs.model = model;
                
                // Randomize start and target positions within the square
                obs.start_pos = ignition::math::Vector3d(
                    ignition::math::Rand::DblUniform(min_bound, max_bound),
                    ignition::math::Rand::DblUniform(min_bound, max_bound),
                    0.1 // Z-height (keep it slightly above ground)
                );

				// Ensure the spawn is far enough from other obstacles
				while (true) {
					bool flag = true;

					for (int i = 0; i < obstacles_.size(); i++) {
						if (obs.start_pos.Distance(obstacles_[i].start_pos) < 0.3) break;
						if (i == obstacles_.size() - 1 ) flag = false;
					}

					if (flag) {
						obs.start_pos = ignition::math::Vector3d(
							ignition::math::Rand::DblUniform(min_bound, max_bound),
							ignition::math::Rand::DblUniform(min_bound, max_bound),
							0.1
						);
					} else break;
				}
                
                obs.target_pos = ignition::math::Vector3d(
                    ignition::math::Rand::DblUniform(min_bound, max_bound),
                    ignition::math::Rand::DblUniform(min_bound, max_bound),
                    0.1
                );

				// Ensure a minimum ditance trajectory
				while (obs.start_pos.Distance(obs.target_pos) < 3.0) {
					obs.target_pos = ignition::math::Vector3d(
						ignition::math::Rand::DblUniform(min_bound, max_bound),
						ignition::math::Rand::DblUniform(min_bound, max_bound),
						0.1
					);
				}

                // Randomize speed between 0.5 and 1.5 m/s
                obs.speed = ignition::math::Rand::DblUniform(0.5, 1.5);
                obs.moving_to_target = true;

                // Teleport to initial start position immediately
                obs.model->SetWorldPose(ignition::math::Pose3d(obs.start_pos, ignition::math::Quaterniond(0, 0, 0)));

                obstacles_.push_back(obs);
                gzmsg << "Tracking: " << model_name 
                      << " | Speed: " << obs.speed 
                      << " | Path length: " << obs.start_pos.Distance(obs.target_pos) << "m\n";
            }
        }

        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RandomMovingObstacles::OnUpdate, this));
	}



	void OnUpdate() {
        common::Time current_time = world_->SimTime();
        double dt = (current_time - last_time_).Double();
        last_time_ = current_time;

        if (dt <= 0) return;

        double proximity_threshold = 1.0; // Distance to trigger a pause
        double goal_tolerance = 0.1;      // Distance to consider the destination reached

        for (size_t i = 0; i < obstacles_.size(); ++i) {
            ignition::math::Vector3d current_pos = obstacles_[i].model->WorldPose().Pos();
            bool should_pause = false;

            // Proximity Check
            for (size_t j = 0; j < obstacles_.size(); ++j) {
                if (i == j) continue;

                ignition::math::Vector3d other_pos = obstacles_[j].model->WorldPose().Pos();
                double distance = current_pos.Distance(other_pos);

                if (distance < proximity_threshold) {
                    // Higher index yields to lower index
                    if (i > j) {
                        should_pause = true;
                        break; 
                    }
                }
            }

            // If blocked by an obstacle, skip updating this one's position for this frame
            if (should_pause) continue;

            // 3. Linear Movement
            ignition::math::Vector3d destination = obstacles_[i].moving_to_target ? obstacles_[i].target_pos : obstacles_[i].start_pos;
            
            // Calculate distance and direction
            ignition::math::Vector3d direction = destination - current_pos;
            double dist_to_dest = direction.Length();

            // Check if reached destination
            if (dist_to_dest < goal_tolerance) {
                obstacles_[i].moving_to_target = !obstacles_[i].moving_to_target;
                continue; // Will start moving the other way on the next physics step
            }

            // Normalize direction and scale by speed and time delta
            direction.Normalize();
            ignition::math::Vector3d step = direction * (obstacles_[i].speed * dt);
            
            if (step.Length() > dist_to_dest) {
                step = direction * dist_to_dest;
            }

            ignition::math::Vector3d new_pos = current_pos + step;

            // Apply new pose
            obstacles_[i].model->SetWorldPose(
                ignition::math::Pose3d(new_pos, ignition::math::Quaterniond(0, 0, 0)));
        }
    }

private:

	physics::WorldPtr world_;
	common::Time last_time_;
	std::vector<physics::ModelPtr> obstacles_;
	event::ConnectionPtr update_connection_;


};

GZ_REGISTER_WORLD_PLUGIN(MovingObstacles)

}
