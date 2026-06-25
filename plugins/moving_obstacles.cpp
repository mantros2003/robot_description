#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class MovingObstacles : public WorldPlugin {

public:

	void Load(physics::WorldPtr world, sdf::ElementPtr) {

		world_ = world;

		obstacle1_ =
		    world_->ModelByName("obstacle1");

		obstacle2_ =
		    world_->ModelByName("obstacle2");

		update_connection_ =
		    event::Events::ConnectWorldUpdateBegin(
		        std::bind(&MovingObstacles::OnUpdate, this));
	}



	void OnUpdate() {

		double t = world_->SimTime().Double();
		double x = 2.5*sin(0.1*t);
		double y = 2.5*cos(0.1*t);
		
		obstacle1_->SetWorldPose(
		    ignition::math::Pose3d(
		        x,
		        0,
		        0.1,
		        0,0,0));

		obstacle2_->SetWorldPose(
		    ignition::math::Pose3d(
		        0,
		        y,
		        0.1,
		        0,0,0));
	}

private:

	physics::WorldPtr world_;
	physics::ModelPtr obstacle1_;
	physics::ModelPtr obstacle2_;

	event::ConnectionPtr update_connection_;

};

GZ_REGISTER_WORLD_PLUGIN(MovingObstacles)

}
