#include "../basic-abstract-game.h"
#include "../assetgen.h"
#include "road_network.cpp"
#include <set>
#include <queue>
#include <cmath>

// python -m procgen.interactive --env-name testgame

const std::string NAME = "testgame";

const int COMPLETION_BONUS = 10.0f;
const int POSITIVE_REWARD = 1.0f;

const int FISH = 2;

const float FISH_MIN_R = .25;
const float FISH_MAX_R = 2;

const int FISH_QUOTA = 30;
const int WPS_GOAL = 1000;

const float MAXVTHETA = 15 * PI / 180;
const float MIXRATEROT = 0.5f;

const float DV_NORM = .05;
const float THROTTLE_NORM = .05;
const float STEER_NORM = .5;

const int MIN_NODES_THRESHOLD = 5; //15;

const int NUM_NPCS = 0;//20;
const bool DRAW_COMPASS = false;

bool USE_AUTOPILOT = false;

class TestGame : public BasicAbstractGame {
  public:
    int fish_eaten = 0;
    float r_inc = 0.0;
    RoadNetwork road_network;

    std::vector<Waypoint> upcoming_waypoints;  
    int current_route_end_node_id;
    int last_node_id;
    float angle_to_target_wp;
    int wps_visited = 0;
    int collision = 0;
    int waypoint_infraction = 0;

    float RAND_ROTATE;

    float front_angle = 0;
    
    float autopilot_steer;
    float autopilot_throttle;

    float last_applied_steer;
    float last_applied_throttle;
    float _last_applied_steer = 0.f;
    float _last_applied_throttle = 0.f;
    float _last_speed = 0.f;

    float current_speed = 0.f;
    float dv = 0.f;

    int successful_stop = 0;


	float TRAJ_WIDTH = 2;

    std::mt19937 stdgen;

    RandGen actually_randgen;

    std::shared_ptr<Entity> traj_marker;

    std::vector<std::shared_ptr<Entity>> npcs;
    std::vector<std::vector<Waypoint>> upcoming_waypoints_npcs;  
    std::vector<int> current_route_end_node_id_npcs;
    std::vector<int> last_node_id_npcs;
    std::vector<pair<pair<float,float>,pair<float,float>>> trajectory_npcs;
    vector<bool> brake_npcs;


    TestGame()
        : BasicAbstractGame(NAME) {
        timeout = 6000;
    }

    // Decides to use procgen background if you don't provide any imgs
    // We're drawing on the imgs directly, so need them to be of constant size,
    // or need to make sure our scaling dynamically responds to img size. We weren't doing
    // this before, causing the need for magic numbers in our scaling. Either use procgen 
    // backgrounds of uniform 500., or make scaling dynamic to first read from the background img dim.
    // Or crop all imgs dynamically to be 500 x 500. DONE. comment out for procgen, leave in for imgs

    void load_background_images() override {
        //main_bg_images_ptr = &water_backgrounds;
        //main_bg_images_ptr = &platform_backgrounds;
    }

    void asset_for_type(int type, std::vector<std::string> &names) override {
        if (type == PLAYER) {
            //names.push_back("misc_assets/fishTile_072.png");
            names.push_back("misc_assets/playerShip1_red.png");
        } else if (type == FISH) {
            // names.push_back("misc_assets/fishTile_074.png");
            // names.push_back("misc_assets/fishTile_078.png");
            //names.push_back("misc_assets/fishTile_080.png");
            names.push_back("misc_assets/playerShip1_red.png");
        }
    }

    void handle_agent_collision(const std::shared_ptr<Entity> &obj) override {
        BasicAbstractGame::handle_agent_collision(obj);

        collision = 1;
    }

    // Taken from cave flyer
    //void set_action_xy(int move_action) override {
    void set_action_xy(float move_action_steer, float move_action_throttle) override {

        // Controls from LAST step
        last_applied_steer = _last_applied_steer;
        last_applied_throttle = _last_applied_throttle;

        determine_npc_braking();

        bool hit_wp = false;
        bool requires_wp_reset = false;

        autopilot_steer = get_autopilot_steer(agent, upcoming_waypoints, last_node_id, current_route_end_node_id, hit_wp, requires_wp_reset);
        if (hit_wp){
            wps_visited += 1;
            step_data.reward += 1;
        } else if (requires_wp_reset) {
            //step_data.done = true;
            waypoint_infraction = 1;
            //reset_agent_wps();
        }

        autopilot_steer /= STEER_NORM;

        action_vrot = USE_AUTOPILOT ? autopilot_steer : move_action_steer;

        // Caching. These will be the "last step" controls for next step
        _last_applied_steer = action_vrot;

        action_vrot *= STEER_NORM;

        
        autopilot_throttle = get_acceleration(agent, false);
        autopilot_throttle /= THROTTLE_NORM;

        current_speed = sqrt(agent->vx*agent->vx + agent->vy*agent->vy);

        dv = (current_speed - _last_speed) / DV_NORM; 

        dv = std::clamp(dv, -3.f, 3.f);

        // Stop signs
        successful_stop = 0;
        if (upcoming_waypoints.at(0).type==WP_STOP_SIGN){
            if (current_speed<.01) {
                //cout << " Stopped for light, nice job! ";
                successful_stop = 1;
            } else {
                //cout << " still need to stop for this sign... ";
                autopilot_throttle=0.;
            }
        } 

        float agent_acc = USE_AUTOPILOT ? autopilot_throttle : action_throttle;

        _last_applied_throttle = agent_acc;

        agent_acc *= THROTTLE_NORM;

        set_velocity(agent, agent_acc);

        _last_speed = current_speed;

        // NPCs
        for (int i = 0; i<npcs.size(); i++) {
            bool hit_wp_npc = false; // not used
            bool requires_wp_reset_npc = false; // not used
            float vrot = get_autopilot_steer(npcs.at(i), upcoming_waypoints_npcs.at(i), last_node_id_npcs.at(i), 
                                                    current_route_end_node_id_npcs.at(i), hit_wp_npc, requires_wp_reset_npc);

            npcs.at(i)->vrot = MIXRATEROT * vrot;
            npcs.at(i)->vrot += MIXRATEROT * MAXVTHETA * vrot;

            float acc = get_acceleration(npcs.at(i), brake_npcs.at(i));
            set_velocity(npcs.at(i), acc);
        }
    }

    float get_autopilot_steer(const shared_ptr<Entity> &ent, 
                                vector<Waypoint> &_upcoming_waypoints, 
                                int &_last_node_id, 
                                int &_current_route_end_node_id,
                                bool &_hit_wp,
                                bool &_requires_reset_wps)
    {
        Waypoint wp = _upcoming_waypoints.front();
        float target_wp_x = wp.x;
        float target_wp_y = wp.y;

        float dx = target_wp_x - ent->x;
        float dy = target_wp_y - ent->y;

        float m = fmod(ent->rotation, PI*2);
        float agent_abs_angle = m >= 0 ? m : (PI*2 + m); // should always be zero to 6.28

        float wp_abs_angle = atan2(dx,dy);
        wp_abs_angle = wp_abs_angle >= 0 ? wp_abs_angle : (PI*2 + wp_abs_angle); // should always be zero to 6.28
        
        float angle_to_wp = wp_abs_angle - agent_abs_angle;

        // Turn whichever direction is faster
        angle_to_wp = angle_to_wp > PI ? angle_to_wp-2*PI : (angle_to_wp < -PI ? angle_to_wp+2*PI : angle_to_wp);
        
        float dist = sqrt(dx*dx + dy*dy);

        if (dist < 3) {
            _upcoming_waypoints.erase(_upcoming_waypoints.begin());
            _hit_wp = true;
        } 
        // This complicates score. Mixes performance across two metrics. Trying to be too clever.
        // else if (dist > 8) {
        //     _requires_reset_wps = true;
        // }

        int ADD_MORE_WPS_TRIGGER = 30;
        if (_upcoming_waypoints.size() < ADD_MORE_WPS_TRIGGER) {
            add_wps_to_route(_upcoming_waypoints, _last_node_id, _current_route_end_node_id);
        }

        // Don't turn when stopped
        float _current_speed = sqrt(ent->vx*ent->vx + ent->vy*ent->vy);
        if (_current_speed < .2) {
            return 0;
        }

        float move_thresh = .03;
        float steer_scale = angle_to_wp / (PI/2); // 90 deg angle results in scale of one.
        float steer = 2.; //TODO vary this
        float autopilot_action_vrot = angle_to_wp > move_thresh ? steer_scale*steer : 
                    (angle_to_wp < -move_thresh ? steer_scale*steer : 0);

        return autopilot_action_vrot;
    }
        
                                

    pair<pair<float,float>,pair<float,float>> get_trajectory(shared_ptr<Entity> &ent, vector<Waypoint> &ent_wps)
    {

        // Trajectory starts slightly behind entity
        // float BACK_DIST = 1.; 
        // float theta = -1 * ent->rotation + PI / 2;
        // float xx = ent->x + (BACK_DIST * cos(theta + PI));
        // float yy = ent->y + (BACK_DIST * sin(theta + PI));

        pair<float,float> traj_start;
        traj_start.first = ent->x;
        traj_start.second = ent->y;

        int TRAJ_WP = 2;
        float TRAJ_DIST = 5;

        float dx = ent_wps.at(TRAJ_WP).x - agent->x;
        float dy = ent_wps.at(TRAJ_WP).y - agent->y;
        //float theta = -1 * atan2(dx,dy) + PI/2;
        float theta = -1 * ent->rotation + PI / 2;

        pair<float,float> traj_end;
        traj_end.first = ent->x + TRAJ_DIST * cos(theta);
        traj_end.second = ent->y + TRAJ_DIST * sin(theta);


        pair<pair<float,float>,pair<float,float>> trajectory;
        trajectory.first = traj_start;
        trajectory.second = traj_end;
        return trajectory;
    }

    pair<float,float> get_intersection(pair<float,float> A, pair<float,float> B, pair<float,float> C, pair<float,float> D) {
        // Line AB represented as a1x + b1y = c1
        float a = B.second - A.second;
        float b = A.first - B.first;
        float c = a*(A.first) + b*(A.second);
        // Line CD represented as a2x + b2y = c2
        float a1 = D.second - C.second;
        float b1 = C.first - D.first;
        float c1 = a1*(C.first)+ b1*(C.second);
        float det = a*b1 - a1*b;

        float x = (b1*c - b*c1)/det;
        float y = (a*c1 - a1*c)/det;

        pair<float,float> ret;
        ret.first = x;
        ret.second = y;
        
        return ret;
    }

    pair<float,float> _trajectories_intersect(pair<pair<float,float>,pair<float,float>> &traj1, 
                                    pair<pair<float,float>,pair<float,float>> &traj2,
                                    bool & has_intersection ) 
    {
        pair<float,float> p1 = traj1.first;
        pair<float,float> p2 = traj1.second;
        pair<float,float> q1 = traj2.first;
        pair<float,float> q2 = traj2.second;

        has_intersection = (((q1.first-p1.first)*(p2.second-p1.second) - (q1.second-p1.second)*(p2.first-p1.first))
				* ((q2.first-p1.first)*(p2.second-p1.second) - (q2.second-p1.second)*(p2.first-p1.first)) < 0)
				&&
			(((p1.first-q1.first)*(q2.second-q1.second) - (p1.second-q1.second)*(q2.first-q1.first))
				* ((p2.first-q1.first)*(q2.second-q1.second) - (p2.second-q1.second)*(q2.first-q1.first)) < 0);

        pair<float,float> intersection_pt;
        if (has_intersection) {
            intersection_pt = get_intersection(p1,p2,q1,q2);
        } 
        return intersection_pt; // returns null ass thing if not intersection. Changes has_intersection in place. 
    }

    pair<pair<float,float>,pair<float,float>> get_parallel_traj(pair<pair<float,float>,pair<float,float>> &traj, std::string side)
    {
            pair<float,float> p1 = traj.first;
            pair<float,float> p2 = traj.second;
			float angle = atan2(p1.second - p2.second, p1.first - p2.first);
			angle += side=="right" ? PI/2. : -PI/2;
            angle = angle < 0 ? 2*PI + angle : angle;

            pair<pair<float,float>,pair<float,float>> ret;
            pair<float,float> p1_ret;
            pair<float,float> p2_ret;

			p1_ret.first = p1.first + (TRAJ_WIDTH/2 * cos(angle));
		    p1_ret.second = p1.second + (TRAJ_WIDTH/2 * sin(angle));
            p2_ret.first = p2.first + (TRAJ_WIDTH/2 * cos(angle));
		    p2_ret.second = p2.second + (TRAJ_WIDTH/2 * sin(angle));

            ret.first = p1_ret;
            ret.second = p2_ret;
            return ret;
    }

    pair<pair<float,float>,pair<float,float>> get_traj_lateral(pair<pair<float,float>,pair<float,float>> &traj)
    {
            pair<float,float> p1 = traj.first;
            pair<float,float> p2 = traj.second;
			float angle = atan2(p1.second - p2.second, p1.first - p2.first);

            pair<float,float> left;
            pair<float,float> right;

            left.first = p1.first + (TRAJ_WIDTH/2 * cos(angle-(PI/2)));
            left.second = p1.second + (TRAJ_WIDTH/2 * sin(angle-(PI/2)));

            right.first = p1.first + (TRAJ_WIDTH/2 * cos(angle+(PI/2)));
            right.second = p1.second + (TRAJ_WIDTH/2 * sin(angle+(PI/2)));

            pair<pair<float,float>,pair<float,float>> ret;
            ret.first = left;
            ret.second = right;
            return ret;
    }

    pair<float,float> trajectories_intersect(pair<pair<float,float>,pair<float,float>> &traj1, 
                            pair<pair<float,float>,pair<float,float>> &traj2,
                            bool & has_intersection) 
    {
        pair<pair<float,float>,pair<float,float>> traj1_lateral = get_traj_lateral(traj1);
        pair<pair<float,float>,pair<float,float>> traj2_lateral = get_traj_lateral(traj2);

        vector<pair<pair<float,float>,pair<float,float>>> ent1_trajs {traj1, traj1_lateral};
        vector<pair<pair<float,float>,pair<float,float>>> ent2_trajs {traj2, traj2_lateral};

        pair<float,float> intersection_pt;
        for (pair<pair<float,float>,pair<float,float>> t1 : ent1_trajs) {
            for (pair<pair<float,float>,pair<float,float>> t2 : ent2_trajs) {
                intersection_pt = _trajectories_intersect(t1,t2,has_intersection);
                // return immediately if have intersection
                if (has_intersection) {
                    return intersection_pt;
                }
            }
        }
        // No intersection
        return intersection_pt;
    }

    float get_distance(pair<float,float> &p1, pair<float,float> &p2)
	{
		float dx = p1.first - p2.first;
		float dy = p1.second - p2.second;
		return sqrt(dx*dx + dy*dy);
	}

    void determine_npc_braking()
    {
        pair<pair<float,float>,pair<float,float>> agent_trajectory = get_trajectory(agent, upcoming_waypoints);
        pair<float,float> agent_loc;
        agent_loc.first = agent->x;
        agent_loc.second = agent->y;

        for (int i = 0; i < npcs.size(); i++) {
            
            pair<pair<float,float>,pair<float,float>> ent2_trajectory = trajectory_npcs.at(i);
            pair<float,float> ent2_loc;
            ent2_loc.first = npcs.at(i)->x;
            ent2_loc.second = npcs.at(i)->y;

            bool _has_intersection = false;
            pair<float,float> intersection_pt = trajectories_intersect(agent_trajectory, ent2_trajectory, _has_intersection);

            if (_has_intersection){
                bool agent_yield = get_distance(agent_loc, intersection_pt) > get_distance(ent2_loc, intersection_pt);
                
                if (!agent_yield){
                    brake_npcs.at(i)=true;
                } else {
                    brake_npcs.at(i)=false;
                }

            } else {
                brake_npcs.at(i)=false;
            }
        } 

    }

    // Used for the agent
    bool should_brake(shared_ptr<Entity> &ent)
    {
        if (ent->type == PLAYER) {

            pair<pair<float,float>,pair<float,float>> ent1_trajectory = get_trajectory(ent, upcoming_waypoints);
            pair<float,float> ent1_loc;
            ent1_loc.first = ent->x;
            ent1_loc.second = ent->y;
            
            for (int i = 0; i < npcs.size(); i++) {
                
                pair<pair<float,float>,pair<float,float>> ent2_trajectory = trajectory_npcs.at(i);
                pair<float,float> ent2_loc;
                ent2_loc.first = npcs.at(i)->x;
                ent2_loc.second = npcs.at(i)->y;

                bool _has_intersection = false;
                pair<float,float> intersection_pt = trajectories_intersect(ent1_trajectory, ent2_trajectory, _has_intersection);

                if (_has_intersection){

                    bool ent1_yield = get_distance(ent1_loc, intersection_pt) > get_distance(ent2_loc, intersection_pt);
                    
                    if (ent1_yield){
                        //agent->rx = .6;
                        //agent->ry = .6;
                        return true;
                    } else {
                        //npcs.at(i)->rx = .6;
                        //npcs.at(i)->ry = .6;

                        return false;
                    }

                }
                agent->rx = .4;
                agent->ry = .4;
                npcs.at(i)->rx = .4;
                npcs.at(i)->ry = .4;
            } 
            return false;
        } else { // if it's a NPC
            return false;
        }
    }

    float get_acceleration(shared_ptr<Entity> &ent, bool must_brake) 
    {
        float SPEED_LIMIT = .35; // TODO vary this
        float THROTTLE = .3; // TODO vary this
        float _current_speed = sqrt(ent->vx*ent->vx + ent->vy*ent->vy);

        float acceleration = _current_speed > SPEED_LIMIT ? 0 : (should_brake(ent) ? 0. : (SPEED_LIMIT-_current_speed)*THROTTLE);
        if (must_brake){
            acceleration = 0;
        }

        return acceleration;
    }

    void set_velocity(shared_ptr<Entity> &ent, float acceleration) 
    {
        float _current_speed = sqrt(ent->vx*ent->vx + ent->vy*ent->vy);

        float theta = -1 * ent->rotation + PI / 2;

        float SPEED_DEGRADE = .85f; //TODO vary this
        _current_speed = (_current_speed * SPEED_DEGRADE) + acceleration;

        ent->vy = _current_speed * sin(theta);
        ent->vx = _current_speed * cos(theta);
    }

    // Gives 'coasting' and momentum behavior to thrust. Like a spaceship...
    // keeping off for simplicity for now
    
    // void update_agent_velocity() override {
    //     float v_scale = get_agent_acceleration_scale();

    //     agent->vx += mixrate * maxspeed * action_vx * v_scale * .2;
    //     agent->vy += mixrate * maxspeed * action_vy * v_scale * .2;

    //     decay_agent_velocity();
    // }


    void add_wps_to_route(vector<Waypoint> &_upcoming_waypoints, int &_last_node_id, int &_current_route_end_node_id) {

        // Changes end node ix in place
        vector<Waypoint> wps = road_network.get_more_wps(_last_node_id, _current_route_end_node_id);

        for (Waypoint wp : wps) {
            _upcoming_waypoints.push_back(wp);
        }
    }

    void make_road(QPainter &painter) {

        road_network.generate_network(); // flag if too small. Get another network. 

        //std::cout << "  NUM NODES IN NETWORK " << road_network.nodes.size() << " ";
        while (road_network.nodes.size() < MIN_NODES_THRESHOLD){
            //std::cout << " Regenerating network bc too small ";
            road_network.generate_network(); 
        }

        road_network.draw(painter);
            
    }

    // TODO make the center point 3 units in front of the agent
    // void choose_center(float &cx, float &cy) {

    //     float m = fmod(agent->rotation, PI*2.);
    //     float agent_abs_angle = m >= 0 ? m : (PI*2. + m); // should always be zero to 6.28

    //     float xx = agent->x + (5 * cos(agent_abs_angle));
	// 	float yy = agent->y + (5 * sin(agent_abs_angle));

    //     cx = xx; //agent->x;
    //     cy = yy; //agent->y;
        
    // }



    void game_draw(QPainter &painter, const QRect &rect) override {

        // // This is where we're rotating to make agent always up
        float _w = rect.width();
        float scale = (_w / main_width);

        float vd = _w/2.; // This bit, the last term, is a magic fucking thing (not in good way) i put together by viewing and guessing what correction was needed. I do not know why this correction was needed. Beware.
        float ax_painter_dim = agent->x * scale - (-vd+(agent->x/25)*vd); // it's lined up when agent is in center, and increasingly off the closer to the edges agent gets. This corrects that.
        float ay_painter_dim = (main_width-agent->y)*scale - (-vd+((50-agent->y)/25)*vd);

        float m = fmod(agent->rotation, PI*2.);

        m += RAND_ROTATE;

        float agent_abs_angle = m >= 0 ? m : (PI*2. + m); // should always be zero to 6.28

        // Comment out this for no rotate
        painter.translate(ax_painter_dim, ay_painter_dim);
        painter.rotate(-agent_abs_angle * (360./(2.*PI)));
        painter.translate(-ax_painter_dim, -ay_painter_dim);

        
        // Use the get_theta etc from jumper compass. Draw compass in corner to see location of forward wp.
        // Get angle to MACRO wp.
        int TARGET_WAYPOINT_IX = 10;//20; // TODO vary this. Want low enough so not ambiguos but high enough so doesn't enable PID
        Waypoint wp = upcoming_waypoints[TARGET_WAYPOINT_IX];
        float target_wp_x = wp.x;
        float target_wp_y = wp.y;

        // TODO add in bias here to mimic miscalibrated GPS

        float dx = target_wp_x - agent->x;
        float dy = target_wp_y - agent->y;

        float mm = fmod(agent->rotation, PI*2);
        agent_abs_angle = mm >= 0 ? mm : (PI*2 + mm); // should always be zero to 6.28
        float wp_abs_angle = atan2(dx,dy);
        wp_abs_angle = wp_abs_angle >= 0 ? wp_abs_angle : (PI*2 + wp_abs_angle); // should always be zero to 6.28
        
        angle_to_target_wp = wp_abs_angle - agent_abs_angle;
        
        // Turn whichever direction is faster
        angle_to_target_wp = angle_to_target_wp > PI ? angle_to_target_wp-2*PI : 
                                (angle_to_target_wp < -PI ? angle_to_target_wp+2*PI : angle_to_target_wp);


        //std::cout << " Angle to wp "<< angle_to_target_wp <<" ";

        // Draw line, for debugging

        
        BasicAbstractGame::game_draw(painter, rect);
        //painter.restore();


        // have to go after basic game draw. But basic game draw has to be after
        // we rotate and translate the painter


        // 
        // Compass line
        if (DRAW_COMPASS) {
            float compass_line_width = 1.5; // not being drawn bc of translation errors in the smaller obs img
            float compass_line_length = 10;
            
            float compass_base_x = ax_painter_dim + 30*cos(agent->rotation+RAND_ROTATE + PI/4);
            float compass_base_y = ay_painter_dim + 30*sin(agent->rotation+RAND_ROTATE + PI/4);

            //float compass_base_x = ax_painter_dim;
            //float compass_base_y = ay_painter_dim;

            painter.setBrush(QColor(200, 200, 200));
            painter.drawEllipse(
                        compass_base_x - compass_line_length, 
                        compass_base_y - compass_line_length,
                        compass_line_length*2,
                        compass_line_length*2);

            painter.setPen(QPen(QColor(0, 00, 150), compass_line_width, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
            painter.drawLine(compass_base_x, 
                                compass_base_y, 
                                compass_base_x+compass_line_length*cos(wp_abs_angle-PI/2), 
                                (compass_base_y + compass_line_length*sin(wp_abs_angle-PI/2)));
        }


                            


        // Refresh all NPC trajectories
        trajectory_npcs.clear();
        for (int i = 0; i < npcs.size(); i++) {
            pair<pair<float,float>,pair<float,float>> traj = get_trajectory(npcs.at(i), upcoming_waypoints_npcs.at(i));

            // float ax = npcs.at(i)->x * scale - (-vd+(npcs.at(i)->x/25)*vd); // it's lined up when agent is in center, and increasingly off the closer to the edges agent gets. This corrects that.
            // float ay = (main_width-npcs.at(i)->y)*scale - (-vd+((50-npcs.at(i)->y)/25)*vd);

            // pair<float,float> t_end = traj.second;
            // float tx = t_end.first * scale - (-vd+(t_end.first/25)*vd); // it's lined up when agent is in center, and increasingly off the closer to the edges agent gets. This corrects that.
            // float ty = (main_width-t_end.second)*scale - (-vd+((50-t_end.second)/25)*vd);

            // painter.drawLine(ax,ay,tx,ty);


            trajectory_npcs.push_back(traj);
        } 

        // // For debugging trajectories for negotiation
        // pair<pair<float,float>,pair<float,float>> agent_trajectory = get_trajectory(agent, upcoming_waypoints);
        // //agent_trajectory = get_traj_lateral(agent_trajectory);
        // pair<float,float> p1 = agent_trajectory.first;
        // pair<float,float> p2 = agent_trajectory.second;

        // traj_marker->x = p2.first;
        // traj_marker->y = p2.second;

    }

    void game_reset() override {
        BasicAbstractGame::game_reset();

        AssetGen bggen(&rand_gen);
        QColor road_color = bggen.get_rand_color(options.color_theme_road);
        QColor sidewalk_color = bggen.get_rand_color(options.color_theme_sidewalk);

          /* initialize random seed: */
        srand (time(NULL));
        int ss = rand();

        //cout << " " << ss << " ";
        actually_randgen.seed(ss);

        // This is the line that makes it not learn.
        RAND_ROTATE = 0; 
        //RAND_ROTATE = rand_gen.randrange(-PI/2, PI/2);

        front_angle = RAND_ROTATE;

        road_network = RoadNetwork(rand_gen, road_color, sidewalk_color);

        options.center_agent = true;
        options.use_generated_assets = true;
        fish_eaten = 0;
        wps_visited = 0;

        float start_r = .4;

        if (options.distribution_mode == EasyMode) {
            start_r = 1;
        }

        r_inc = (FISH_MAX_R - start_r) / FISH_QUOTA;

        agent->rx = start_r; // TODO vary these
        agent->ry = start_r;
        //agent->y = 1 + agent->ry; // This was overriding ego position start location as set in general game

        /////////////////////////
        // This paints the road on the BACKGROUND img

        std::shared_ptr<QImage> background_image = main_bg_images_ptr->at(background_index);
        QPainter painter(background_image.get());

        painter.translate(0, 500); //+ 65 // these are translated in painter xy space. From top left, more like pixel dist
        float SCALER = 1.;//512.0f / 100.0f; // background main dim / map dim
        painter.scale(SCALER,-SCALER);// negative scale flips the dim
        
        make_road(painter);

        //////////////////////////
        // Waypoints apparatus

        // Random node to start
        
        current_route_end_node_id = road_network.nodes[0].node_id;//road_network.nodes[rand_gen.randn(road_network.nodes.size())].node_id;
        last_node_id = -1; // init to nothin

        upcoming_waypoints.clear();
        for (int i; i < 10; i++) {
            add_wps_to_route(upcoming_waypoints, last_node_id, current_route_end_node_id);
        }

        // Set agent initial location and rotation
        agent->x = upcoming_waypoints[0].x;
        agent->y = upcoming_waypoints[0].y;

        Waypoint wp = upcoming_waypoints[5]; // orienting towards slight distance in the future. Initial wp is too close.
        float target_wp_x = wp.x;
        float target_wp_y = wp.y;

        float dx = target_wp_x - agent->x;
        float dy = target_wp_y - agent->y;

        float wp_abs_angle = atan2(dx,dy);
        wp_abs_angle = wp_abs_angle >= 0 ? wp_abs_angle : (PI*2 + wp_abs_angle); // should always be zero to 6.28
        
        agent->rotation = wp_abs_angle; //actually_randgen.randrange(0, 2*PI); //

        // NPCs
        npcs.clear();
        current_route_end_node_id_npcs.clear();
        last_node_id_npcs.clear();
        upcoming_waypoints_npcs.clear();
        brake_npcs.clear();

         //ceil((road_network.nodes.size()-1) / .8);
        for (int i = 0; i<NUM_NPCS; i++) {
//NOTE THIS HAS BEEN REMOVED!!!!
            //upcoming_waypoints_npcs.at(i).clear();
            for (int ii=0; ii < 10; ii++) {
                add_wps_to_route(upcoming_waypoints_npcs.at(i), last_node_id_npcs.at(i), current_route_end_node_id_npcs.at(i));
            }

            npcs.at(i)->x = upcoming_waypoints_npcs.at(i).at(0).x;
            npcs.at(i)->y = upcoming_waypoints_npcs.at(i).at(0).y;

            Waypoint wp_npc = upcoming_waypoints_npcs.at(i)[5]; // orienting towards slight distance in the future. Initial wp is too close.
            float target_wp_x_npc = wp_npc.x;
            float target_wp_y_npc = wp_npc.y;

            float dx_npc = target_wp_x_npc - npcs.at(i)->x;
            float dy_npc = target_wp_y_npc - npcs.at(i)->y;

            float wp_abs_angle_npc = atan2(dx_npc,dy_npc);
            wp_abs_angle_npc = wp_abs_angle_npc >= 0 ? wp_abs_angle_npc : (PI*2 + wp_abs_angle_npc); // should always be zero to 6.28
            
            npcs.at(i)->rotation = wp_abs_angle_npc;
        }


        traj_marker = std::make_shared<Entity>(0, 0, 0, 0, .2, FISH);
        traj_marker->render_z=1;
        traj_marker->smart_step=true;
        entities.push_back(traj_marker);
    }

    // Agent can move within this space. This is the fenced area within the background image that is drawn
    // entity placement happens within this grid. We want our roads to be placed in this space.
    void choose_world_dim() override {
        main_width = 50;
        main_height = 50;
    }

    void game_step() override {

        collision = 0; // This will get set to one if have collision
        waypoint_infraction = 0;

        BasicAbstractGame::game_step();

        if (wps_visited >= WPS_GOAL) {
            step_data.done = true;
            step_data.reward += COMPLETION_BONUS;
            step_data.level_complete = true;
        }
    }

    void observe() override {
        Game::observe();
        *(float_t *)(info_bufs[info_name_to_offset.at("autopilot_steer")]) = (float) std::clamp(autopilot_steer, -5.f, 5.f);
        *(float_t *)(info_bufs[info_name_to_offset.at("autopilot_throttle")]) = (float) std::clamp(autopilot_throttle, -5.f, 5.f);

        *(float_t *)(info_bufs[info_name_to_offset.at("last_applied_steer")]) = (float) 0.0; //std::clamp(last_applied_steer, -5.f, 5.f);
        *(float_t *)(info_bufs[info_name_to_offset.at("last_applied_throttle")]) = (float) 0.0; // std::clamp(last_applied_throttle, -5.f, 5.f);

        *(float_t *)(info_bufs[info_name_to_offset.at("current_speed")]) = (float) std::clamp(current_speed, -5.f, 5.f);
        *(float_t *)(info_bufs[info_name_to_offset.at("dv")]) = (float) dv;

        *(float_t *)(info_bufs[info_name_to_offset.at("front_angle")]) = (float) (front_angle); // making it less important in the loss



        *(float_t *)(info_bufs[info_name_to_offset.at("angle_to_wp")]) = (float) angle_to_target_wp < -.5 ? -1.0 : (angle_to_target_wp > .5 ? 1.0 : 0.0); 
        *(int32_t *)(info_bufs[info_name_to_offset.at("collision")]) = (int) collision;
        *(int32_t *)(info_bufs[info_name_to_offset.at("waypoint_infraction")]) = (int) waypoint_infraction;

        *(int32_t *)(info_bufs[info_name_to_offset.at("successful_stop")]) = (int) successful_stop;

    }

    void serialize(WriteBuffer *b) override {
        BasicAbstractGame::serialize(b);
        b->write_int(fish_eaten);
        b->write_float(r_inc);
    }

    void deserialize(ReadBuffer *b) override {
        BasicAbstractGame::deserialize(b);
        fish_eaten = b->read_int();
        r_inc = b->read_float();
    }
};

REGISTER_GAME(NAME, TestGame);
