#include "../basic-abstract-game.h"
#include "../assetgen.h"
#include "road_network.cpp"
#include <set>
#include <queue>
#include <cmath>

const std::string NAME = "testgame";

const int COMPLETION_BONUS = 10.0f;
const int POSITIVE_REWARD = 1.0f;

const int FISH = 2;

const float FISH_MIN_R = .25;
const float FISH_MAX_R = 2;

const int FISH_QUOTA = 30;
const int WPS_GOAL = 3000;

class TestGame : public BasicAbstractGame {
  public:
    int fish_eaten = 0;
    float r_inc = 0.0;
    std::vector<pair<float,float>> upcoming_waypoints;
    RoadNetwork road_network;
    int current_route_end_node_id;
    int last_node_id;
    float angle_to_target_wp;
    int wps_visited = 0;



    TestGame()
        : BasicAbstractGame(NAME) {
        timeout = 6000;
    }

    // Decides to use procgen background if you don't provide any imgs
    // We're drawing on the imgs directly, so need them to be of constant size,
    // or need to make sure our scaling dynamically responds to img size. We weren't doing
    // this before, causing the need for magic numbers in our scaling. Either use procgen 
    // backgrounds of uniform 500., or make scaling dynamic to first read from the background img dim.
    // Or crop all imgs dynamically to be 500 x 500. 

    // void load_background_images() override {
    //     main_bg_images_ptr = &water_backgrounds;
    // }

    void asset_for_type(int type, std::vector<std::string> &names) override {
        if (type == PLAYER) {
            //names.push_back("misc_assets/fishTile_072.png");
            names.push_back("misc_assets/playerShip1_red.png");
        } else if (type == FISH) {
            names.push_back("misc_assets/fishTile_074.png");
            names.push_back("misc_assets/fishTile_078.png");
            names.push_back("misc_assets/fishTile_080.png");
        }
    }

    void handle_agent_collision(const std::shared_ptr<Entity> &obj) override {
        BasicAbstractGame::handle_agent_collision(obj);

        if (obj->type == FISH) {
            if (obj->rx > agent->rx) {
                step_data.done = true;
            } else {
                step_data.reward += POSITIVE_REWARD;
                obj->will_erase = true;
                agent->rx += r_inc;
                agent->ry += r_inc;
                fish_eaten += 1;
            }
        }
    }

    // Taken from cave flyer
    //void set_action_xy(int move_action) override {
    void set_action_xy(float move_action) override {
        float acceleration = .6; //move_action % 3 - 1;
        if (acceleration < 0)
            acceleration *= 0.33f;

        float theta = -1 * agent->rotation + PI / 2;

        pair<float,float> wp = upcoming_waypoints.front();
        float target_wp_x = wp.first;
        float target_wp_y = wp.second;

        float dx = target_wp_x - agent->x;
        float dy = target_wp_y - agent->y;

        // Get angle to wp. Cumbersome. Surely a pithy one-liner somewhere for all of this...
        float m = fmod(agent->rotation, PI*2);
        float agent_abs_angle = m >= 0 ? m : (PI*2 + m); // should always be zero to 6.28

        float wp_abs_angle = atan2(dx,dy);
        wp_abs_angle = wp_abs_angle >= 0 ? wp_abs_angle : (PI*2 + wp_abs_angle); // should always be zero to 6.28
        
        float angle_to_wp = wp_abs_angle - agent_abs_angle;

        // Turn whichever direction is faster
        angle_to_wp = angle_to_wp > PI ? angle_to_wp-2*PI : (angle_to_wp < -PI ? angle_to_wp+2*PI : angle_to_wp);
        

        float dist = sqrt(dx*dx + dy*dy);

        if (dist < 2) {
            upcoming_waypoints.erase(upcoming_waypoints.begin());
            wps_visited += 1;
            step_data.reward += 1;
        };

        int ADD_MORE_WPS_TRIGGER = 30;
        if (upcoming_waypoints.size() < ADD_MORE_WPS_TRIGGER) {
            add_wps_to_route();
        }

        //std::cout << "    dist to wp " << dist; 
        action_vy = acceleration * sin(theta);
        action_vx = acceleration * cos(theta);
        float move_thresh = .1;
        float steer_scale = angle_to_wp / (PI/2); // 90 deg angle results in scale of one.
        float steer = 2.; 
        action_vrot = angle_to_wp > move_thresh ? steer_scale*steer : 
                    (angle_to_wp < -move_thresh ? steer_scale*steer : 0); //move_action / 3 - 1;
    }

    // Gives 'coasting' and momentum behavior to thrust. Like a spaceship...
    // keeping off for simplicity for now
    
    // void update_agent_velocity() override {
    //     float v_scale = get_agent_acceleration_scale();

    //     agent->vx += mixrate * maxspeed * action_vx * v_scale * .2;
    //     agent->vy += mixrate * maxspeed * action_vy * v_scale * .2;

    //     decay_agent_velocity();
    // }


    void add_wps_to_route() {

        // Changes end node ix in place
        vector<pair<float,float>> wps = road_network.get_more_wps(last_node_id, current_route_end_node_id);

        for (pair<float,float> wp : wps) {
            upcoming_waypoints.push_back(wp);
        }
    }

    void make_road(QPainter &painter) {

        road_network.generate_network();

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
        float scale = (500. / main_width);

        float vd = 250; // This bit, the last term, is a magic fucking thing (not in good way) i put together by viewing and guessing what correction was needed. I do not know why this correction was needed. Beware.
        float ax_painter_dim = agent->x * scale - (-vd+(agent->x/25)*vd); // it's lined up when agent is in center, and increasingly off the closer to the edges agent gets. This corrects that.
        float ay_painter_dim = (main_width-agent->y)*scale - (-vd+((50-agent->y)/25)*vd);

        float m = fmod(agent->rotation, PI*2.);
        float agent_abs_angle = m >= 0 ? m : (PI*2. + m); // should always be zero to 6.28
        
        painter.translate(ax_painter_dim, ay_painter_dim);
        painter.rotate(-agent_abs_angle * (360./(2.*PI)));
        painter.translate(-ax_painter_dim, -ay_painter_dim);

        
        // Use the get_theta etc from jumper compass. Draw compass in corner to see location of forward wp.
        // Get angle to MACRO wp.
        int TARGET_WAYPOINT_IX = 20;
        pair<float,float> wp = upcoming_waypoints[TARGET_WAYPOINT_IX];
        float target_wp_x = wp.first;
        float target_wp_y = wp.second;

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

        // have to go after basic game draw. But basic game draw has to be after
        // we rotate and translate the painter

        painter.setPen(QPen(QColor(12, 163, 39), 3, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));

        painter.drawLine(ax_painter_dim, 
                            ay_painter_dim, 
                            ax_painter_dim+100*cos(wp_abs_angle-PI/2), 
                            (ay_painter_dim + 100*sin(wp_abs_angle-PI/2)) );
    }

    void game_reset() override {
        BasicAbstractGame::game_reset();

        road_network = RoadNetwork(rand_gen);

        options.center_agent = true;
        //options.use_generated_assets = true;
        fish_eaten = 0;
        wps_visited = 0;

        float start_r = .5;

        if (options.distribution_mode == EasyMode) {
            start_r = 1;
        }

        r_inc = (FISH_MAX_R - start_r) / FISH_QUOTA;

        agent->rx = start_r;
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
            add_wps_to_route();
        }

        // Set agent initial location and rotation
        agent->x = upcoming_waypoints[0].first;
        agent->y = upcoming_waypoints[0].second;

        pair<float,float> wp = upcoming_waypoints[5]; // orienting towards slight distance in the future. Initial wp is too close.
        float target_wp_x = wp.first;
        float target_wp_y = wp.second;

        float dx = target_wp_x - agent->x;
        float dy = target_wp_y - agent->y;

        float wp_abs_angle = atan2(dx,dy);
        wp_abs_angle = wp_abs_angle >= 0 ? wp_abs_angle : (PI*2 + wp_abs_angle); // should always be zero to 6.28
        
        agent->rotation = wp_abs_angle;
    }

    // Agent can move within this space. This is the fenced area within the background image that is drawn
    // entity placement happens within this grid. We want our roads to be placed in this space.
    void choose_world_dim() override {
        main_width = 50;
        main_height = 50;
    }

    void game_step() override {
        BasicAbstractGame::game_step();

        if (wps_visited >= WPS_GOAL) {
            step_data.done = true;
            step_data.reward += COMPLETION_BONUS;
            step_data.level_complete = true;
        }
    }

    void observe() override {
        Game::observe();
        *(float_t *)(info_bufs[info_name_to_offset.at("autopilot_steer")]) = action_vrot;
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
