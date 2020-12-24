#include "../basic-abstract-game.h"
#include "../assetgen.h"
#include "pathfinder.cpp"
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

class TestGame : public BasicAbstractGame {
  public:
    int fish_eaten = 0;
    float r_inc = 0.0;
    std::vector<pair<float,float>> wps;
    int start_node = 0;
    int target_node;
    vector<Node*> nodes; 
    vector<QPainterPath> roads;

    TestGame()
        : BasicAbstractGame(NAME) {
        timeout = 6000;

        main_width = 20;
        main_height = 20;
    }

    void load_background_images() override {
        main_bg_images_ptr = &water_backgrounds;
    }

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
    void set_action_xy(int move_action) override {
        float acceleration = .4; //move_action % 3 - 1;
        if (acceleration < 0)
            acceleration *= 0.33f;

        float theta = -1 * agent->rotation + PI / 2;

        pair<float,float> wp = wps[0];
        float target_wp_x = wp.first;
        float target_wp_y = wp.second;

        float dx = target_wp_x - agent->x;
        float dy = target_wp_y - agent->y;
        float angle_to_wp = atan2(dx,dy) - agent->rotation;

        float dist = sqrt(dx*dx + dy*dy);

        if (dist < 2.0) {
            wps.erase(wps.begin());
        };

        if (wps.size() < 10) {
            add_wps_to_route();
        }

        //std::cout << "    dist to wp " << dist; 
        action_vy = acceleration * sin(theta);
        action_vx = acceleration * cos(theta);
        action_vrot = angle_to_wp > 0.0 ? .8 : -.8; //move_action / 3 - 1;
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

        std::vector<pair<int, int>> c = nodes[target_node]->children;
        pair<int, int> target_child = c[rand_gen.randn(c.size())];
        target_node = target_child.first;
        QPainterPath path_to_target_node = roads[target_child.second];

        for (float i=.0f; i<1.0f; i+=.05) {
            QPointF q = path_to_target_node.pointAtPercent(i);
            pair<float, float> wp;
            wp.first = (q.x() / 80.0f) * main_width;
            wp.second = (q.y() / 80.0f) * main_height;
            wps.push_back(wp);
        }
    }

    void make_road(QPainter &painter) {

        QPointF a = QPointF(30,10);
        QPointF b = QPointF(50,10);
        QPointF c = QPointF(10,30);
        QPointF d = QPointF(30,30);
        QPointF f = QPointF(70,30);
        QPointF j = QPointF(30,70);
        QPointF g = QPointF(30,50);
        QPointF i = QPointF(70,50);

        QPainterPath p0;
        p0.moveTo(a);
        p0.lineTo(b);

        QPainterPath p1;
        p1.moveTo(b);
        p1.cubicTo(QPointF(60,10), QPointF(70,20), f);

        QPainterPath p2;
        p2.moveTo(f);
        p2.lineTo(d);

        QPainterPath p3;
        p3.moveTo(d);
        p3.lineTo(a);

        QPainterPath p4;
        p4.moveTo(d);
        p4.lineTo(c);

        QPainterPath p5;
        p5.moveTo(c);
        p5.cubicTo(QPointF(10,20), QPointF(20,10), a);

        QPainterPath p6;
        p6.moveTo(c);
        p6.cubicTo(QPointF(10,60), QPointF(20,70), j);

        QPainterPath p7;
        p7.moveTo(j);
        p7.cubicTo(QPointF(40,70), QPointF(70,60), i);

        QPainterPath p8;
        p8.moveTo(i);
        p8.lineTo(f);

        QPainterPath p9;
        p9.moveTo(j);
        p9.lineTo(g);

        QPainterPath p10;
        p10.moveTo(g);
        p10.lineTo(d);

        QPainterPath p11;
        p11.moveTo(g);
        p11.lineTo(i);


        painter.setPen(QPen(QColor(79, 106, 25), 5, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
        // painter.setBrush(QColor(122, 163, 39));

        painter.drawPath(p0);
        painter.drawPath(p1);
        painter.drawPath(p2);
        painter.drawPath(p3);
        painter.drawPath(p4);
        painter.drawPath(p5);
        painter.drawPath(p6);
        painter.drawPath(p7);
        painter.drawPath(p8);
        painter.drawPath(p9);
        painter.drawPath(p10);
        painter.drawPath(p11);

        roads = {p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11};

        // QPointF sample_point = p2.pointAtPercent(.99f);
        // painter.setPen(QPen(QColor(122, 163, 39), 3, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
        // painter.drawPoint(sample_point);

        // int target_node = rand_gen.randn(8);

        target_node = 6;
        std::vector<QPainterPath> paths_to_take {p0, p1, p2, p4, p6, p7};
        for (auto &p : paths_to_take) {
            for (float i=.0f; i<1.0f; i+=.05) {
                QPointF q = p.pointAtPercent(i);
                pair<float, float> wp;
                wp.first = (q.x() / 80.0f) * main_width;
                wp.second = (q.y() / 80.0f) * main_height;
                wps.push_back(wp);
            }
        }

        // Conceptual graph

        Node* A = new Node(0);
        A->add_child(1, 0);
        nodes.push_back(A);

        Node* B = new Node(1);
        B->add_child(4, 1);
        nodes.push_back(B);

        Node* C = new Node(2);
        C->add_child(0, 5);
        C->add_child(7, 6);
        nodes.push_back(C);

        Node* D = new Node(3);
        D->add_child(0, 3);
        D->add_child(2, 4);
        nodes.push_back(D);

        Node* F = new Node(4);
        F->add_child(3, 2);
        nodes.push_back(F);

        Node* G = new Node(5);
        G->add_child(3, 10);
        G->add_child(6, 11);
        nodes.push_back(G);

        Node* I = new Node(6);
        I->add_child(4, 8);
        nodes.push_back(I);

        Node* J = new Node(7);
        J->add_child(5, 9);
        J->add_child(6, 7);
        nodes.push_back(J);

        // // Start node
        // int s = 0; 

        // vector<int> path(v.size()); 
        // vector<int> dist = dijkstraDist(v, s, path); 

        // // Loop to print the distance of 
        // // every node from source vertex 
        // for (int i = 0; i < dist.size(); i++) { 
        //     if (dist[i] == infi) { 
        //         cout << i << " and " << s 
        //             << " are not connected"
        //             << endl; 
        //         continue; 
        //     } 
        //     cout << "Distance of " << i 
        //         << "th vertex from source vertex "
        //         << s << " is: "
        //         << dist[i] << endl; 
        // }
        // std::cout << " Target node " << target_node;
        
        // printPath(path, target_node, s);
    }

    // void game_draw(QPainter &painter, const QRect &rect) override {

    //     float rot = 60; //(agent->rotation / 2*PI) *360;

    //     float d = 512;
    //     float n = (d/2) * sqrt(2);
    //     float a = sqrt(2*pow(n,2) - (2*(n*n*cos(rot))));

    //     float dx = a/2;
    //     float dy = a * sqrt(3)/2;
        
    //     //std::cout << "  dx " << dx << "dy " << dy;
    //     painter.rotate(rot);
    //     painter.translate(dx/2,-dy/2);
    //     BasicAbstractGame::game_draw(painter, rect);

    // }

    void game_reset() override {
        BasicAbstractGame::game_reset();

        options.center_agent = true;
        fish_eaten = 0;

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

        //painter.fillRect(main_rect, QColor(100, 100, 100));
        //painter.rotate(180);
        painter.translate(0, 512 + 70); // these are translated in painter xy space. From top left, more like pixel dist
        float SCALER = 512.0f / 70.0f; // background main dim / map dim
        painter.scale(SCALER,-SCALER);
        //painter.setWorldTransform(painter.worldTransform().transposed());
        

        make_road(painter);

        agent->x = wps[0].first;
        agent->y = wps[0].second;
    }

    // Agent can move within this space. This is the fenced area within the background image that is drawn
    // entity placement happens within this grid. We want our roads to be placed in this space.
    void choose_world_dim() override {
        main_width = 50;
        main_height = 50;
    }

    void game_step() override {
        BasicAbstractGame::game_step();

        // if (rand_gen.randn(10) == 1) {
        //     float ent_r = (FISH_MAX_R - FISH_MIN_R) * pow(rand_gen.rand01(), 1.4) + FISH_MIN_R;
        //     float ent_y = rand_gen.rand01() * (main_height - 2 * ent_r);
        //     float moves_right = rand_gen.rand01() < .5;
        //     float ent_vx = (.15 + rand_gen.rand01() * .25) * (moves_right ? 1 : -1);
        //     float ent_x = moves_right ? -1 * ent_r : main_width + ent_r;
        //     int type = FISH;
        //     auto ent = add_entity(ent_x, ent_y, ent_vx, 0, ent_r, type);
        //     choose_random_theme(ent);
        //     match_aspect_ratio(ent);
        //     ent->is_reflected = !moves_right;
        //     cout << "spawn loc " << ent_x << " " << ent_y;
        // }

        if (fish_eaten >= FISH_QUOTA) {
            step_data.done = true;
            step_data.reward += COMPLETION_BONUS;
            step_data.level_complete = true;
        }

        // if (action_vx > 0)
        //     agent->is_reflected = false;
        // if (action_vx < 0)
        //     agent->is_reflected = true;
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
