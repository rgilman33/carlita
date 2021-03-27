
#include "../randgen.h"
#include "../basic-abstract-game.h"
#include "../assetgen.h"
#include <cmath>
#include <algorithm>
#include <vector>



using namespace std; 

float LANE_WIDTH = 8; //12;

// for waypoint types
const int WP_NORMAL = 0;
const int WP_STOP_SIGN = 1;
const int WP_AVOIDING_OBSTACLE = 2;

const float OBSTACLE_PROB = 0.f;


class Node;
class Edge;

struct Waypoint {
	float x;
	float y;
	int type = WP_NORMAL; 

	float painter_x;
	float painter_y;
};

class Node { 
public: 
	int node_id; 
	float m_x;
	float m_y;
	QPointF q_point;

	Node(){}
	Node(float x, float y) : m_x(x), m_y(y)
	{ 
		this->node_id = node_id; 
		this->q_point = QPointF(x,y);
	} 
}; 

class Edge { 
public: 
	int m_n1; 
	int m_n2;
	Node n1_node;
	Node n2_node;
	QPointF b1;
	QPointF b2;
	QPainterPath m_q_path;
	vector<Waypoint> m_waypoints;
	int edge_type = 0;
	bool ends_in_stopsign = false;
	bool has_obstacle = false;
	pair<float,float> obstacle_location;

	Edge(Node &n1, Node &n2) : m_n1 (n1.node_id), m_n2 (n2.node_id), n1_node (n1), n2_node (n2)
	{
		b1 = QPointF(n1.m_x, n1.m_y);
		b2 = QPointF(n2.m_x, n2.m_y);
	} 

	void create_wps ()
	{
		// Make waypoints as x, y coords. Half a lane width to the right of the center line
		int num_waypoints = ceil(m_q_path.length() / 5.);
		float wp_dist = 1./num_waypoints;
		for (float i=(wp_dist*1.5); i<(1.0-(wp_dist*1.5)); i+=wp_dist) {
			QPointF slightly_previous_point = m_q_path.pointAtPercent(i-(wp_dist/10.f));
			QPointF q_point = m_q_path.pointAtPercent(i);
			float angle = atan2((slightly_previous_point.y() - q_point.y()), (slightly_previous_point.x() - q_point.x() ));
			angle += 3.14/2.;
			int edge_distance = 0;//LANE_WIDTH * .75f;
			float xx = q_point.x() + (edge_distance * cos(angle));
			float yy = q_point.y() + (edge_distance * sin(angle));


			Waypoint wp;
			wp.x = xx * (50./500.);
			wp.y = yy * (50./500.); 
			wp.painter_x = xx;
			wp.painter_y = yy; 

			m_waypoints.push_back(wp);
		}

		int s = m_waypoints.size();	

		if (ends_in_stopsign) {
			if (s > 10) {
				for (int i=4; i < 6; i++) {
					m_waypoints.at(s-i).type = WP_STOP_SIGN;
				}

			} else {
				ends_in_stopsign = false;
			}
		}

		if (has_obstacle) {
			if (s > 14) {
				obstacle_location.first = m_waypoints.at(8).painter_x;
				obstacle_location.second = m_waypoints.at(8).painter_y;
				for (int i=10; i > 8; i--) {
					Waypoint prev_wp = m_waypoints.at(i-1);
					m_waypoints.at(i).type = WP_AVOIDING_OBSTACLE;

					float angle = atan2(prev_wp.painter_y-m_waypoints.at(i).painter_y, prev_wp.painter_x-m_waypoints.at(i).painter_x);
					angle -= 3.14/2.;
					float avoid_dist = LANE_WIDTH * 1.6;

					m_waypoints.at(i).painter_x = m_waypoints.at(i).painter_x + avoid_dist*cos(angle);
					m_waypoints.at(i).painter_y = m_waypoints.at(i).painter_y + avoid_dist*sin(angle);
					m_waypoints.at(i).x = m_waypoints.at(i).painter_x  * (50./500.);
					m_waypoints.at(i).y = m_waypoints.at(i).painter_y  * (50./500.);
				}
			} else {
				has_obstacle = false;
			}
		}
	}
};

class RoadNetwork { 
public:  
	vector<Node> nodes; 
	vector<Edge> edges;
	int node_id_counter = 0; // This must always be perfectly incremented. Brittle.
	RandGen rand_gen;

	float width = 500.0f;
	float height = 500.0f;

	float MIN_INTERSECTION_ANGLE = 60.0f;
	float BASE_EDGE_DISTANCE = width / 8.0f;
	float MIN_MERGE_DISTANCE = BASE_EDGE_DISTANCE * 1.8f;
	float MIN_ALLOWABLE_DIST = BASE_EDGE_DISTANCE * 1.f;

	float MARGIN = width / 20.f;
	
	int NUM_GROW_ITERS = 500;

	RoadNetwork () {}
	RoadNetwork (RandGen & rand_gen_in) : rand_gen (rand_gen_in) {}

	int add_node(Node &node) 
	{ 
		node.node_id = node_id_counter;
		nodes.push_back(node); 
		node_id_counter += 1;
		return node.node_id;
	} 

	void add_edge(Edge &edge)
	{
		edges.push_back(edge);
	}

	vector<int> get_neighboring_nodes (Node &n) 
	{
		vector<int> neighboring_nodes;
		for (Edge &e : edges) {
			if (e.m_n1==n.node_id){
				neighboring_nodes.push_back(e.m_n2);
			} else if (e.m_n2==n.node_id) {
				neighboring_nodes.push_back(e.m_n1);
			}
		}
		return neighboring_nodes;
	}

	int GetAngleABC( Node &a, Node &b, Node &c )
	{
		pair<float,float> ab;// = { b.x - a.x, b.y - a.y };
		ab.first = b.m_x - a.m_x;
		ab.second = b.m_y - a.m_y;

		pair<float,float> cb; //= { b.x - c.x, b.y - c.y };
		cb.first = b.m_x - c.m_x;
		cb.second = b.m_y - c.m_y;

		float dot = (ab.first * cb.first + ab.second * cb.second); // dot product
		float cross = (ab.first * cb.second - ab.second * cb.first); // cross product

		float alpha = atan2(cross, dot);

		float angle = (int) abs(floor(alpha * 180. / 3.14159 + 0.5));

		return angle;
	}

	float get_distance(Node &n1, Node &n2)
	{
		float dx = n1.m_x - n2.m_x;
		float dy = n1.m_y - n2.m_y;
		return sqrt(dx*dx + dy*dy);
	}

	float det(float dx1, float dx2, float dy1, float dy2) 
	{
		return (dx1 * dy2) - (dy1 * dx2);
	}


	bool isIntersecting(Node& p1, Node& p2, Node& q1, Node& q2) {
		return (((q1.m_x-p1.m_x)*(p2.m_y-p1.m_y) - (q1.m_y-p1.m_y)*(p2.m_x-p1.m_x))
				* ((q2.m_x-p1.m_x)*(p2.m_y-p1.m_y) - (q2.m_y-p1.m_y)*(p2.m_x-p1.m_x)) < 0)
				&&
			(((p1.m_x-q1.m_x)*(q2.m_y-q1.m_y) - (p1.m_y-q1.m_y)*(q2.m_x-q1.m_x))
				* ((p2.m_x-q1.m_x)*(q2.m_y-q1.m_y) - (p2.m_y-q1.m_y)*(q2.m_x-q1.m_x)) < 0);
	}

	bool intersects_existing_edge(Node &n1, Node &n2) 
	{
		for (Edge &e : edges) {

			// If edges share a node, don't compare bc don't want 'intersection' flagged at join point
			// TODO is this doing anything
			if (e.m_n1==n1.node_id || e.m_n2==n1.node_id){continue;}
			
			int n3_id = e.m_n1;
			int n4_id = e.m_n2;

			// TODO why the hell would this ever happen? Happens every blue moon, which is too often... Hack to allow continuing dev
			if (n3_id < 0  || n4_id < 0) {
				cout << " what the hell unrealistic node id! ";
				return true;
			}

			Node &n3 = get_node(n3_id);
			Node &n4 = get_node(n4_id); // THIS DOESN"T WORK. IX != ID.

			if (isIntersecting(n1, n2, n3, n4)) { 
				return true;
			}
		}
		return false;
	}

	bool angle_too_small(Node &n1, Node &n2)
	// the first angle is the ref
	{	
		vector<int> neighboring_nodes = get_neighboring_nodes(n1);
		for (int node_id : neighboring_nodes) { // TODO these should also be incoming
			if (node_id==n2.node_id) {continue;}

			if (node_id<0){
				cout << " what the hell unrealistic node id! ";
				return true;
			}
			Node o = get_node(node_id);
			float a = GetAngleABC(n2, n1, o);
			if (a < MIN_INTERSECTION_ANGLE) {
				return true;
			}
		}
		return false;
	}

	bool outside_bounds(float x, float y) {
		return x < MARGIN || x > width-MARGIN || y < MARGIN || y > height-MARGIN;
	}

	void trim_danglers() {
		// Get rid of reverse edges or duplicates. DOESN"T DO ANYTHING
		for (int i = edges.size()-1; i >= 0; i--) {
			Edge &e = edges[i];

			for (int ii = i-1; ii>=0; ii--) {
				Edge &o = edges[ii];
				if ((e.m_n1==o.m_n2 && e.m_n2==o.m_n1) || (e.m_n1==o.m_n1 && e.m_n2==o.m_n2)) {
					edges.erase(edges.begin()+i);
					//cout<<"found double edge";
					break;
				}
			}
		}

		// This chunk breaks it. Removes necessary nodes somehow. 
		// // Get rid of duplicate nodes. DOESN"T DO ANYTHING CURRENTLY
		// for (int i = nodes.size()-1; i >= 0; i--) {
		// 	Node &n = nodes[i];

		// 	for (int ii = i-1; ii>=0; ii--) {
		// 		Node &o = nodes[ii];
		// 		if (round(n.m_x)==round(o.m_x) && round(n.m_y)==round(o.m_y)) {
		// 			nodes.erase(nodes.begin()+i);
		// 			cout<<"found double NODE";
		// 			break;
		// 		}
		// 	}
		// }

		int TRIM_ITERS = 100;
		for (int _ = 0; _ < TRIM_ITERS; _++) {
			// Get rid of dangling nodes
			vector<int> removed_nodes;
			for (int i = nodes.size()-1; i >= 0; i--) {
				if (get_neighboring_nodes(nodes[i]).size()==1) {
					removed_nodes.push_back(nodes[i].node_id);
					nodes.erase(nodes.begin()+i);
				}
			}
			// Get rid of edges w no node
			for (int i = edges.size()-1; i >= 0; i--) {
				Edge &e = edges[i];
				if ((std::find(removed_nodes.begin(), removed_nodes.end(), e.m_n2) != removed_nodes.end()) ||
					(std::find(removed_nodes.begin(), removed_nodes.end(), e.m_n1) != removed_nodes.end())) {
					edges.erase(edges.begin()+i);
				}
			}
		}
	}

	void generate_network()
	{
		nodes.clear(); 
		edges.clear();
		node_id_counter = 0;

		// Initial spine
		//rand_gen.seed(345);

		Node n1 = Node(MARGIN, MARGIN);
		int n1_id = add_node(n1);
		Node n2 = Node(MARGIN+BASE_EDGE_DISTANCE, MARGIN+BASE_EDGE_DISTANCE);
		int n2_id = add_node(n2);
		Edge e = Edge(n1, n2);
		add_edge(e);
		
		float angle = atan2((n2.m_y - n1.m_y), (n2.m_x - n1.m_x));
		int n_spine_iters = 10;
		for (int i; i<n_spine_iters; i++){
			//angle *= rand_gen.randrange(.8f, 1.2f);
			int edge_distance = rand_gen.randint(BASE_EDGE_DISTANCE, BASE_EDGE_DISTANCE*1.5);

			float xx = n2.m_x + (edge_distance * cos(angle));
			float yy = n2.m_y + (edge_distance * sin(angle));

			Node n3 = Node(xx, yy);
			int n3_id = add_node(n3);
			Edge e = Edge(n2, n3);
			add_edge(e);
			
			n2 = n3;
		}
		
		//////////////////////////////////
		// Growing segments off the spine
		for (int i=0; i<NUM_GROW_ITERS; i++){
			Node &n4 = nodes[rand_gen.randn(nodes.size())];

			float angle = rand_gen.randrange(.0f, 6.28f);
			int edge_distance = rand_gen.randint(BASE_EDGE_DISTANCE, BASE_EDGE_DISTANCE*1.5); 
			float xx = n4.m_x + (edge_distance * cos(angle));
			float yy = n4.m_y + (edge_distance * sin(angle));

			if (outside_bounds(xx,yy)) { continue; }
			Node n5 = Node(xx, yy); // the proposed node

			if (angle_too_small(n4,n5)) { continue; }
			if (intersects_existing_edge(n4,n5)) { continue; }

			int n5_id = add_node(n5);
			Edge e = Edge(n4,n5);
			add_edge(e);
			
			int n_gen_iters = 2;
			for (int ii=0; ii<n_gen_iters; ii++){

				angle *= rand_gen.randrange(.8f, 1.2f);
				int edge_distance = rand_gen.randint(BASE_EDGE_DISTANCE, BASE_EDGE_DISTANCE*1.5);
				float xx = n5.m_x + (edge_distance * cos(angle));
				float yy = n5.m_y + (edge_distance * sin(angle));

				Node candidate_node = Node(xx,yy);

				if (outside_bounds(xx,yy)) { continue; }

				Node n6; //  Remember this will be a copy of the node, can't do anything TO it

				// If there is a close node, use it. If not, create new one
				vector<int> close_nodes;
				bool using_existing_node = false;
				for (Node & o : nodes) {
					float dist = get_distance(candidate_node, o);
					if (dist < MIN_MERGE_DISTANCE) {
						close_nodes.push_back(o.node_id);
					}
				}
				if (close_nodes.size() > 0){ 
					// If close node, check to see if makes an acceptable angle
					int ix = close_nodes[rand_gen.randn(close_nodes.size())];
					n6 = nodes[ix]; 
					using_existing_node = true;
					if (angle_too_small(n6, n5) || angle_too_small(n5, n6)) { continue; }
				} else {
					// No close node found, using the newly created one
					n6 = candidate_node;
				}

				// We now have our candidate node but nothing has been added officially to the road network
				// Run the gauntlet of tests to see if new node will be used

				// If has overlap w another edge, skip iter 
				if (intersects_existing_edge(n5, n6)) { continue; }

				if (get_distance(n5,n6) < MIN_ALLOWABLE_DIST) { continue; }

				// Passed the gauntlet! Add the node and edge to the network

				// Register new node if that's the one we're using
				if (!using_existing_node) {
					add_node(n6);
				}
				Edge e = Edge(n5,n6);
				add_edge(e);
				
				n5 = n6;
			}
		}

		trim_danglers();

		// Prep edges for curviness
		for (Node &n : nodes) {
			vector<int> o = get_neighboring_nodes(n);
			if (o.size()==2) {
				Node e2_end_node = get_node(o.at(1));
				Edge &e1 = get_edge(n.node_id, o.at(0));
				Edge &e2 = get_edge(n.node_id, e2_end_node.node_id);
				e1.edge_type = 1;
		
				float angle = atan2((n.m_y - e2_end_node.m_y), (n.m_x - e2_end_node.m_x));
				int b_dist = 20;
				float xx = n.m_x + (b_dist * cos(angle));
				float yy = n.m_y + (b_dist * sin(angle));

				e1.b2 = QPointF(xx, yy); // TODO do we need to check whether to alter b1 or b2? 
			}
		}

		add_opposite_direction_edges_for_all();

		// Create paths
		for (Edge &e : edges) {
			e.m_q_path.moveTo(e.n1_node.q_point);
			e.m_q_path.cubicTo(e.b1, e.b2, e.n2_node.q_point);
		}

		float STOP_SIGN_PROB = 0.;

		for (Edge &e : edges) {
			Node end_node = get_node(e.m_n2);
			int num_outgoing_edges = get_neighboring_nodes(end_node).size();
			if (num_outgoing_edges > 4) { // 4 bc edges are reversed and added again. Actually is two.
				e.ends_in_stopsign = rand_gen.randrange(0.,1.) > (1.-STOP_SIGN_PROB);
			} else {
				e.has_obstacle = rand_gen.randrange(0.,1.) > (1.-OBSTACLE_PROB);
			}
		}

		// Calculate waypoints
		for (Edge &e : edges) {
			e.create_wps();
		}

	}

	void draw (QPainter &painter) 
	{
		//QColor color = QColor(rand_gen.randn(255), rand_gen.randn(255), rand_gen.randn(255)); 
		painter.save();
		painter.setOpacity(.5f);
		for (Edge &e : edges) {
			QColor color = QColor("DarkCyan"); 
			
			painter.setPen(QPen(color, LANE_WIDTH*2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
			painter.drawPath(e.m_q_path);
		}
		painter.restore();

		painter.save();
		for (Edge &e : edges) {
			//painter.setBrush(QColor(200, 10, 10));
			// int s = e.m_waypoints.size();
			// if (e.ends_in_stopsign && s > 10) {

			// 	painter.setPen(QPen(QColor(200, 10, 10), LANE_WIDTH/2, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));

			// 	Waypoint wp_1 = e.m_waypoints.at(s - 8);
			// 	Waypoint wp_2 = e.m_waypoints.at(s - 5);
		
			// 	painter.drawLine(wp_1.painter_x, wp_1.painter_y, wp_2.painter_x, wp_2.painter_y);
			// }

			vector<Waypoint> wps = e.m_waypoints;
			for (Waypoint & wp : wps) {
				if (wp.type==WP_STOP_SIGN){
					painter.setPen(QPen(QColor(200, 10, 10), LANE_WIDTH/2, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
					painter.drawPoint(wp.painter_x, wp.painter_y);
				} 
				// else if (wp.type==WP_AVOIDING_OBSTACLE) {
				// 	painter.setPen(QPen(QColor(0, 10, 100), LANE_WIDTH/2, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
				// 	painter.drawPoint(wp.painter_x, wp.painter_y);
				// }
			}

			if (e.has_obstacle) {
				painter.setPen(QPen(QColor(200, 10, 50), LANE_WIDTH/2, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
				painter.drawPoint(e.obstacle_location.first, e.obstacle_location.second);
			}
		}
		painter.restore();
		// color = QColor(rand_gen.randn(255), rand_gen.randn(255), rand_gen.randn(255)); 
		// painter.setBrush(color);
		// for (Node &n : nodes) {
		// 	painter.drawEllipse(n.m_x, n.m_y, LANE_WIDTH/2, LANE_WIDTH/2);
		// }
		//color = QColor(rand_gen.randn(255), rand_gen.randn(255), rand_gen.randn(255));

		// center
		QColor color = QColor(150,50,150);
		painter.setPen(QPen(color, 1, Qt::DashLine, Qt::FlatCap, Qt::MiterJoin));
		for (Edge &e : edges) {
			painter.drawPath(e.m_q_path);
		}
	}

	vector<Edge> get_outgoing_edges(int node_id) 
	{
		vector<Edge> outgoing_edges;
		for (Edge & e : edges) {
			if (e.m_n1 == node_id) {
				outgoing_edges.push_back(e);
			}
		}
		return outgoing_edges;
	}

	void add_opposite_direction_edges_for_all() 
	{
		for (int i = edges.size()-1; i >= 0; i--) {
			Edge &e = edges.at(i);
			Edge reversed_edge = Edge(get_node(e.m_n2), get_node(e.m_n1));
			reversed_edge.b1 = e.b2;
			reversed_edge.b2 = e.b1;
			add_edge(reversed_edge);
		}
	}

	Node& get_node(int node_id) 
	{
		for (Node &n : nodes) {
			if (n.node_id==node_id) {
				return n;
			}
		}
		// flag if nothing returned
	}

	Edge& get_edge(int node_id_1, int node_id_2) 
	// doesn't matter which order give node ids
	{
		for (Edge &e : edges) {
			if ((e.m_n1==node_id_1 && e.m_n2==node_id_2) || (e.m_n1==node_id_2 && e.m_n2==node_id_1)) {
				return e;
			}
		}
		// flag if nothing returned
	}


	vector<Waypoint> get_more_wps (int& last_node_id, int &current_route_end_node_id)
	{
        vector<Edge> upcoming_candidate_edges = get_outgoing_edges(current_route_end_node_id);

		// flag if none

		if (upcoming_candidate_edges.size()>1) { 
			for (int i = upcoming_candidate_edges.size()-1; i >=0; i--) {
				if (upcoming_candidate_edges.at(i).m_n2==last_node_id){
					upcoming_candidate_edges.erase(upcoming_candidate_edges.begin()+i);
				}
			}
		}

        Edge edge = upcoming_candidate_edges.at(rand_gen.randn(upcoming_candidate_edges.size()));

		last_node_id = current_route_end_node_id;
		current_route_end_node_id = edge.m_n2;


		return edge.m_waypoints;
	}
};