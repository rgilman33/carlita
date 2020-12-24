
#include "../randgen.h"
#include "../basic-abstract-game.h"
#include "../assetgen.h"
#include <cmath>
#include <algorithm>
#include <vector>



using namespace std; 

class Node;
class Edge;

class Node { 
public: 
	int node_id; 
	float m_x;
	float m_y;
	QPointF q_point;

	Node(){}
	Node(float x, float y) : m_x(x), m_y(y)
	{ 
		//this->x = x;
		//this->y = y;
		this->node_id = node_id; 
		this->q_point = QPointF(x,y);
	} 
}; 

class Edge { 
public: 
	int m_n1; 
	int m_n2;
	QPainterPath m_q_path;
	vector<pair<float,float>> m_waypoints;

	Edge(Node &n1, Node &n2) : m_n1 (n1.node_id), m_n2 (n2.node_id) // Don't also set below. THIS does this definition.
	{ 
		//m_n1 = n1;
		//m_n2 = n2;
		m_q_path.moveTo(n1.q_point);
        m_q_path.lineTo(n2.q_point);

		// Make waypoints as x, y coords
		int num_waypoints = ceil(m_q_path.length() / 5.);
		for (float i=.0f; i<1.0f; i+=(1./num_waypoints)) {
			QPointF q_point = m_q_path.pointAtPercent(i);
			pair<float, float> wp;
			// wp.first = q_point.x() * (50./512.);
			// wp.second = q_point.y() * (50./512.); 
			wp.first = q_point.x() * (50./500.);
			wp.second = q_point.y() * (50./500.); 
			m_waypoints.push_back(wp);
		}
	} 
}; 
// TODO get rid of ALL extra edges and nodes

class RoadNetwork { 
public:  
	vector<Node> nodes; 
	vector<Edge> edges;
	int node_id_counter = 0; // This must always be perfectly incremented
	RandGen rand_gen;

	float width = 500.0f;
	float height = 500.0f;

	float MIN_INTERSECTION_ANGLE = 70.0f;
	float BASE_EDGE_DISTANCE = width / 10.0f;
	float MIN_MERGE_DISTANCE = BASE_EDGE_DISTANCE * 1.0f;
	float MIN_ALLOWABLE_DIST = BASE_EDGE_DISTANCE * .3f;

	float MARGIN = width / 10.f;
	
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

		return (int) abs(floor(alpha * 180. / 3.14159 + 0.5));
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

			if (isIntersecting(n1, n2, nodes[e.m_n1], nodes[e.m_n2])) { 
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
			Node o = nodes[node_id];
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
					cout<<"found double edge";
					break;
				}
			}
		}

		// Get rid of duplicate nodes. DOESN"T DO ANYTHING CURRENTLY
		for (int i = nodes.size()-1; i >= 0; i--) {
			Node &n = nodes[i];

			for (int ii = i-1; ii>=0; ii--) {
				Node &o = nodes[ii];
				if (round(n.m_x)==round(o.m_x) && round(n.m_y)==round(o.m_y)) {
					nodes.erase(nodes.begin()+i);
					cout<<"found double NODE";
					break;
				}
			}
		}

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
		for (int i=0; i<1000; i++){
			Node &n4 = nodes[rand_gen.randn(nodes.size())];

			float angle = rand_gen.randrange(.0f, 6.28f);
			int edge_distance = rand_gen.randint(BASE_EDGE_DISTANCE, BASE_EDGE_DISTANCE*1.5); 
			float xx = n4.m_x + (edge_distance * cos(angle));
			float yy = n4.m_y + (edge_distance * sin(angle));

			if (outside_bounds(xx,yy)) { continue; }
			Node n5 = Node(xx, yy);

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
					if (angle_too_small(n6, n5)) { continue; }
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
		add_opposite_direction_edges_for_all(); // Temporary hack?

	}

	void draw (QPainter &painter) 
	{

        painter.setPen(QPen(QColor(79, 106, 25), 15, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
        // painter.setBrush(QColor(122, 163, 39));

		for (Edge &e : edges) {
			painter.drawPath(e.m_q_path);
		}
		painter.setPen(QPen(QColor(122, 163, 39), 5, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
		for (Node &n : nodes) {
			painter.drawPoint(n.q_point);
		}
	}

	vector<Edge> get_outgoing_edges(int node_id) 
	{
		vector<Edge> outgoing_edges;
		for (Edge e : edges) {
			if (e.m_n1 == node_id) {
				outgoing_edges.push_back(e);
			}
		}
		return outgoing_edges;
	}

	void add_opposite_direction_edges_for_all() 
	{
		for (int i = edges.size()-1; i >= 0; i--) {
			Edge &e = edges[i];
			Edge reversed_edge = Edge(get_node(e.m_n2), get_node(e.m_n1));
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
	}


	vector<pair<float,float>> get_more_wps (int& last_node_id, int &current_route_end_node_id)
	{
        vector<Edge> upcoming_candidate_edges = get_outgoing_edges(current_route_end_node_id);

		for (int i = upcoming_candidate_edges.size(); i >=0; i--) {
			if (upcoming_candidate_edges[i].m_n2==last_node_id){
				upcoming_candidate_edges.erase(upcoming_candidate_edges.begin()+i);
			}
		}

        Edge edge = upcoming_candidate_edges[rand_gen.randn(upcoming_candidate_edges.size())];
		last_node_id = current_route_end_node_id;
		current_route_end_node_id = edge.m_n2;


		return edge.m_waypoints;
	}
};