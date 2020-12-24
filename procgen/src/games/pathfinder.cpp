// C++ implementation to find the 
// shortest path in a directed 
// graph from source vertex to 
// the destination vertex 

#include <bits/stdc++.h> 
#define infi 1000000000 
using namespace std; 

// Class of the node 
class Node { 
public: 
	int node_id; 

	// Adjacency list that shows the 
	// vertexNumber of child vertex 
	// and the weight of the edge 
	vector<pair<int, int> > children; 

	Node(int node_id) 
	{ 
		this->node_id = node_id; 
	} 

	// Function to add the child for 
	// the given node 
	void add_child(int node_id, int road_id) 
	{ 
		pair<int, int> p; 
		p.first = node_id; 
		p.second = road_id; 
		children.push_back(p); 
	} 
}; 

// Function to find the distance of 
// the node from the given source 
// vertex to the destination vertex 
vector<int> dijkstraDist( 
	vector<Node*> g, 
	int s, vector<int>& path) 
{ 
	// Stores distance of each 
	// vertex from source vertex 
	vector<int> dist(g.size()); 

	// Boolean array that shows 
	// whether the vertex 'i' 
	// is visited or not 
	bool visited[g.size()]; 
	for (int i = 0; i < g.size(); i++) { 
		visited[i] = false; 
		path[i] = -1; 
		dist[i] = infi; 
	} 
	dist[s] = 0; 
	path[s] = -1; 
	int current = s; 

	// Set of vertices that has 
	// a parent (one or more) 
	// marked as visited 
	unordered_set<int> sett; 
	while (true) { 

		// Mark current as visited 
		visited[current] = true; 
		for (int i = 0; i < g[current]->children.size(); i++) { 

			int v = g[current]->children[i].first; 
			if (visited[v]) 
				continue; 

			// Inserting into the visited vertex 
			sett.insert(v); 
			int alt = dist[current] + g[current]->children[i].second; 

			// Condition to check the distance 
			// is correct and update it 
			// if it is minimum from the previous 
			// computed distance 
			if (alt < dist[v]) { 
				dist[v] = alt; 
				path[v] = current; 
			} 
		} 
		sett.erase(current); 
		if (sett.empty()) 
			break; 

		// The new current 
		int minDist = infi; 
		int index = 0; 

		// Loop to update the distance of the vertices of the graph 
		for (int a: sett) { 
			if (dist[a] < minDist) { 
				minDist = dist[a]; 
				index = a; 
			} 
		} 
		current = index; 
	} 
	return dist; 
} 

// Function to print the path 
// from the source vertex to 
// the destination vertex 
void printPath(vector<int> path, int i, int s) 
{ 
	if (i != s) { 
		printPath(path, path[i], s); 
		cout << path[i] << " "; 
	} 
}
