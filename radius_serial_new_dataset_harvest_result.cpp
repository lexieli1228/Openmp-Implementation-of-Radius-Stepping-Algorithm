// serial version of radius stepping algorithm
// the implementation of algorithm 1 in the original paper

#include <bits/stdc++.h>
#include <fstream>
using namespace std;

// we implement the version where k = 1, rho = 3
#define rho 3

// maze size of the benchmark
// original: 10000
// self-dataset: 1000
#define maze_size 1000

#define edge_add_parameter 6

// the position of the starting point and the ending point
int start_x, start_y;
int end_x, end_y;


// four directions of one-step movement
int dx[4] = {-1, 0, 0, 1};
int dy[4] = {0, -1, 1, 0};


// the maze
int maze[maze_size][maze_size];

// radius value for every vertex
int rv[maze_size][maze_size];


int cnt_edge[maze_size*maze_size];


// dijkstra heap
struct edge_node
{
	public:
		int ending_index;
		int weight;
		
		edge_node(int ending_index_x, int weight_x)
		{
			ending_index = ending_index_x;
			weight = weight_x;
	    }
        
        bool operator < (const edge_node &x)const
        {
        	if (weight == x.weight)
        	{
        		if (ending_index == x.ending_index)
        		{
        			return true;
				}
        		return ending_index < x.ending_index;
			}
			return weight < x.weight;
		}	
};


edge_node* e;


// distance used in dijkstra search
int distance_x[maze_size*maze_size];

int* exclude_S;
// gamma_distance: note the distance from a node to the starting point
int* gamma_distance;
	

// memorize the road
// prev_node: the mediate node number of the edge from A to B
int* prev_node;

int* prev_node_from_source;


// output file
ofstream fout;


/*initialize_space_x
  initialize the e and prev_node used
*/
void initialize_space_x()
{
	long long e_size = maze_size*maze_size*edge_add_parameter;
	e = (edge_node*)malloc((e_size)*sizeof(edge_node));
	exclude_S = (int*)malloc(maze_size*maze_size*sizeof(int));
    gamma_distance = (int*)malloc(maze_size*maze_size*sizeof(int));

    prev_node = (int*)malloc(e_size*sizeof(int));
    prev_node_from_source = (int*)malloc(maze_size*maze_size*sizeof(int));
    if (prev_node != NULL)
    {
    	cout << "prev_node allocation successful!" << endl;
	}
	cout << "initialize space successful!" << endl;
}


/*initialize_variables
  initialize arrays and stuffs for further use
*/
void initialize_variables()
{
	// maze rv, cnt_edge
	for (int i = 0; i < maze_size; ++ i)
	{
		for (int j = 0; j < maze_size; ++ j)
		{
			maze[i][j] = 0;
			rv[i][j] = 0;
			cnt_edge[i*maze_size + j] = 0;
			gamma_distance[i*maze_size + j] = 1<<30;
			prev_node_from_source[i*maze_size + j] = i*maze_size + j;
		}
	}
    
	long long e_size = maze_size*maze_size*edge_add_parameter;
	
	for (long long i = 0; i < e_size; ++ i)
	{
		e[i] = edge_node(0, 0);
		prev_node[i] = -1;
	}
}


/*read_data
  read data into maze array*/
void read_data(int case_i)
{
    string case_i_name = "../benchmark_self/case";
    if (case_i >= 10)
    {
    	case_i_name += (char(int('0') + case_i/10));
	}
    case_i_name += (char(int('0') + case_i%10));
    case_i_name += ".txt";
    //string case_i_name = "test_input.txt";
    ifstream case_i_file;
    //string case_i_name = "./here.txt";
    case_i_file.open(case_i_name.c_str());
    if (!case_i_file.is_open())
    {
    	cout << "Open file " << case_i_name << " error!" << endl;
    	return;
	}
	else
	{
		cout << "Open file " << case_i_name << " successful!" << endl;
	}
	string temp;
	for (int i = 0; i < maze_size; ++ i)
	{
		for (int j = 0; j < maze_size; ++ j)
		{
			case_i_file >> temp;
			if (temp == "S")
			{
				start_x = i;
				start_y = j;
			}
			else if (temp == "D")
			{
				end_x = i;
				end_y = j;
			}
			else
			{
				maze[i][j] = atoi(temp.c_str());
			}
		}
	}
	case_i_file.close();
    return;
}


/*dijkstra_find_r
  dijkstra funcition used for finding rv for every node in the graph
  aochang2648 from csdn
  our implementation: run dijkstra from each vertex for rho rounds*/
int dijkstra_find_r(int node_index)
{
	for (int i = 0; i < maze_size; ++ i)
	{
		for (int j = 0; j < maze_size; ++ j)
		{
			distance_x[i*maze_size + j] = 1<<30;
		}
	}
	
	priority_queue<edge_node, vector<edge_node> > q;
	distance_x[node_index] = 0;
	
	edge_node temp_node = edge_node(node_index, 0);
	q.push(temp_node);

	int round_cnt = 0;
	
	while (!q.empty() && round_cnt < rho)
	{
		int dest_index = q.top().ending_index;
		int dest_weight = q.top().weight;
		q.pop();

		++ round_cnt;
		
		// search for the node that is have edge connections with dest_index
		int dest_x = dest_index / maze_size;
		int dest_y = dest_index % maze_size;
		
		if (dest_weight != distance_x[dest_index])
		{
			continue;
		}
		
		for (int k = 0; k < min(rho, cnt_edge[dest_index]); ++ k)
		{
			int new_dest_x = e[dest_index*edge_add_parameter + k].ending_index/maze_size;
			int new_dest_y = e[dest_index*edge_add_parameter + k].ending_index%maze_size;
			int cost_new_dest;
			
			if (maze[new_dest_x][new_dest_y] == -1)
			{
				continue;
			}
			else
			{
				cost_new_dest = e[dest_index*edge_add_parameter + k].weight;
			}
			
			if ((distance_x[dest_index] + cost_new_dest) < distance_x[new_dest_x*maze_size + new_dest_y])
			{
				int previous_distance = distance_x[new_dest_x*maze_size + new_dest_y];
				distance_x[new_dest_x*maze_size + new_dest_y] = distance_x[dest_index] + cost_new_dest;
				q.push(edge_node(new_dest_x*maze_size + new_dest_y, distance_x[new_dest_x*maze_size + new_dest_y]));
				
				// add a new edge: checking if there is a short cut edge
				// add a new edge
				
				if (previous_distance == 1<<30)
				{
					if (cnt_edge[node_index] < edge_add_parameter - 1)
					{
						e[node_index*edge_add_parameter + cnt_edge[node_index]] = edge_node(new_dest_x*maze_size + new_dest_y, distance_x[new_dest_x*maze_size + new_dest_y]);
//				    	cout << "add edge: (" << node_index/maze_size << "," << node_index%maze_size << ") to (" << new_dest_x << "," << new_dest_y;
//						cout << ") via: (" << dest_x << "," << dest_y << ") with weight " << distance_x[new_dest_x*maze_size + new_dest_y] << endl;
//						cout << "after adding cnt: " << cnt_edge[node_index] << endl;
				        prev_node[node_index*edge_add_parameter + cnt_edge[node_index]] = dest_index;
						++ cnt_edge[node_index];
						if (node_index == start_x*maze_size + start_y)
						{
							prev_node_from_source[new_dest_x*maze_size + new_dest_y] = dest_index;
						}		 
					} 
				}
				// just change weight
				else
				{
					for (int m = 0; m < cnt_edge[node_index]; ++ m)
					{
						if (e[node_index*edge_add_parameter + m].ending_index == new_dest_x*maze_size + new_dest_y)
						{
							e[node_index*edge_add_parameter + m].weight = distance_x[new_dest_x*maze_size + new_dest_y];
							prev_node[node_index*edge_add_parameter + m] = dest_index;
						}
					}
					if (node_index == start_x*maze_size + start_y)
					{
						prev_node_from_source[new_dest_x*maze_size + new_dest_y] = dest_index;
					}
				}
		    }
	    }
	}
	
//	cout << "here are the edges for node (" << node_index / maze_size << "," << node_index % maze_size << ")" << endl;
//	for (int i = 0; i < cnt_edge[node_index]; ++ i)
//	{
//		cout << "from (" <<  node_index / maze_size << "," << node_index % maze_size << ")";
//		cout << " to (" << e[node_index*edge_add_parameter + i].ending_index/maze_size << "," << e[node_index*edge_add_parameter + i].ending_index%maze_size << ")";
//		cout << " with weight: " << e[node_index*edge_add_parameter + i].weight << endl;
//	} 
	
//	cout << "round " << node_index << " dijkstra end" << endl;
	
    // after running for rho rounds
    // harvesting r(v) from the distance array
    // we set r(v) = r_{rho}(v), which is the rho^{th} nearest neighbor
    // since the distance from the starting node to itself is zero then it should be 0
    // we find the (rho + 1)^{th} smallest number in the distance array
    
    vector<int> new_distance;
    for (int i = 0; i < cnt_edge[node_index]; ++ i)
    {
    	new_distance.push_back(e[node_index*edge_add_parameter + i].weight);
	}
    
    // cout << "returning (" << node_index/maze_size << "," << node_index%maze_size << ")" << endl;
    if (new_distance.size() < rho)
    {
    	if (new_distance.size() != 0)
    	{
    		sort(new_distance.begin(), new_distance.end());
    		return new_distance[new_distance.size() - 1];
		}
		else
		{
			return 0;
		}
	}
    
    if (rho > 0)
    {
    	sort(new_distance.begin(), new_distance.end());
    	return new_distance[rho - 1];
	}
	else
	{
		return new_distance[rho];
	}
}


/*calculate_rv
  calculate r(v) for every v before radius stepping
  acccording to lemma 2.1, we set r(v) = r_{rho}(V)
  r_{rho}(v) is the distance from v to the rho-th closest vertex to v
  calculate r{rho}(v) using dijkstra algorithm for weighted graph*/
void calculate_rv()
{
	// initialize direct edges given by maze
//	cout << "calculating rv" << endl;
	for (int i = 0; i < maze_size; ++ i)
	{
		for (int j = 0; j < maze_size; ++ j)
		{
		//	cout << "here we are dealing with " << i << "," << j << endl;
			for (int k = 0; k < 4; ++ k)
			{
				int new_x = i + dx[k];
				int new_y = j + dy[k];
				if (new_x >= 0 && new_x < maze_size && new_y >= 0 && new_y < maze_size && maze[new_x][new_y] != -1)
				{
					e[(i*maze_size + j)*edge_add_parameter + cnt_edge[i*maze_size + j]] = edge_node(new_x*maze_size + new_y, maze[new_x][new_y]);
					prev_node[(i*maze_size + j)*edge_add_parameter + cnt_edge[i*maze_size + j]] = i*maze_size + j;
					++ cnt_edge[i*maze_size + j];
					if (i == start_x && j == start_y)
					{
						prev_node_from_source[new_x*maze_size + new_y] = i*maze_size + j;
					}
				}
			}
		}
	}
	
	// run parallel dijkstra from each vertex for rho rounds
	// in one cell, we can only move to four directions for one step
	// we simply do not calculate the cost of starting points of paths from every vertex
	
//	cout << "after initializing existing edges" << endl;
	// run dijkstra for every point in the maze
	for (int i = 0; i < maze_size; ++ i)
	{
		cout << "processing line " << i << "..." << endl;
		for (int j = 0; j < maze_size; ++ j)
		{
			// if maze[i][j] == -1, then it is a block and there is no need to calculate dijkstra
			if (maze[i][j] == -1)
			{
				continue;
			}
			// we implement this using heap
			// source node: i, j
		    rv[i][j] = dijkstra_find_r(i*maze_size + j);
		}
	}
//	cout << endl;
//	cout << "after running dijkstra" << endl;
	return;
}


/*radius_stepping
  radius stepping algorithm, we implement algorithm 1 introduced in the paper
  at the same time, we memorize the road using  array*/
int radius_stepping()
{
//	cout << "start radius stepping" << endl;
	int start_index = start_x*maze_size + start_y;
	int end_index = end_x*maze_size + end_y;
	
	// initialize set exclude_S to store the nodes that has not been iterated
	int exclude_S_size = 0;
	
	for (int i = 0; i < maze_size; ++ i)
	{
		for (int j = 0; j < maze_size; ++ j)
		{
			if (i*maze_size + j != start_index)
			{
				exclude_S[i*maze_size + j] = 1;
				++ exclude_S_size;
			}
			else
			{
				exclude_S_size = 0;
			}
		}          
	}
	
	gamma_distance[start_index] = 0;
	
	// assign the gamma distance if the there are edges between the starting point and the current node
	// we value every node v in the set N(s) and let gamma_distance[v] <- weight(s, v)
	for (int i = 0; i < cnt_edge[start_index]; ++ i)
	{
		gamma_distance[e[start_index*edge_add_parameter + i].ending_index] = e[start_index*edge_add_parameter + i].weight;
	}
	
	// cout << "before bellman ford" << endl;
	while (exclude_S_size > 0)
	{
		// find the value of di
		int di = 1<<30;
		for (int i = 0; i < maze_size*maze_size; ++ i)
		{
			if (exclude_S[i] == 1)
			{
				if (gamma_distance[i] != 1<<30)
				{
					if (gamma_distance[i] + rv[i / maze_size][i % maze_size] < di)
					{
						di = gamma_distance[i] + rv[i / maze_size][i % maze_size];
					}
				}
			}
		}
		
		// Bellman-Ford operation
		// execute until no gamma_distance[v] <= di was updated
		int execute_flag = 1;
		while (execute_flag)
		{
			// cout << "inside execute flag loop" << endl;
			execute_flag = 0;
			for (int u = 0; u < maze_size*maze_size; ++ u)
			{
				if (exclude_S[u] == 1)
				{
					if (maze[u/maze_size][u%maze_size] != -1 && gamma_distance[u] <= di)
					{
						for (int i = 1; i < cnt_edge[u]; ++ i)
						{
							int v = e[u*edge_add_parameter + i].ending_index;
							int weight_u_v = e[u*edge_add_parameter + i].weight;
							if (maze[v/maze_size][v%maze_size] != -1 && gamma_distance[v] > gamma_distance[u] + weight_u_v)
							{
								gamma_distance[v] = gamma_distance[u] + weight_u_v;
								// memorize the middle node
							    prev_node_from_source[v] = u;
								// execute until no gamma_distance[v] <= di was updated
								if (gamma_distance[v] <= di)
								{
									execute_flag = 1;
								}
							}
						}
					}
				}
			}
		}
		
		// renew S and exclude_S
		// if a node in S' gamma distance is less than or equal to di, then keep it in S, which means we should erase it from the exclude set
		// if a node in S' gamma distance is greater than di, then erase it from S, which means we should insert it to the exclude set
		exclude_S_size = 0;
		for (int i = 0; i < maze_size*maze_size; ++ i)
		{
			if (gamma_distance[i] > di)
			{
				exclude_S[i] = 1;
				++ exclude_S_size;
			}
			else
			{
				exclude_S[i] = 0;
			}
		}
	}
	
	/*
	cout << "gamma distance: " << endl;
	for (int i = 0; i < maze_size; ++ i)
	{
		for (int j = 0; j < maze_size; ++ j)
		{
			cout << gamma_distance[i*maze_size + j] << " ";
		}
		cout << endl;
	}*/
	
	// cout << "after bellman ford" << endl;
	
    // check the result
	if (gamma_distance[end_index] == 1<<30)
	{
		return -1;
	}
	return gamma_distance[end_index];
}


/* output_result
   recursively output path */
void output_result(int curr_node_start, int curr_node_end)
{
    if (curr_node_start == curr_node_end)
	{
		fout << "(" << curr_node_start/maze_size << "," << curr_node_start%maze_size << ") ";
		return;
	}
	if (curr_node_start == start_x*maze_size + start_y)
	{
		if (prev_node_from_source[curr_node_end] == curr_node_start)
		{
			fout << "(" << prev_node_from_source[curr_node_end]/maze_size << "," << prev_node_from_source[curr_node_end]%maze_size << ") ";
     	    return;
		}
		else
		{
			output_result(curr_node_start, prev_node_from_source[curr_node_end]);
		}
		output_result(prev_node_from_source[curr_node_end], curr_node_end);
	}
	else
	{
		for (int i = 0; i <= cnt_edge[curr_node_start]; ++ i)
		{
			if (e[curr_node_start*edge_add_parameter + i].ending_index == curr_node_end && prev_node[curr_node_start*edge_add_parameter + i] != -1)
			{
				if (prev_node[curr_node_start*edge_add_parameter + i] == curr_node_start)
				{
					fout << "(" << prev_node[curr_node_start*edge_add_parameter + i]/maze_size << "," << prev_node[curr_node_start*edge_add_parameter + i]%maze_size << ") ";
     	            return;
				}
				else
				{
					output_result(curr_node_start, prev_node[curr_node_start*edge_add_parameter + i]);
				}
				output_result(prev_node[curr_node_start*edge_add_parameter + i], curr_node_end);
                break;
			}
		}
	}
}


/*check_edges
  check radius stepping results*/
void check_edges()
{
	cout << "source node: (" << start_x << "," << end_x << ")" << endl;
	cout << "overall prev:" << endl;
	for (int i = 0; i < maze_size*maze_size; ++ i)
	{
	    cout << "from source node to (" << i/maze_size << "," << i%maze_size << "): (" << prev_node_from_source[i]/maze_size << "," << prev_node_from_source[i]%maze_size << ")" << endl;
	}
	
	cout << "prev and edges:" << endl;
	for (int i = 0; i < maze_size*maze_size; ++ i)
	{
		cout << "checking node (" << i/maze_size << "," << i%maze_size << ")" << endl;
		cout << "edge number: " << cnt_edge[i] << endl;
		for (int j = 0; j < cnt_edge[i]; ++ j)
		{
			cout << "edge info: from (" << i/maze_size << "," << i%maze_size << ") to (" << e[i*edge_add_parameter + j].ending_index/maze_size << "," <<  e[i*edge_add_parameter + j].ending_index%maze_size << ") ";
			cout << "with weight " <<  e[i*edge_add_parameter + j].weight;
			cout << " with mediate node (" << prev_node[i*edge_add_parameter + j]/maze_size << "," << prev_node[i*edge_add_parameter + j]%maze_size << ")" << endl; 
		}
	}
}

/* main function */
int main()
{
	// initialize the space used by computing
	initialize_space_x();
	
	// output file
	fout.open("../results/radius_serial_new_dataset_harvest_output.txt");
	if (!fout.is_open())
    {
    	cout << "Open output file error!" << endl;
    	return 0;
	}
	else
	{
		cout << "Open output file successful!" << endl;
	}
    // overall there are 50 cases
    for (int case_i = 0; case_i < 50; ++ case_i)
    {
    	
        // initialization of arrays and stuffs
        initialize_variables();
        
    	// read data into maze
        read_data(case_i);
        
        cout << "reading data for " << case_i << " successful!" << endl;
        
        // calculate matrix r(v) before radius stepping
        calculate_rv();
        
        
        
        cout << "calculate_rv successful" << endl;
       
        /*
        cout << "here is the rv after overall calculate_rv:" << endl;
		for (int i = 0; i < maze_size; ++ i)
		{
			for (int j = 0; j < maze_size; ++ j)
			{
				cout << rv[i][j] << " ";
			}
			cout << endl;
		}*/
		
        // harvest radius stepping output
        int result = radius_stepping();
        cout << "radius stepping successful" << endl;
        
        // output result
        string output_result_string = "case";
        if (case_i >= 10)
        {
        	output_result_string += (char(int('0') + case_i/10));
		}
        output_result_string += (char(int('0') + case_i%10));
        output_result_string += ": ";
        
    //    check_edges();
		
		// checking the result
        if (result == -1)
        {
        	fout << output_result_string << result << " " << endl;
		}
		else
		{
		    fout << output_result_string << result << " ";
	
     		int start_index = start_x*maze_size + start_y;
			int end_index = end_x*maze_size + end_y;
			
			/*
			for (int i = 0; i < maze_size*maze_size; ++ i)
			{
				cout << "from source to node (" << i/maze_size << "," << i%maze_size << "): mediate point (" << prev_node_from_source[i]/maze_size << "," << prev_node_from_source[i]%maze_size << ")" << endl;
			}
			
			cout << "checking prev: " << endl;
			for (int i = 0; i < maze_size*maze_size; ++ i)
			{
				for (int j = 0; j < cnt_edge[i]; ++ j)
				{
					cout << "mediate point from (" << i/maze_size << "," << i%maze_size << ") to (";
					cout << e[i*edge_add_parameter + j].ending_index/maze_size << "," << e[i*edge_add_parameter + j].ending_index%maze_size << ") is : (";
					cout << prev_node[i*edge_add_parameter + j]/maze_size << "," << prev_node[i*edge_add_parameter + j]%maze_size << ")" << endl;
				}
			}
			cout << "here" << endl;*/
	     	
			output_result(start_index, end_index);
		    fout << "(" << end_x << "," << end_y << ")" << endl;
		}
    }
    fout.close();
    return 0;
}