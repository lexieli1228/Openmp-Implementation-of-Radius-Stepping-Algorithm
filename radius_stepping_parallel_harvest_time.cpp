// parallel version of radius stepping algorithm
// the implementation of algorithm 1 in the original paper

#include <bits/stdc++.h>
#include <fstream>
#include <omp.h>
using namespace std;

#define maximum_thread_num 4

// we implement the version where k = 1, rho = 3
#define rho 3

// maze size of the benchmark
// original: 10000
// self-dataset: 1000
#define maze_size 1000

#define edge_add_parameter 6

string output_file_name = "../results/20220626_1009_different_parameters/radius_parallel_harvesting_time_4_3.txt";

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

int cnt_edge[maze_size * maze_size];

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

	bool operator<(const edge_node &x) const
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

edge_node *e;

int *exclude_S;
// gamma_distance: note the distance from a node to the starting point
int *gamma_distance;

// memorize the road
// prev_node: the mediate node number of the edge from A to B
int *prev_node;

int *prev_node_from_source;

// output file
ofstream fout;

/*variables for recording time
*/
double start_time;
double end_time;

/*initialize_space_x
  initialize the e and prev_node used
*/
void initialize_space_x()
{
	long long e_size = maze_size * maze_size * edge_add_parameter;
	e = (edge_node *)malloc((e_size) * sizeof(edge_node));
	exclude_S = (int *)malloc(maze_size * maze_size * sizeof(int));
	gamma_distance = (int *)malloc(maze_size * maze_size * sizeof(int));

	prev_node = (int *)malloc(e_size * sizeof(int));
	prev_node_from_source = (int *)malloc(maze_size * maze_size * sizeof(int));
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
	for (int i = 0; i < maze_size; ++i)
	{
		for (int j = 0; j < maze_size; ++j)
		{
			maze[i][j] = 0;
			rv[i][j] = 0;
			cnt_edge[i * maze_size + j] = 0;
			gamma_distance[i * maze_size + j] = 1 << 30;
			prev_node_from_source[i * maze_size + j] = i * maze_size + j;
		}
	}

	long long e_size = maze_size * maze_size * edge_add_parameter;

	for (long long i = 0; i < e_size; ++i)
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
	// string case_i_name = "./here.txt";
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
	for (int i = 0; i < maze_size; ++i)
	{
		for (int j = 0; j < maze_size; ++j)
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


/* output_result
   recursively output path */
void output_result(int curr_node_start, int curr_node_end)
{
	if (curr_node_start == curr_node_end)
	{
		fout << "(" << curr_node_start / maze_size << "," << curr_node_start % maze_size << ") ";
		return;
	}
	if (curr_node_start == start_x * maze_size + start_y)
	{
		if (prev_node_from_source[curr_node_end] == curr_node_start)
		{
			fout << "(" << prev_node_from_source[curr_node_end] / maze_size << "," << prev_node_from_source[curr_node_end] % maze_size << ") ";
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
		for (int i = 0; i <= cnt_edge[curr_node_start]; ++i)
		{
			if (e[curr_node_start * edge_add_parameter + i].ending_index == curr_node_end && prev_node[curr_node_start * edge_add_parameter + i] != -1)
			{
				if (prev_node[curr_node_start * edge_add_parameter + i] == curr_node_start)
				{
					fout << "(" << prev_node[curr_node_start * edge_add_parameter + i] / maze_size << "," << prev_node[curr_node_start * edge_add_parameter + i] % maze_size << ") ";
					return;
				}
				else
				{
					output_result(curr_node_start, prev_node[curr_node_start * edge_add_parameter + i]);
				}
				output_result(prev_node[curr_node_start * edge_add_parameter + i], curr_node_end);
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
	for (int i = 0; i < maze_size * maze_size; ++i)
	{
		cout << "from source node to (" << i / maze_size << "," << i % maze_size << "): (" << prev_node_from_source[i] / maze_size << "," << prev_node_from_source[i] % maze_size << ")" << endl;
	}

	cout << "prev and edges:" << endl;
	for (int i = 0; i < maze_size * maze_size; ++i)
	{
		cout << "checking node (" << i / maze_size << "," << i % maze_size << ")" << endl;
		cout << "edge number: " << cnt_edge[i] << endl;
		for (int j = 0; j < cnt_edge[i]; ++j)
		{
			cout << "edge info: from (" << i / maze_size << "," << i % maze_size << ") to (" << e[i * edge_add_parameter + j].ending_index / maze_size << "," << e[i * edge_add_parameter + j].ending_index % maze_size << ") ";
			cout << "with weight " << e[i * edge_add_parameter + j].weight;
			cout << " with mediate node (" << prev_node[i * edge_add_parameter + j] / maze_size << "," << prev_node[i * edge_add_parameter + j] % maze_size << ")" << endl;
		}
	}
}


/* main function */
int main()
{
	// initialize the space used by computing
	initialize_space_x();

	// output file
	fout.open(output_file_name);
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
	for (int case_i = 0; case_i < 50; ++case_i)
	{

		// initialization of arrays and stuffs
		initialize_variables();

		// read data into maze
		read_data(case_i);

		int result;

		cout << "reading data for " << case_i << " successful!" << endl;

		// calculate matrix r(v) before radius stepping

		int id;
		int w = maze_size / maximum_thread_num;
		omp_set_num_threads(maximum_thread_num);
		
		start_time = omp_get_wtime();

#pragma omp parallel shared(e, maze, cnt_edge, prev_node, prev_node_from_source, rv, w) private(id)
		{
			id = omp_get_thread_num();

			for (int i = w * id; i < w * (id + 1); ++i)
			{
				for (int j = 0; j < maze_size; ++j)
				{
					//	cout << "here we are dealing with " << i << "," << j << endl;
					for (int k = 0; k < 4; ++k)
					{
						int new_x = i + dx[k];
						int new_y = j + dy[k];
						if (new_x >= 0 && new_x < maze_size && new_y >= 0 && new_y < maze_size && maze[new_x][new_y] != -1)
						{
							e[(i * maze_size + j) * edge_add_parameter + cnt_edge[i * maze_size + j]] = edge_node(new_x * maze_size + new_y, maze[new_x][new_y]);
							prev_node[(i * maze_size + j) * edge_add_parameter + cnt_edge[i * maze_size + j]] = i * maze_size + j;
							++cnt_edge[i * maze_size + j];
							if (i == start_x && j == start_y)
							{
								prev_node_from_source[new_x * maze_size + new_y] = i * maze_size + j;
							}
						}
					}
				}
			}

#pragma omp barrier

			// run parallel dijkstra from each vertex for rho rounds
			// in one cell, we can only move to four directions for one step
			// we simply do not calculate the cost of starting points of paths from every vertex

			//	cout << "after initializing existing edges" << endl;
			// run dijkstra for every point in the maze
			for (int ix = w * id; ix < w * (id + 1); ++ix)
			{
				for (int jx = 0; jx < maze_size; ++jx)
				{
					// if maze[i][j] == -1, then it is a block and there is no need to calculate dijkstra
					if (maze[ix][jx] == -1)
					{
						continue;
					}
					// we implement this using heap
					// source node: i, j
					int node_index = ix * maze_size + jx;

					vector<edge_node> distance_x;

					priority_queue<edge_node, vector<edge_node>> q;

					edge_node temp_node = edge_node(node_index, 0);
					q.push(temp_node);
					distance_x.push_back(temp_node);

					int round_cnt = 0;

					while (!q.empty() && round_cnt < rho)
					{
						int dest_index = q.top().ending_index;
						int dest_weight = q.top().weight;
						q.pop();

						++round_cnt;

						// search for the node that is have edge connections with dest_index
						int dest_x = dest_index / maze_size;
						int dest_y = dest_index % maze_size;

						int marker_been = 0;

						for (int k = 0; k < distance_x.size(); ++k)
						{
							if (distance_x[k].ending_index == dest_index)
							{
								if (distance_x[k].weight != dest_weight)
								{
									marker_been = 1;
								}
							}
						}

						if (marker_been == 1)
						{
							continue;
						}

						for (int k = 0; k < min(rho, cnt_edge[dest_index]); ++k)
						{
							int new_dest_x = e[dest_index * edge_add_parameter + k].ending_index / maze_size;
							int new_dest_y = e[dest_index * edge_add_parameter + k].ending_index % maze_size;
							int cost_new_dest;

							int distance_dest_index = 1 << 30;
							int distance_new_dest = 1 << 30;
							int distance_renew_index = -1;

							for (int tx = 0; tx < distance_x.size(); ++tx)
							{
								if (distance_x[tx].ending_index == dest_index)
								{
									distance_dest_index = distance_x[tx].weight;
								}
								else if (distance_x[tx].ending_index == new_dest_x * maze_size + new_dest_y)
								{
									distance_new_dest = distance_x[tx].weight;
									distance_renew_index = tx;
								}
							}

							if (maze[new_dest_x][new_dest_y] == -1)
							{
								continue;
							}
							else
							{
								cost_new_dest = e[dest_index * edge_add_parameter + k].weight;
							}

							if ((distance_dest_index + cost_new_dest) < distance_new_dest)
							{
								int previous_distance = distance_new_dest;
								distance_new_dest = distance_dest_index + cost_new_dest;
								q.push(edge_node(new_dest_x * maze_size + new_dest_y, distance_new_dest));

								// add a new edge: checking if there is a short cut edge
								// add a new edge

								if (previous_distance == 1 << 30)
								{
									if (cnt_edge[node_index] < edge_add_parameter - 1)
									{
	#pragma omp critical
	{
										e[node_index * edge_add_parameter + cnt_edge[node_index]] = edge_node(new_dest_x * maze_size + new_dest_y, distance_new_dest);
										//				    	cout << "add edge: (" << node_index/maze_size << "," << node_index%maze_size << ") to (" << new_dest_x << "," << new_dest_y;
										//						cout << ") via: (" << dest_x << "," << dest_y << ") with weight " << distance_x[new_dest_x*maze_size + new_dest_y] << endl;
										//						cout << "after adding cnt: " << cnt_edge[node_index] << endl;
										prev_node[node_index * edge_add_parameter + cnt_edge[node_index]] = dest_index;
										++cnt_edge[node_index];
										if (node_index == start_x * maze_size + start_y)
										{
											prev_node_from_source[new_dest_x * maze_size + new_dest_y] = dest_index;
										}
	}
									}
									distance_x.push_back(edge_node(new_dest_x * maze_size + new_dest_y, distance_new_dest));
								}
								// just change weight
								else
								{
									for (int m = 0; m < cnt_edge[node_index]; ++m)
									{
										if (e[node_index * edge_add_parameter + m].ending_index == new_dest_x * maze_size + new_dest_y)
										{
										#pragma omp critical
										{
											e[node_index * edge_add_parameter + m].weight = distance_new_dest;
											prev_node[node_index * edge_add_parameter + m] = dest_index;
										}
										}
									}
									if (node_index == start_x * maze_size + start_y)
									{
									#pragma omp critical
									{
										prev_node_from_source[new_dest_x * maze_size + new_dest_y] = dest_index;
									}
									}
									distance_x[distance_renew_index].weight = distance_new_dest;
								}
							}
						}
					}
					//	cout << "round " << node_index << " dijkstra end" << endl;

					// after running for rho rounds
					// harvesting r(v) from the distance array
					// we set r(v) = r_{rho}(v), which is the rho^{th} nearest neighbor
					// since the distance from the starting node to itself is zero then it should be 0
					// we find the (rho + 1)^{th} smallest number in the distance array

					vector<int> new_distance;
					for (int i = 0; i < cnt_edge[node_index]; ++i)
					{
						new_distance.push_back(e[node_index * edge_add_parameter + i].weight);
					}

//					cout << "returning (" << node_index/maze_size << "," << node_index%maze_size << ")" << endl;
//					cout << "new distance size: " << new_distance.size() << endl;
					if (new_distance.size() < rho)
					{
						if (new_distance.size() != 0)
						{
							sort(new_distance.begin(), new_distance.end());
							rv[ix][jx] = new_distance[new_distance.size() - 1];
						}
						else
						{
							rv[ix][jx] = 0;
						}
					}
                    else
					{
                    	if (rho > 0)
						{
							sort(new_distance.begin(), new_distance.end());
							rv[ix][jx] = new_distance[rho - 1];
						}
						else
						{
							rv[ix][jx] = new_distance[rho];
						}
					}
				}
			}
#pragma omp barrier
		}

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
		//	cout << "start radius stepping" << endl;
		int start_index = start_x * maze_size + start_y;
		int end_index = end_x * maze_size + end_y;

		// initialize set exclude_S to store the nodes that has not been iterated
		int exclude_S_size = 0;

		memset(exclude_S, 0, sizeof(int) * maze_size * maze_size);

		exclude_S_size = maze_size * maze_size - 1;
		exclude_S[start_index] = 1;

		gamma_distance[start_index] = 0;

		// assign the gamma distance if the there are edges between the starting point and the current node
		// we value every node v in the set N(s) and let gamma_distance[v] <- weight(s, v)
		for (int i = 0; i < cnt_edge[start_index]; ++i)
		{
			gamma_distance[e[start_index * edge_add_parameter + i].ending_index] = e[start_index * edge_add_parameter + i].weight;
		}

		int di;
		int execute_flag;

		while (exclude_S_size > 0)
		{
			// find the value of di
			di = 1 << 30;
			for (int i = 0; i < maze_size * maze_size; ++i)
			{
				if (exclude_S[i] == 0)
				{
					if (gamma_distance[i] != 1 << 30)
					{
						if (gamma_distance[i] + rv[i / maze_size][i % maze_size] < di)
						{
							di = gamma_distance[i] + rv[i / maze_size][i % maze_size];
						}
					}
				}
			}

			execute_flag = 1;
			// Bellman-Ford operation
			// execute until no gamma_distance[v] <= di was updated
			while (execute_flag)
			{
#pragma omp parallel shared(e, maze, cnt_edge, prev_node, prev_node_from_source, rv, w, gamma_distance, exclude_S, exclude_S_size, result, execute_flag, di) private(id)
				{
					id = omp_get_thread_num();

					// cout << "inside execute flag loop" << endl;
#pragma omp critical
					{
						execute_flag = 0;
					}
					
					for (int u = w * id * maze_size; u < w * (id + 1) * maze_size; ++u)
					{
						if (exclude_S[u] == 0)
						{
							if (maze[u / maze_size][u % maze_size] != -1 && gamma_distance[u] <= di)
							{
								for (int i = 1; i < cnt_edge[u]; ++i)
								{
									int v = e[u * edge_add_parameter + i].ending_index;
									int weight_u_v = e[u * edge_add_parameter + i].weight;
									if (maze[v / maze_size][v % maze_size] != -1 && gamma_distance[v] > gamma_distance[u] + weight_u_v)
									{
#pragma omp critical
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
#pragma omp barrier
				}
			}

			// renew S and exclude_S
			// if a node in S' gamma distance is less than or equal to di, then keep it in S, which means we should erase it from the exclude set
			// if a node in S' gamma distance is greater than di, then erase it from S, which means we should insert it to the exclude set
			exclude_S_size = 0;
			for (int i = 0; i < maze_size * maze_size; ++i)
			{
				if (gamma_distance[i] > di)
				{
					exclude_S[i] = 0;
					++exclude_S_size;
				}
				else
				{
					exclude_S[i] = 1;
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
		if (gamma_distance[end_index] == 1 << 30)
		{
			result = -1;
		}
		result = gamma_distance[end_index];
		
		end_time = omp_get_wtime();

		double delta_time = end_time - start_time;

		// output result
		string output_result_string = "case";
		if (case_i >= 10)
		{
			output_result_string += (char(int('0') + case_i / 10));
		}
		output_result_string += (char(int('0') + case_i % 10));
		output_result_string += ": ";

		//    check_edges();
		cout << "here is the time" << endl;
		
		cout << delta_time << endl;
		
		fout << output_result_string << delta_time << endl;
	}
	fout.close();
	cout << "successful" << endl;
	return 0;
}