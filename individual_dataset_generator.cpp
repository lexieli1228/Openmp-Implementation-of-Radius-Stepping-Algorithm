// serial version of radius stepping algorithm
// the implementation of algorithm 1 in the original paper

#include <bits/stdc++.h>
#include <fstream>
using namespace std;

// maze size of the benchmark
// original: 10000
// size of paper: 1000
#define maze_size 1000


/*generate_data
  generate testing data into maze array*/
void generate_data(int case_i)
{
    string case_i_name = "../benchmark_self/case";
    if (case_i >= 10)
    {
    	case_i_name += (char(int('0') + case_i/10));
	}
    case_i_name += (char(int('0') + case_i%10));
    case_i_name += ".txt";
    ofstream case_i_file;
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
	
	int start_index = rand()%(maze_size*maze_size);
	int end_index = rand()%(maze_size*maze_size);
	while (start_index == end_index)
	{
		end_index = rand()%(maze_size*maze_size);
	}

	for (int i = 0; i < maze_size; ++ i)
	{
		for (int j = 0; j < maze_size; ++ j)
		{
			int index_x = i*maze_size + j;
			if (index_x == start_index)
			{
				case_i_file << "S" << " ";
			}
			else if (index_x == end_index)
			{
				case_i_file << "D" << " ";
			}
			else
			{	
			    case_i_file << rand()%7 - 1 << " ";
		    }
		}
	}                        
	
	case_i_file.close();
    return;
}


/* main function */
int main()
{
    // overall there are 50 cases
    for (int case_i = 0; case_i < 50; ++ case_i)
    {
    	
        generate_data(case_i);	
    }
    cout << "generate data successful" << endl;
    return 0;
}