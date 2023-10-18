#include <vector>
#include <algorithm>
#include <map>
#include <set>
//#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <typed_array.h>

using namespace std;

// asked to pass a reference (obj & x), makes the code easier/more modern
// could pass a pointer, which makes the code a little clunkier:
// (*v).begin() and (*v).end()  to get the value of the object at the pointer..
void sort_by_magnitude(vector<double> & v) {

    auto c = [](double x, double y) { return abs(x) > abs(y); };
    sort(v.begin(), v.end(), c);

}

/*
    Expects a valid matrix represented in text (rectangular and double)
    Sets with m.get(row).set(column, value);

    Lazy - simply throws for any error encoutered.  

    String parsing - in <cstring> fundamentally doing char * 
        strtok "tokenizes" an input cstring, but modifies the input..
        sscanf "could be used if we had a specific format string to parse"
    Stringstream and getline(stream, token, delim) seemed
        like the simplest and most modern

*/
TypedArray<TypedArray<double>> read_matrix_csv(const string path) {
    TypedArray<TypedArray<double>> m;   // 
    ifstream file(path);
    
    string line;
    string token;
    const char delim = ',';
    set<int> row_dim;

    int i = 0, j = 0;
    //cout << "read_matrix_csv( "  << path << " )" << endl;
    while( getline(file, line)) {
        stringstream ss(line);
        
        while (getline(ss, token, delim)) {
            //cout << "i: " << i << " j: " << j << "val: " << token << endl; 
            m.get(i).set(j, stod(token));
            j++;
        }
        row_dim.insert(j);
        i++;
        j=0;
    }
    //cout << "value of i: " << i << endl;
    // do sanity checking on input matrix.  for each row, verify that size is same
    //cout << "number of row_dims: " << row_dim.size() << endl;
    if (row_dim.size() != 1) {
        throw std::invalid_argument("Invalid matrix input");
    }
    
    return m;
}

/* 
    Takes a valid matrix.  (Does not make consistency checks)
    And writes the output in a standard form
*/
void write_matrix_csv(const TypedArray<TypedArray<double>> &matrix, const string path) {
    ofstream file(path);
    int nRow = matrix.size();
    for(int i = 0; i< nRow; i++) {
        int nCol = matrix.safe_get(i).size();
        for(int j = 0; j<nCol; j++) {
            file << matrix.safe_get(i).safe_get(j);
            if (j < (nCol-1)) {
                file << ",";
            } else {
                file << endl;
            }

        }

    }
}

/*
    Takes in a file path to ascii file - and processes all *valid* words in the file
    Outputs a map of word occurences
    Does not attempt to do any error handling..

*/
map<string, int> occurrence_map(const string path) {
    
    map<string, int> m;
    ifstream file(path);
    string line;
    string token;
    const char delim = ' ';
    bool token_flag = true;
    while( getline(file, line)) {

        stringstream ss(line);
        
        // get each line, parse the line for valid word tokens, generate a word count
        while (getline(ss, token, delim)) {
            /* 
            rules:
                split on whitespace, (I do not split on tabs)
                remove starting and trailing quotes and valid punctuation [().,:;!?], 
                remove certain begining or trailing punctuation
                check for invalid chars
            */
            //cout << "Original token: " << token << endl;
            token_flag = true;
            int i = 0;
            char c; 
            while ( i == 0 ) {
                c = token.front();
                if (ispunct(c)) {   // don't remove single quote '
                    if (c == '\'') { i = 1; } 
                    else {  token.erase(0,1); } // remove other leading punct
                } else {  // don't remove non punct
                    i = 1;
                }
            }
            i = 0;
            while ( i == 0 ) {
                c = token[token.size() - 1 ];
                if (ispunct(c)) {
                    if (c == '\'') { i = 1; } 
                    else { token.erase(token.size()-1,1); } // remove other leading punct}
                } else {
                    i = 1;
                }
            }

            // check for validity - is alnum or '
            for( char c : token) {
                if (!isalnum(c)) {
                    if (c != '\'') {
                        token_flag = false;
                        // could break here..
                    } 
                }
            }
            // generate to lower
            i = 0;
            for( char c : token) {
                token[i]=tolower(c);
                i++;
            }
            //cout << "Processed token: " << token << " is valid: " << token_flag << endl;
            if (token_flag) {
                m[token] = m[token]+1;
            }
        }
    }
    return m;
}