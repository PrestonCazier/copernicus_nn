#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstring>
#include <string>
using namespace std;

#define MAIN "./eng_sentences.csv"
#define PATH "./sentences"
#define MAX 50000

int main()
{
    string buffer, filepath;
    int i, k = 1, count = 0;
    bool ascii = true;

    ifstream fd;
    ofstream out;
    ostringstream filename;
    
    fd.open(MAIN);

    while(getline(fd,buffer))
    {
        if(count == 0)
        {
            filename << PATH << k << ".txt";
            filepath = filename.str().c_str();
            out.open(filepath.c_str());
            k++;
        }

        for(i=0;i<buffer.length();i++)
        {
            if(buffer[i]<0 || buffer[i] >= 128)
            {
                ascii = false;
                break;
            }
        }
        
        if(ascii == true)
        {
            out << buffer << endl;
            count++;
        }

        if(count >= MAX-1)
        {
            filename.str("");
            filename.clear();
            count = 0;
            out.close();
        }
        ascii = true;
    }
    fd.close();
    return 1;
}

