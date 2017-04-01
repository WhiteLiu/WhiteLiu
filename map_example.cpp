#include <iostream>
#include  <map>
using namespace std;


struct lessthan
{
    bool operator()(const char& a,const char& b)
    {
        return a<b;
    }
};

ostream& operator <<(ostream& out,const pair<char,int>& p)
{
    return out<<p.first<<" "<<p.second<<endl;
}

int main()
{
    std::map<char,int,lessthan> map;
    for(char b = 'z';b!='A';b--)
    {
        map[b]++;
    }

    for(char b = 'A';b!='G';b++)
    {
        map[b]++;
    }

    for(int i = 0;i!= 10;i++)
    {
        map['b']++;
    }


    for(std::map<char,int>::iterator b=map.begin();b!= map.end();b++)
    {
        cout<< *b <<endl;
    }
    return 0;

}
