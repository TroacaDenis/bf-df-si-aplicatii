#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
using namespace std;
ifstream fin("biconex.in");
ofstream fout("biconex.out");

class Graph
{
    int n;                          //nr de noduri
    int m;                          //nr de muchii
    vector<vector<int> > neighbors; //vector ce contine cate un vector cu vecini pt fiecare nod
    bool oriented;                  // variabiabila pt a verifca daca e orientat
    bool from1;                     // variabila pt a verifica daca nodurile sunt numerotate de la 1
public:
    //constructori:
    Graph(int, bool, bool);
    Graph(int, int, bool, bool);

    void insert_edge(int, int); //functie pt a insera muchii

    vector<int> bfs(int); //functie pt a afla distantele minime de la un nod la celelate

    int connected_comp();          //functie pt a afla nr de componente conexe
    void dfs(int, vector<bool> &); //functie pt parcurgerea in adancime a unei componente

    vector<vector<int> > biconnected_comp();                                                                             //functie pt a afla nr de componente biconexe
    void biconnected_dfs(int, int &, vector<int> &, vector<int> &, stack<int> &, vector<vector<int> > &, vector<int> &); //functie pt parcurgerea in adancime

    vector<vector<int> > stronglyconnected_comp();                                                                              //functie pt a afla nr de componente tare conexe
    void stronglyconnected_dfs(int, int &, vector<int> &, vector<int> &, stack<int> &, vector<vector<int> > &, vector<bool> &); //functie pt parcurgerea in adancime

    vector<int> topological_sort();                          //functie ce returneaza un vector cu nodurile sortate topologic
    void topological_dfs(int, vector<bool> &, stack<int> &); //functie pt parcurgere in adancime a nodurilor

    bool havel_hakimi(vector<int> &); //functie ce verifica daca poate exista un graf cu gradele primite ca parametru

    vector<vector<int> > critical_connections();                                                            //functie ce returneaza un vector cu muchii critice                                                  //functie pt a afla nr de componente biconexe
    void cconnections_dfs(int, int &, vector<int> &, vector<int> &, vector<int> &, vector<vector<int> > &); //functie pt parcurgerea in adancime
};

Graph::Graph(int n, bool oriented = false, bool from1 = false)
{
    this->n = n;
    m = 0;
    this->from1 = from1;
    this->oriented = oriented;
    for (int i = 0; i < n; i++)
    {
        vector<int> aux;
        neighbors.push_back(aux);
    }
}
Graph::Graph(int n, int m, bool oriented = false, bool from1 = false)
{
    this->n = n;
    this->m = m;
    this->from1 = from1;
    this->oriented = oriented;
    for (int i = 0; i < n; i++)
    {
        vector<int> aux;
        neighbors.push_back(aux);
    }
}

void Graph::insert_edge(int x, int y)
{
    if (from1)
    {
        neighbors[x - 1].push_back(y - 1);
        if (!oriented)
            neighbors[y - 1].push_back(x - 1);
    }
    else
    {
        neighbors[x].push_back(y);
        if (!oriented)
            neighbors[y].push_back(x);
    }
}

vector<int> Graph::bfs(int x)
{
    vector<int> dist; //vector pt a memora distantele
    queue<int> aux;   //coada ce retine nodurile ce trebuie explorate
    for (int i = 0; i < n; i++)
        //nodurile nevizitate primesc distanta -1:
        dist.push_back(-1);
    if (from1)
        x--;
    aux.push(x);
    dist[x] = 0;
    while (!aux.empty())
    {
        for (int i = 0; i < neighbors[aux.front()].size(); i++)
        {
            //verificam daca nodurile vecine cu cel din varful cozii nu au fost vizitate:
            if (dist[neighbors[aux.front()][i]] == -1)
            {
                //retinem distanta:
                dist[neighbors[aux.front()][i]] = dist[aux.front()] + 1;
                //adaugam nodul vizitat in coada:
                aux.push(neighbors[aux.front()][i]);
            }
        }
        //trecem la urmatorul nod ce trebuie explorat:
        aux.pop();
    }
    return dist;
}

int Graph::connected_comp()
{
    int nr = 0;           //variabila pt a memora nr de componente conexe
    vector<bool> visited; // vector care verifica daca nodurile au fost vizitate
    for (int i = 0; i < n; i++)
        visited.push_back(false);
    for (int i = 0; i < n; i++)
    {
        if (visited[i] == false)
        {
            nr++;
            //facem parcurgere in adancime pt a vizita toate nodurile componentei conexe:
            dfs(i, visited);
        }
    }
    return nr;
}
void Graph::dfs(int x, vector<bool> &visited)
{
    visited[x] = true;
    for (int i = 0; i < neighbors[x].size(); i++)
        if (visited[neighbors[x][i]] == false)
            dfs(neighbors[x][i], visited);
}

vector<vector<int> > Graph::biconnected_comp()
{
    vector<int> disc;                //vector cu timpurile de descoperie ale nodurilor din dfs
    vector<int> low;                 //vector cu timpurile de descoperie minime la care pot ajunge nodurile din dfs prin muchii de intoarcere
    stack<int> nodes;                //stiva in care memoram nodurile vizitate
    vector<vector<int> > components; //vector cu componentele biconexe
    int disc_time = 0;               //variabila pt a cunoaste timpul curent de descoperire
    vector<int> parents;             //vecotr cu parintii nodurilor in dfs

    for (int i = 0; i < n; i++)
    {
        disc.push_back(-1);
        low.push_back(-1);
        parents.push_back(-1);
    }
    for (int i = 0; i < n; i++)
    {
        if (disc[i] < 0)
        {
            biconnected_dfs(i, disc_time, disc, low, nodes, components, parents);
            //dupa ce terminam dfs-ul, daca mai avem noduri in stiva, ele formeaza inca o componenta biconexa:
            if (!nodes.empty())
            {
                vector<int> aux;
                while (!nodes.empty())
                {
                    aux.push_back(nodes.top());
                    nodes.pop();
                }
                aux.push_back(i);
                components.push_back(aux);
            }
        }
    }
    return components;
}
void Graph::biconnected_dfs(int x, int &disc_time, vector<int> &disc, vector<int> &low, stack<int> &nodes, vector<vector<int> > &components, vector<int> &parents)
{
    disc[x] = disc_time;
    low[x] = disc_time;
    disc_time++;
    int children = 0; //variabila ce retine numar de copii al nodului
    for (int i = 0; i < neighbors[x].size(); i++)
    {
        if (disc[neighbors[x][i]] < 0)
        {
            children++;
            nodes.push(neighbors[x][i]);
            parents[neighbors[x][i]] = x;
            biconnected_dfs(neighbors[x][i], disc_time, disc, low, nodes, components, parents);
            low[x] = min(low[x], low[neighbors[x][i]]);
            //daca nodul curent este radacina si are mai mult de un nod fiu
            //sau daca are un nod fiu care nu poate ajunge la un timp de descoperire mai mic decat al tatalui prin muchie de intoarcere
            //inseamna ca nodul curent este nod critic
            if ((disc[x] == 0 && children > 1) || (disc[x] > 0 && disc[x] <= low[neighbors[x][i]]))
            {
                vector<int> aux; //variabila in care adaugam toate nodurile din componenta biconexa curenta(nodurile adaugate dupa nodul critic)
                while (nodes.top() != neighbors[x][i])
                {
                    aux.push_back(nodes.top());
                    nodes.pop();
                }
                nodes.pop();
                aux.push_back(neighbors[x][i]);
                aux.push_back(x);
                components.push_back(aux);
            }
        }
        else //intre noduri e muchie de intoarcere
            if (parents[x] != neighbors[x][i])
            low[x] = min(low[x], disc[neighbors[x][i]]);
    }
}

vector<vector<int> > Graph::stronglyconnected_comp()
{
    vector<int> disc;                //vector cu timpurile de descoperie ale nodurilor din dfs
    vector<int> low;                 //vector cu timpurile de descoperie minime la care pot ajunge nodurile din dfs prin muchii de intoarcere
    stack<int> nodes;                //stiva in care memoram nodurile vizitate
    vector<vector<int> > components; //vector cu componentele biconexe
    int disc_time = 0;               //variabila pt a cunoaste timpul curent de descoperire
    vector<bool> in_stack;           //vector care memoreaza daca nodurle sunt prezente in stiva

    for (int i = 0; i < n; i++)
    {
        disc.push_back(-1);
        low.push_back(-1);
        in_stack.push_back(false);
    }
    for (int i = 0; i < n; i++)
    {
        if (disc[i] < 0)
        {
            stronglyconnected_dfs(i, disc_time, disc, low, nodes, components, in_stack);
        }
    }
    return components;
}
void Graph::stronglyconnected_dfs(int x, int &disc_time, vector<int> &disc, vector<int> &low, stack<int> &nodes, vector<vector<int> > &components, vector<bool> &in_stack)
{
    disc[x] = disc_time;
    low[x] = disc_time;
    disc_time++;
    nodes.push(x);
    in_stack[x] = true;
    for (int i = 0; i < neighbors[x].size(); i++)
    {
        if (disc[neighbors[x][i]] < 0)
        {
            stronglyconnected_dfs(neighbors[x][i], disc_time, disc, low, nodes, components, in_stack);
            low[x] = min(low[x], low[neighbors[x][i]]);
        }
        else
            //verificam daca nodul vizitat mai este in stiva, caz in care avem muchie de intoarcere:
            if (in_stack[neighbors[x][i]])
            low[x] = min(low[x], disc[neighbors[x][i]]);
    }
    //daca nodul curent nu poate ajunge la un timp de descoperire mai mic decat al sau, atunci este radacina unuei componente tare conexe:
    if (low[x] == disc[x])
    {
        vector<int> aux; //variabila in care adaugam toate nodurile din componenta tare conexa curenta
        while (nodes.top() != x)
        {
            aux.push_back(nodes.top());
            in_stack[nodes.top()] = false;
            nodes.pop();
        }
        aux.push_back(nodes.top());
        in_stack[nodes.top()] = false;
        nodes.pop();
        components.push_back(aux);
    }
}

vector<int> Graph::topological_sort()
{
    vector<bool> visited; //vector care retine daca nodurile au fost vizitate
    stack<int> nodes;     // stiva cu nodurile vizitate
    for (int i = 0; i < n; i++)
    {
        visited.push_back(false);
    }
    for (int i = 0; i < n; i++)
    {
        if (!visited[i])
        {
            topological_dfs(i, visited, nodes);
        }
    }
    vector<int> aux; //vector ce contine nodurile sortate topologic
    while (!nodes.empty())
    {
        aux.push_back(nodes.top());
        nodes.pop();
    }
    return aux;
}
void Graph::topological_dfs(int x, vector<bool> &visited, stack<int> &nodes)
{
    visited[x] = true;
    for (int i = 0; i < neighbors[x].size(); i++)
    {
        if (!visited[neighbors[x][i]])
            topological_dfs(neighbors[x][i], visited, nodes);
    }
    nodes.push(x);
}

bool Graph::havel_hakimi(vector<int> &degrees)
{
    while (!degrees.empty())
    {
        //sortam vectorul in ordine descrecatoare a gradelor nodurilor ramase:
        sort(degrees.begin(), degrees.end(), greater<int>());
        if (degrees[0] == 0) //in tot vectorul gradele sunt egale cu 0
            return true;
        if (degrees[0] > degrees.size() - 1) //gradul unui nod este mai mare decat nr de noduri disponibile
            return false;
        //stergem nodul cu cel mai mare grad si pentru fiecare vecin al nodului sters micsoram unul din gradele ramase cu 1:
        int aux = degrees[0];
        degrees.erase(degrees.begin());
        for (int i = 0; i < aux; i++)
        {
            degrees[i]--;
            if (degrees[i] < 0)
                return false;
        }
    }
    return true;
}

vector<vector<int> > Graph::critical_connections()
{
    vector<int> disc;                  //vector cu timpurile de descoperie ale nodurilor din dfs
    vector<int> low;                   //vector cu timpurile de descoperie minime la care pot ajunge nodurile din dfs prin muchii de intoarcere
    vector<int> parents;               //vector cu parintii nodurile in dfs
    vector<vector<int> > cconnections; //vector cu muchii critice
    int disc_time = 0;                 //variabila pt a cunoaste timpul curent de descoperire

    for (int i = 0; i < n; i++)
    {
        disc.push_back(-1);
        low.push_back(-1);
        parents.push_back(-1);
    }
    for (int i = 0; i < n; i++)
    {
        if (disc[i] < 0)
        {
            cconnections_dfs(i, disc_time, disc, low, parents, cconnections);
        }
    }
    return cconnections;
}
void Graph::cconnections_dfs(int x, int &disc_time, vector<int> &disc, vector<int> &low, vector<int> &parents, vector<vector<int> > &cconnections)
{
    disc[x] = disc_time;
    low[x] = disc_time;
    disc_time++;
    for (int i = 0; i < neighbors[x].size(); i++)
    {
        if (disc[neighbors[x][i]] < 0)
        {
            parents[neighbors[x][i]] = x;
            cconnections_dfs(neighbors[x][i], disc_time, disc, low, parents, cconnections);
            low[x] = min(low[x], low[neighbors[x][i]]);
            //daca nodul curent are un nod fiu care nu poate ajunge la un timp de descoperire mai mic decat al tatalui prin muchie de intoarcere
            //inseamna ca de la nodul curent la acel nod fiu avem muchie critica
            if (disc[x] < low[neighbors[x][i]])
            {
                vector<int> aux;
                aux.push_back(x);
                aux.push_back(neighbors[x][i]);
                cconnections.push_back(aux);
            }
        }
        else //intre noduri e muchie de intoarcere
            if (parents[x] != neighbors[x][i])
            low[x] = min(low[x], disc[neighbors[x][i]]);
    }
}

class Solution
{
public:
    vector<vector<int> > criticalConnections(int n, vector<vector<int> > &connections)
    {
        Graph g(n);
        for (int i = 0; i < n; i++)
        {
            g.insert_edge(connections[i][0], connections[i][1]);
        }
        vector<vector<int> > aux;
        aux = g.critical_connections();
        return aux;
    }
};

int main()
{
    int n, m, a, b;
    fin >> n >> m;
    Graph g(n, m, false, true);
    for (int i = 0; i < m; i++)
    {
        fin >> a >> b;
        g.insert_edge(a, b);
    }
    vector<vector<int> > aux;
    aux = g.biconnected_comp();
    fout << aux.size() << '\n';
    for (int i = 0; i < aux.size(); i++)
    {
        for (int j = 0; j < aux[i].size(); j++)
            fout << aux[i][j] + 1 << " ";
        fout << '\n';
    }
}