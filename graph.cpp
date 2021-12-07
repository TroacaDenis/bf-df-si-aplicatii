#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <utility>
#include <queue>
#include <climits>
using namespace std;
ifstream fin("maxflow.in");
ofstream fout("maxflow.out");

class Graph
{
    int n;                          //nr de noduri
    int m;                          //nr de muchii
    vector<vector<int> > neighbors; //vector ce contine cate un vector cu vecini pt fiecare nod
    vector<vector<int> > weights;   //vector ce contine cate un vector cu costurile pana la vecinii fiecarui nod
    bool oriented;                  // variabiabila pt a verifca daca e orientat
    bool from1;                     // variabila pt a verifica daca nodurile sunt numerotate de la 1
public:
    //constructori:
    Graph(int);
    Graph(int, bool, bool);
    Graph(int, int, bool, bool);

    void insert_edge(int, int);        //functie pt a insera muchii
    void insert_weight(int, int, int); //functie pt a insera costuri

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

    vector<vector<int> > critical_connections();                                                            //functie ce returneaza un vector cu muchii critice
    void cconnections_dfs(int, int &, vector<int> &, vector<int> &, vector<int> &, vector<vector<int> > &); //functie pt parcurgerea in adancime

    void disjoint(queue<pair<int, pair<int, int> > >); //functie ce primeste mai multe operatii si numere asupra carora se aplica si afiseaza raspunsurile operatiilor de tip 2
    int disjoint_root(int, vector<int> &);             //functie ce returneaza radacina arborelui din care face parte nodul si uneste cu radacina toate nodurile parcurse pana la aceasta

    vector<int> apm(int &); //functie ce returneaza un vector cu parintii fiecarui nod din arborele partial de cost minim al grafului si costul total al muchiilor acestuia(transmis ca parametru)

    vector<int> dijkstra(); //functie ce returneaza un vector cu lungimile drumurilor de la primul nod la toate celelalte

    vector<int> bellman_ford(); //functie ce returneaza un vector cu lungimile drumurilor de la primul nod la toate celelalte sau un vector fara elemente daca avem ciclu negativ

    int max_flow(int, int);
    bool max_flow_bfs(int **, int, int, vector<int> &, queue<int> &);

    vector<vector<int> > roy_floyd();

    int darb();
    int darb_bfs(int &);
};

Graph::Graph(int n)
{
    this->n = n;
}
Graph::Graph(int n, bool oriented, bool from1)
{
    this->n = n;
    m = 0;
    this->from1 = from1;
    this->oriented = oriented;
    for (int i = 0; i < n; i++)
    {
        vector<int> aux;
        neighbors.push_back(aux);
        weights.push_back(aux);
    }
}
Graph::Graph(int n, int m, bool oriented, bool from1)
{
    this->n = n;
    this->m = m;
    this->from1 = from1;
    this->oriented = oriented;
    for (int i = 0; i < n; i++)
    {
        vector<int> aux;
        neighbors.push_back(aux);
        weights.push_back(aux);
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
void Graph::insert_weight(int x, int y, int z)
{
    if (from1)
    {
        weights[x - 1].push_back(z);
        if (!oriented)
            weights[y - 1].push_back(z);
    }
    else
    {
        weights[x].push_back(z);
        if (!oriented)
            weights[y].push_back(z);
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

void Graph::disjoint(queue<pair<int, pair<int, int> > > q)
{
    vector<int> parents; //vector cu parintii fiecarui nod in arborii din padurile de multimi
    vector<int> heights; //vector cu inaltimile arborilor(facem height[radacina] pt a afla inaltimea)

    for (int i = 0; i <= n; i++)
    {
        parents.push_back(-1);
        heights.push_back(1);
    }

    while (!q.empty())
    {
        int op_type, x, y, root_x, root_y;
        op_type = q.front().first;
        x = q.front().second.first;
        y = q.front().second.second;
        q.pop();
        //aflam radacinile celor 2 noduri:
        root_x = disjoint_root(x, parents);
        root_y = disjoint_root(y, parents);

        if (op_type == 1)
        {
            //unim arborele cu inaltimea mai mica de cel cu inaltime mai mare:
            if (heights[root_x] > heights[root_y])
                parents[root_y] = root_x;
            else
            {
                parents[root_x] = root_y;
                //daca arborii au aceeasi inaltime, trebuie sa modificam inaltimea unuia dintre ei:
                if (heights[root_x] == heights[root_y])
                    heights[root_y]++;
            }
        }
        else
        {
            //daca radacinile arborilor din care fac parte 2 noduri sunt identice, nodurile fac parte din acelasi arbore:
            if (root_x == root_y)
                fout << "DA" << '\n';
            else
                fout << "NU" << '\n';
        }
    }
}
int Graph::disjoint_root(int x, vector<int> &parents)
{
    int root_node = x;
    while (parents[root_node] != -1)
        root_node = parents[root_node];

    while (parents[x] != -1)
    {
        int aux = parents[x];
        parents[x] = root_node;
        x = aux;
    }
    return root_node;
}

vector<int> Graph::apm(int &total_weight)
{
    vector<int> keys;                                                                      //vector cu costurile minime de la arborele curent la fiecare nod
    vector<int> parents;                                                                   //vector cu parintii nodurilor din apm
    vector<bool> visited;                                                                  //vector care verifica daca un nod a fost adaugat in apm
    priority_queue<pair<int, int>, vector<pair<int, int> >, greater<pair<int, int> > > pq; //priority queue de minim ce memoreaza cheia unui nod si nodul
    total_weight = 0;                                                                      //costul total al apm-ului
    for (int i = 0; i < n; i++)
    {
        keys.push_back(INT_MAX); //setam costurile minime ca fiind infinit
        parents.push_back(-1);
        visited.push_back(false);
    }
    //adaugam primul nod in coada:
    keys[0] = 0;
    pq.push(make_pair(keys[0], 0));

    //cat timp avem elemente in coada, le stergem pana gasim nodul de cost minim care nu a fost vizitat:
    while (!pq.empty())
    {
        int current_node = pq.top().second;
        int current_key = pq.top().first;
        pq.pop();
        if (!visited[current_node])
        {
            //cand gasim un nod, il trecem ca vizitat si mdoificam cheia si parintele nodurilor vecine cu acesta:
            visited[current_node] = true;
            total_weight += current_key;
            for (int i = 0; i < neighbors[current_node].size(); i++)
            {
                //daca vecinul nu a fost adaugat deja in apm si distanta de la nod la el e mai mica decat cheia lui, il modificam si il punem in coada:
                if (!visited[neighbors[current_node][i]] && weights[current_node][i] < keys[neighbors[current_node][i]])
                {
                    keys[neighbors[current_node][i]] = weights[current_node][i];
                    parents[neighbors[current_node][i]] = current_node;
                    pq.push(make_pair(keys[neighbors[current_node][i]], neighbors[current_node][i]));
                }
            }
        }
    }
    return parents;
}

vector<int> Graph::dijkstra()
{
    vector<int> distances;                                                                 //vector cu distantele minime de la primul nod la celelalte
    vector<bool> visited;                                                                  //vector care verifica daca un nod a fost adaugat in apm
    priority_queue<pair<int, int>, vector<pair<int, int> >, greater<pair<int, int> > > pq; //priority queue de minim ce memoreaza distanta pana la un nod si nodul

    for (int i = 0; i < n; i++)
    {
        distances.push_back(INT_MAX); //setam distantele minime ca fiind infinit
        visited.push_back(false);
    }
    //adaugam primul nod in coada:
    distances[0] = 0;
    pq.push(make_pair(distances[0], 0));

    //cat timp avem elemente in coada, le stergem pana gasim nodul de distanta minima care nu a fost vizitat:
    while (!pq.empty())
    {
        int current_node = pq.top().second;
        pq.pop();
        if (!visited[current_node])
        {
            //cand gasim un nod, il trecem ca vizitat si mdoificam distantele pana la nodurile vecine cu acesta:
            visited[current_node] = true;
            for (int i = 0; i < neighbors[current_node].size(); i++)
            {
                //daca suma dintre distanta pana la nod si lungimea drumului de la nod la vecin este mai mica decat distanta pana la vecin, ii modificam distanta si il punem in coada:
                if (!visited[neighbors[current_node][i]] && (distances[current_node] + weights[current_node][i]) < distances[neighbors[current_node][i]])
                {
                    distances[neighbors[current_node][i]] = distances[current_node] + weights[current_node][i];
                    pq.push(make_pair(distances[neighbors[current_node][i]], neighbors[current_node][i]));
                }
            }
        }
    }
    for (int i = 0; i < distances.size(); i++)
        if (distances[i] == INT_MAX)
            distances[i] = 0;
    return distances;
}

vector<int> Graph::bellman_ford()
{
    queue<int> q;                 //coada cu nodurile ale caror costuri au fost modificate
    vector<int> costs;            //vector cu costurile minime de la primul nod la celelalte
    vector<bool> in_queue;        //vector care verifica daca un nod se afla in coada
    vector<int> in_queue_counter; //vector care numara de cate ori a fost adaugat un nod in coada
    bool negative_cycle = false;  //variabila ce verifica daca avem un ciclu de cost negativ

    for (int i = 0; i < n; i++)
    {
        costs.push_back(INT_MAX); //setam costurile minime ca fiind infinit
        in_queue.push_back(false);
        in_queue_counter.push_back(0);
    }
    //adaugam primul nod in coada:
    costs[0] = 0;
    q.push(0);
    in_queue[0] = true;
    in_queue_counter[0]++;

    //cat timp avem elemente in coada si nu avem ciclu de cost negativ, modificam costurile vecinilor primului nod din coada si il stergem:
    while (!q.empty() && !negative_cycle)
    {
        int current_node = q.front();
        q.pop();
        in_queue[current_node] = false;
        for (int i = 0; i < neighbors[current_node].size(); i++)
        {
            //daca suma dintre costul nodului si costul muchiei de la nod la vecin e mai mare decat costul vecinului, il modificam:
            if (costs[current_node] + weights[current_node][i] < costs[neighbors[current_node][i]])
            {
                costs[neighbors[current_node][i]] = costs[current_node] + weights[current_node][i];
                //daca vecinul nu este deja in coada, il adaugam:
                if (!in_queue[neighbors[current_node][i]])
                {
                    //avem doar n-1 etape, deci un nod du poate sa fie introdus in coada de mai multe ori decat daca avem ciclu infinit:
                    //o etapa = modificare veciniilor tuturor nodurilor din coada care au fost modificate in acelasi timp
                    if (in_queue_counter[neighbors[current_node][i]] >= n)
                        negative_cycle = true;
                    else
                    {
                        q.push(neighbors[current_node][i]);
                        in_queue[neighbors[current_node][i]] = true;
                        in_queue_counter[neighbors[current_node][i]]++;
                    }
                }
            }
        }
    }
    if (negative_cycle)
        costs.clear();
    return costs;
}

int Graph ::max_flow(int s, int d)
{
    if (from1)
    {
        s--;
        d--;
    }
    vector<int> parents(n, -1);
    queue<int> sink_neighbors;
    int maximum_flow = 0;
    int **residual_graph;
    residual_graph = new int *[n];
    for (int i = 0; i < n; i++)
        residual_graph[i] = new int[n]();
    for (int i = 0; i < n; i++)
        for (int j = 0; j < neighbors[i].size(); j++)
            residual_graph[i][neighbors[i][j]] = weights[i][j];

    while (max_flow_bfs(residual_graph, s, d, parents, sink_neighbors))
    {
        while (!sink_neighbors.empty())
        {
            parents[d] = sink_neighbors.front();
            sink_neighbors.pop();
            int current_flow = INT_MAX;
            for (int i = d; i > s; i = parents[i])
                current_flow = min(current_flow, residual_graph[parents[i]][i]);

            for (int i = d; i > s; i = parents[i])
            {
                residual_graph[parents[i]][i] -= current_flow;
                residual_graph[i][parents[i]] += current_flow;
            }

            maximum_flow += current_flow;
        }
    }

    return maximum_flow;
}
bool Graph ::max_flow_bfs(int **residual_graph, int s, int d, vector<int> &parents, queue<int> &sink_neighbors)
{
    queue<int> q;
    vector<bool> visited(n, false);
    q.push(s);
    visited[s] = true;

    while (!q.empty())
    {
        for (int i = 0; i < n; i++)
        {
            if (residual_graph[q.front()][i] && !visited[i])
            {
                if (i == d)
                    sink_neighbors.push(q.front());
                else{
                    q.push(i);
                    visited[i] = true;
                    parents[i] = q.front();
                }
                
            }
        }
        q.pop();
        
    }
    return !sink_neighbors.empty();
}

vector<vector<int> > Graph::roy_floyd()
{
    vector<vector<int> > dist;
    for (int i = 0; i < n; i++)
    {
        vector<int> aux;
        for (int j = 0; j < n; j++)
        {
            if (weights[i][j] != 0 || i == j)
                aux.push_back(weights[i][j]);
            else
                aux.push_back(INT_MAX);
        }
        dist.push_back(aux);
    }

    for (int k = 0; k < n; k++)
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX && dist[i][k] + dist[k][j] < dist[i][j])
                    dist[i][j] = dist[i][k] + dist[k][j];
    return dist;
}

int Graph::darb()
{
    int diametru;
    int x = 0;
    diametru = darb_bfs(x);
    diametru = darb_bfs(x);
    return diametru;
}
int Graph::darb_bfs(int &x)
{
    vector<int> dist;
    queue<int> aux;
    for (int i = 0; i < n; i++)
        dist.push_back(-1);
    aux.push(x);
    dist[x] = 1;
    while (!aux.empty())
    {
        for (int i = 0; i < neighbors[aux.front()].size(); i++)
        {
            if (dist[neighbors[aux.front()][i]] == -1)
            {
                dist[neighbors[aux.front()][i]] = dist[aux.front()] + 1;
                aux.push(neighbors[aux.front()][i]);
            }
        }

        x = aux.front();
        aux.pop();
    }
    return dist[x];
}

class Solution
{
public:
    vector<vector<int> > criticalConnections(int n, vector<vector<int> > &connections)
    {
        Graph g(n, false, false);
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
    int n, m, x, y, z;
    fin >> n >> m;
    Graph g(n, m, true, true);
    for (int i = 0; i < m; i++)
    {
        fin >> x >> y >> z;
        g.insert_edge(x, y);
        g.insert_weight(x, y, z);
    }
    fout<<g.max_flow(1, n);
}