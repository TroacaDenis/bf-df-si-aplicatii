#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <utility>
#include <queue>
#include <climits>
#include <map>
using namespace std;
ifstream fin("dfs.in");
ofstream fout("dfs.out");
 
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
 
    int max_flow(int, int);                                           //functie ce primeste ca parametrii sursa si destinatia si returneaza fluxul maxim al retelei
    bool max_flow_bfs(int **, int, int, vector<int> &, queue<int> &); //functie pentru a parcurge graful residual
 
    vector<vector<int> > roy_floyd(); //functie care returneaza matricea distantelor minime dintre 2 noduri
 
    int darb();          //functie de returneaza diametrul unui arbore
    int darb_bfs(int &); //functie pentru parcurgere in adancime ce returneaza distanta panaa la ultimul nod si il salveaza pe acesta in variabila data ca parametru
 
    vector<int> eulerian_circuit(); //functie ce returneaza un vector cu nodurile parcurse in ciclul Eulerian
 
    int hamiltonian_cycle(); //functie ce returneaza costul minim al unui ciclu hamiltonian

    vector<int>max_matching(int, int, int &); //functie ce returneaza un vector cu perechiile nodurilor din partea stanga a cuplajului maxim si numarul de muchii al acestuia ca parametru
    bool matching_bfs(int, int, vector<int> &, vector<int> &, vector<int> &); //functie ce verifica daca mai exista un drum augmentativ 
    bool matching_dfs(int, int, vector<int> &, vector<int> &, vector<int> &, int); //functie ce verifica daca putem avea drum augmentativ ce incepe cu nodul transmis ca parametru
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
 
    //toate nodurile dintre x si radacina primesc drept parinte pe radacina, pt a face gasirea radacinii unui nod mai rapida:
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
            //cand gasim un nod, il trecem ca vizitat si modificam cheia si parintele nodurilor vecine cu acesta:
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
            //cand gasim un nod, il trecem ca vizitat si modificam distantele pana la nodurile vecine cu acesta:
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
                    //avem doar n-1 etape, deci un nod nu poate sa fie introdus in coada de mai multe ori decat daca avem ciclu infinit:
                    //o etapa = modificarea veciniilor tuturor nodurilor din coada care au fost modificate in acelasi timp
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
    vector<int> parents(n, -1); //vector cu parintii fiecarui nod in bfs-ul grafului residual
    queue<int> sink_neighbors;  //coada care pastreaza frunzele legate de destinatie printr-o muchie nesaturata
    int maximum_flow = 0;       //variabila ce memoreaza fluxul maxim
    int **residual_graph;       //graf residual
    residual_graph = new int *[n];
    for (int i = 0; i < n; i++)
        residual_graph[i] = new int[n]();
    //muchiile grafului residual au la inceput capacitatiile muchiilor grafului initial, iar muchiile inverse au capacitatea 0:
    for (int i = 0; i < n; i++)
        for (int j = 0; j < neighbors[i].size(); j++)
            residual_graph[i][neighbors[i][j]] = weights[i][j];
 
    //cat timp se poate ajunge de la sursa la destinatie in arborele residual:
    while (max_flow_bfs(residual_graph, s, d, parents, sink_neighbors))
    {
        //pentru fiecare nod din care se ajunge la radacina printr-o muchie nesaturata, vizitam nodurile de la destinatie la sursa:
        while (!sink_neighbors.empty())
        {
            parents[d] = sink_neighbors.front();
            sink_neighbors.pop();
            int current_flow = INT_MAX; //fluxul ce poate fi transmis de la sursa la destinatie prin nodurile parinte din bfs
 
            //fluxul curent este egal cu capacitatea minima a muchiilor parcurse din graful residual:
            for (int i = d; i > s; i = parents[i])
                current_flow = min(current_flow, residual_graph[parents[i]][i]);
 
            //micsoram fluxul muchiilor vizitate si il marim pe ce al inverselor:
            for (int i = d; i > s; i = parents[i])
            {
                residual_graph[parents[i]][i] -= current_flow;
                residual_graph[i][parents[i]] += current_flow;
            }
            //adaugam fluxul curent la cel total:
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
                //daca nodul curent este nodul destinatie, nu il adaugam in bfs si ii adaugam parintele in coada vecinilor destinatiei:
                if (i == d)
                    sink_neighbors.push(q.front());
                else
                {
                    q.push(i);
                    visited[i] = true;
                    parents[i] = q.front();
                }
            }
        }
        q.pop();
    }
    //cand sink_neighbors nu are niciun element, inseamna ca nu see poate ajunge de la sursa la destinatie
    return !sink_neighbors.empty();
}
 
vector<vector<int> > Graph::roy_floyd()
{
    vector<vector<int> > dist; //matrice cu distantele
    //initial distantele sunt egale cu costurile muchiilor din graf sau cu infinit daca 2 noduri nu sunt legate prin o singura muchie:
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
 
    //pt fiecare nod k, daca este nod intermediar intre alte 2 noduri i si j, incercam sa modificam distanta de la i la j:
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
    //facem bfs din primul nod, iar x o sa primeasca valoarea ultimului nod vizitat:
    diametru = darb_bfs(x);
    //facem bfs din ultimul nod vizitat, iar diametrul o sa fie egal cu distanta de la acesta la ultimul nod din noul bfs:
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
 
vector<int> Graph::eulerian_circuit(){
    stack<int> nodes; //stiva ce retine nodurile din ciclul format
    vector<int> sol; //vector cu nodurile din ciclul eulerian
    vector<vector<int> > remaining_edges(n); //vector cu vectori ce retin ce muchii pornesc dintr-un nod
    int edge_nr = 0; //variabila folosita pentru a cunoaste ce muchie trebuie adaugata in remaining_edges
    vector<pair<int, int> > edge_nodes; //vector ce retine capetele unei muchii
    vector<bool> visited(m, false); //vector ce verifica daca o muchie a fost vizitata

    for(int i = 0; i < n; i++)
        for(int j = 0; j < neighbors[i].size(); j++){
            remaining_edges[i].push_back(edge_nr);
            remaining_edges[ neighbors[i][j] ].push_back(edge_nr);
            edge_nodes.push_back(make_pair(i, neighbors[i][j]));
            edge_nr++;
        }
    //verificam daca nu putem avea ciclu eulerian:
    for(int i = 0; i < n; i++)
        if(neighbors[i].size()%2 != 0)
            return sol;

    nodes.push(0);
    while(!nodes.empty()){
        int current_node = nodes.top();
        //alegem o muchie care pleaca din nod si nu a fost vizitata inca:
        if(!remaining_edges[current_node].empty()){
            int current_edge = remaining_edges[current_node].back();
            //stergem muchia aleasa:
            remaining_edges[current_node].pop_back();
            if(!visited[current_edge]){
                //marcam muchia ca vizitata si adaugam nodul la care duce in stiva, fara a il elimina pe cel precedent:
                visited[current_edge] = true;
                int next_node;
                if(current_node == edge_nodes[current_edge].first)
                    next_node = edge_nodes[current_edge].second;
                else
                    next_node = edge_nodes[current_edge].first;
                nodes.push(next_node);
            }
        }
        else{
            //dupa ce am vizitat toate muchiile care pleaca din nod, il stergem din stiva si il adaugam la solutie
            nodes.pop();
            sol.push_back(current_node);
        }
    }
    //nu mai trebuie sa afisam muchia cu care ne intoarcem la nodul din care am plecat, deci stergem un nod:
    sol.pop_back();
    return sol;
    
}
 
int Graph::hamiltonian_cycle(){
    int C[1<<n][n]; //matrice cu costurile minime C[i][j] ale lanturilor ce incep de la nodul 0 si se termina cu nodul j
                    //lanturile folosesc doar noduri egale cu pozitiile in care apare 1 in reprezentarea binara a lui i
    for(int i = 0; i < 1<<n; i++)
        for(int j = 0; j < n; j++)
            C[i][j] = 100000000; //setam costurile ca fiind infinit
    C[1][0] = 0;

    for(int i = 0; i < 1<<n; i++)
        for(int j = 0; j < n; j++)
            //daca pe pozitia j din reprezentarea binara a lui i avem 1:
            if(i & (1<<j))
                //verficam daca lantul curent poate folosi nodurile vecine cu j:
                for(int k = 0; k < neighbors[j].size(); k++)
                    if(i & (1<<neighbors[j][k]))
                        //costul minim al vecinului este minimul dintre el si costul minim al lantului care se termina cu j si nu il foloseste pe vecin + costul muchiei de la j la vecin:
                        C[i][ neighbors[j][k] ] = min(C[i][ neighbors[j][k] ], C[i ^ (1<<neighbors[j][k])][j] + weights[j][k]);
    
    int sol = 100000000;
    //pentru toate nodurile verificam daca se poate ajunge la nodul 0 printr-o muchie:
    for(int i = 1; i < n; i++)
        for(int j = 0; j < neighbors[i].size(); j++)
            if (neighbors[i][j] == 0)
                //costul minim este minimul dintre el si costul minim al lantului ce contine toate nodurile si se termina in nodul i + costul muchiei de la i la 0:
                sol = min(sol, C[(1<<n)-1][i] + weights[i][j]);

    return sol;
}

vector<int> Graph::max_matching(int nr_left, int nr_right, int &nr){
    //vom avea noduri de la 1 la nr_left in partea stanga si de la 1 la nr_right in partea dreapta
    //folosim si nodul 0 care este initial conectat la toate nodurile din stanga si din dreapta
    //daca gasim un drum alternant care pleaca din 0 si ajunge tot in acesta, atunci avem drum augmentativ
    vector<int> right_match(nr_left + 1, 0); //vector cu perechile din stanga ale nodurilor din dreapta
    vector<int> left_match(nr_right + 1, 0); //vector cu perechile din stanga ale nodurilor din dreapta
    vector<int> dist(nr_left + 1, 0); //vector ce ne ajuta sa verificam daca nodurile fac parte din cuplaj si sa parcurgem un drum augmentativ

    nr = 0;
    //cat timp avem drumuri augmentative:
    while(matching_bfs(nr_left, nr_right, right_match, left_match, dist)){
        //verificam din ce noduri avem drumuri augmentative:
        for(int i = 1; i <= nr_left; i++)
            if(right_match[i] == 0 && matching_dfs(nr_left, nr_right, right_match, left_match, dist, i))
                nr++;
    }
    return right_match;
}
bool Graph::matching_bfs(int nr_left, int nr_right, vector<int> &right_match, vector<int> &left_match, vector<int> &dist){
    queue<int> q; //coada cu nodurile libere din stanga(noduri ce nu fac parte din cuplaj)
    for(int i = 1; i <= nr_left; i++){
        //daca un nod este liber distanta e 0, altfel e infinit
        if(right_match[i] == 0){
            dist[i] = 0;
            q.push(i);
        }
        else
            dist[i] = INT_MAX;
    }
    dist[0] = INT_MAX;

    while(!q.empty()){
        int left_node = q.front();
        q.pop();
        if(left_node != 0 && dist[left_node] < INT_MAX){
            //pentru fiecare vecin posibil din dreapta al nodului verificam daca perechea lui din stanga face parte din cuplaj (sau este nodul 0):
            for(int i = 0; i < neighbors[left_node - 1].size(); i++){
                int righ_node = neighbors[left_node - 1][i] + 1;
                if(dist[ left_match[righ_node] ] == INT_MAX){
                    //de la nodul left_node pana la nodul left_match[right_node] avem o muchie care nu face parte din cupal si una care face parte
                    //deci putem sa adaugam nodul left_match[right_node] in q pentru ca dupa el urmeaza o muchie care nu face parte din cuplaj
                    dist[ left_match[righ_node] ] = dist[left_node] + 1;
                    q.push(left_match[righ_node]);
                }
            }
        }
    }
    //daca nu avem drum alternant care sa ajunga in 0, atungi nu avem drum augmentativ:
    return (dist[0] != INT_MAX);
}
bool Graph::matching_dfs(int nr_left, int nr_right, vector<int> &right_match, vector<int> &left_match, vector<int> &dist, int left_node){
    if(left_node != 0){
        //urmam drumul augmentativ setat de bfs prin vectorul dist:
        for(int i = 0; i < neighbors[left_node - 1].size(); i++){
            int righ_node = neighbors[left_node - 1][i] + 1;
            if(dist[ left_match[righ_node] ] == dist[left_node] + 1){
                if(matching_dfs(nr_left, nr_right, right_match, left_match, dist, left_match[righ_node])){
                    left_match[righ_node] = left_node;
                    right_match[left_node] = righ_node;
                    return true;
                }
            }
        }
        dist[left_node] = INT_MAX;
        return false;
    }
    //cand ajungem in nodul 0, inseamna ca avem drum augmentativ
    return true;
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
    int n, m, a, b;
    fin >> n >> m;
    Graph g(n, m, false, true);
    for (int i = 0; i < m; i++)
    {
        fin >> a >> b;
        g.insert_edge(a, b);
    }
    fout << g.connected_comp();
}