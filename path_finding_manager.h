//
// Created by juan-diego on 3/29/24.
// OPTIMIZED VERSION
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H

#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <cmath>
#include <queue>
#include <climits>
#include <iostream>
#include <chrono>
using namespace std;

enum Algorithm {
    Dijkstra,
    AStar,
    BestFirst
};

class PathFindingManager {
private:
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    int render_counter = 0;
    const int RENDER_FREQUENCY = 50;

    struct Entry {
        Node* node;
        double dist;
        double priority; // Para A* y Best First

        bool operator < (const Entry& other) const {
            return priority > other.priority;  // Min-heap
        }
    };

    // DIJKSTRA OPTIMIZADO
    void dijkstra(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> dist;
        std::unordered_set<Node *> visited;

        // Inicializar distancias
        for (const auto& [id, node] : graph.nodes) {
            dist[node] = INFINITY;
        }
        dist[src] = 0;

        priority_queue<Entry> pq;
        pq.push({src, 0, 0});

        while (!pq.empty()) {
            Entry current_entry = pq.top();
            pq.pop();

            Node* current = current_entry.node;

            if (visited.find(current) != visited.end()) {
                continue;
            }
            visited.insert(current);
            if (current == dest) {
                break;
            }
            for (Edge* edge : current->edges) {
                Node* vecino = nullptr;
                if (edge->src == current) {
                    vecino = edge->dest;
                } else if (!edge->one_way && edge->dest == current) {
                    vecino = edge->src;
                } else {
                    continue;
                }
                if (visited.find(vecino) != visited.end()) {
                    continue;
                }
                double nueva_dist = dist[current] + edge->length;
                if (nueva_dist < dist[vecino]) {
                    dist[vecino] = nueva_dist;
                    parent[vecino] = current;
                    pq.push({vecino, nueva_dist, nueva_dist});

                    visited_edges.push_back(sfLine(current->coord, vecino->coord,
                                                  sf::Color::Yellow, 1.5f));

                    render_counter++;
                    if (render_counter % RENDER_FREQUENCY == 0) {
                        render();
                    }
                }
            }
        }

        set_final_path(parent);
    }

    // A* OPTIMIZADO
    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> g_score;
        std::unordered_set<Node *> visited;

        // Inicializar
        for (const auto& [id, node] : graph.nodes) {
            g_score[node] = INFINITY;
        }
        g_score[src] = 0;

        priority_queue<Entry> pq;
        double h_start = euclid(src, dest);
        pq.push({src, 0, h_start});

        while (!pq.empty()) {
            Entry current_entry = pq.top();
            pq.pop();

            Node* current = current_entry.node;

            if (visited.find(current) != visited.end()) {
                continue;
            }

            visited.insert(current);

            if (current == dest) {
                break;
            }

            for (Edge* edge : current->edges) {
                Node* vecino = nullptr;

                if (edge->src == current) {
                    vecino = edge->dest;
                } else if (!edge->one_way && edge->dest == current) {
                    vecino = edge->src;
                } else {
                    continue;
                }

                if (visited.find(vecino) != visited.end()) {
                    continue;
                }

                double tentative_g = g_score[current] + edge->length;

                if (tentative_g < g_score[vecino]) {
                    parent[vecino] = current;
                    g_score[vecino] = tentative_g;
                    double f_score = tentative_g + euclid(vecino, dest);
                    pq.push({vecino, tentative_g, f_score});

                    visited_edges.push_back(sfLine(current->coord, vecino->coord,
                                                  sf::Color::Yellow, 1.5f));

                    render_counter++;
                    if (render_counter % RENDER_FREQUENCY == 0) {
                        render();
                    }
                }
            }
        }

        set_final_path(parent);
    }

    void best_first_search(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_set<Node *> visited;

        priority_queue<Entry> pq;
        double h_start = euclid(src, dest);
        pq.push({src, 0, h_start});

        while (!pq.empty()) {
            Entry current_entry = pq.top();
            pq.pop();

            Node* current = current_entry.node;

            if (visited.find(current) != visited.end()) {
                continue;
            }

            visited.insert(current);

            if (current == dest) {
                break;
            }

            for (Edge* edge : current->edges) {
                Node* vecino = nullptr;

                if (edge->src == current) {
                    vecino = edge->dest;
                } else if (!edge->one_way && edge->dest == current) {
                    vecino = edge->src;
                } else {
                    continue;
                }

                if (visited.find(vecino) != visited.end()) {
                    continue;
                }

                double heuristic = euclid(vecino, dest);

                if (parent.find(vecino) == parent.end()) {
                    parent[vecino] = current;
                    pq.push({vecino, 0, heuristic}); // dist = 0 porque no importa en Best First

                    // Agregar para visualización
                    visited_edges.push_back(sfLine(current->coord, vecino->coord,
                                                  sf::Color::Yellow, 1.5f));

                    // Renderizar con menos frecuencia
                    render_counter++;
                    if (render_counter % RENDER_FREQUENCY == 0) {
                        render();
                    }
                }
            }
        }

        set_final_path(parent);
    }

    // Heurística euclidiana optimizada
    double euclid(Node* a, Node* b) {
        double dx = a->coord.x - b->coord.x;
        double dy = a->coord.y - b->coord.y;
        return sqrt(dx * dx + dy * dy);
    }

    void render() {
        if (current_graph == nullptr) {
            return;
        }

        window_manager->clear();

        // Dibujar grafo base (solo nodos, no todas las aristas)
        for (const auto& [id, node] : current_graph->nodes) {
            node->draw(window_manager->get_window());
        }

        // Dibujar aristas visitadas
        for (const sfLine& line : visited_edges) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar nodos especiales
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }

        window_manager->display();

        // Pausa más corta para mayor velocidad
        sf::sleep(sf::milliseconds(1));
    }

    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        path.clear();
        Node* current = dest;

        while (current != nullptr && parent.find(current) != parent.end()) {
            Node* prev = parent[current];
            if (prev != nullptr) {
                path.push_back(sfLine(prev->coord, current->coord,
                                     sf::Color::Red, 3.0f));
            }
            current = prev;
        }
    }

    Graph* current_graph = nullptr;

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }

        // Limpiar y preparar
        path.clear();
        visited_edges.clear();
        render_counter = 0;
        current_graph = &graph;

        // Medir tiempo de ejecución
        auto start = std::chrono::high_resolution_clock::now();

        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            case BestFirst:
                best_first_search(graph);
                break;
            default:
                break;
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::cout << "Algoritmo ejecutado en: " << duration.count() << " ms" << std::endl;
        std::cout << "Nodos explorados: " << visited_edges.size() << std::endl;
        std::cout << "Longitud del camino: " << path.size() << " segmentos" << std::endl;
    }

    void reset() {
        path.clear();
        visited_edges.clear();
        render_counter = 0;

        if (src) {
            src->reset();
            src = nullptr;
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
        }
    }

    void draw(bool draw_extra_lines) {
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};

#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H