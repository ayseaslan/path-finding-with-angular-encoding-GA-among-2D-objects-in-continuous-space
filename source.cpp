#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>
#include <SFML/Graphics.hpp>

// GA parameters 
const int POPULATION_SIZE = 10000;
const int NUM_GENERATIONS = 200;
const double MUTATION_RATE = 0.1;
const double SIMILAR_RATE = 0.5;
const int infeas_penalty = 10000;
const int n_random_sols = 100;
const int tournament_size = 10; 
const int n_make_similar = 1;
const int n_seed_straight_angle = 1;

// 2D environment parameters 
const double delta = 0.1; // safety distance 
const int bound_x = 5; //  width of the 2D env.
const int bound_y = 8; //  height of the 2D env. 
const double step = 0.25; // distance taken in a second 
const int Tmax = 100; // limit the number of seconds of movement to consider in feasible paths

std::uniform_real_distribution<> dist(0.0, 1.0);


struct Point {
    double x, y;
};

struct Region {
    Point p1, p2;
};

struct RectObject {
    sf::RectangleShape shape;
    RectObject(float x, float y, float width, float height) {
        shape.setPosition(x, y);
        shape.setSize(sf::Vector2f(width, height));
        shape.setFillColor(sf::Color::Blue);
    }
};



// Structure to represent a point
struct Point_visual {
    sf::CircleShape shape;
    Point_visual(float x, float y) {
        shape.setPosition(x, y);
        shape.setRadius(3);
        shape.setFillColor(sf::Color::Red);
    }
};

struct IdxCompareAscending
{
    const std::vector<int>& target;

    IdxCompareAscending(const std::vector<int>& target) : target(target) {}

    bool operator()(int a, int b) const { return target[a] < target[b]; }
};

double distance(Point a, Point b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

int infeasible(int l, std::vector<Point>& path, std::vector<Region>& blocked) {
    int infeasibility = 0;
    for (int i = 0; i < l; i++) {
        if (path[i].x<0 || path[i].x>bound_x || path[i].y<0 || path[i].y>bound_y) { // out of boundary feasibility
            infeasibility++;
        }
        for (int j = 0; j < blocked.size(); j++) {
            if (path[i].x >= blocked[j].p1.x - delta && path[i].x <= blocked[j].p2.x + delta && path[i].y >= blocked[j].p1.y - delta && path[i].y <= blocked[j].p2.y + delta) {
                infeasibility++;
            }
        }
    }
    return infeasibility;
}

int fitness(std::vector<Point>& path, std::vector<Region>& blocked, Point terminal) {

    int reachtime = -1;
    for (int i = 1; i < path.size(); i++) {
        int infeas = infeasible(i, path, blocked);
        if (infeas != 0) {
            break;
        }
        else {
            double d = distance(terminal, path[i - 1]);
            if (d < step) { // within a reachable distance now
                reachtime = i - 1;
                break;
            }
        }

    }
    if (reachtime == -1) {
        reachtime = infeas_penalty;
    }

    return reachtime;

}


std::vector<Point> decodePath(const std::vector<Point>& points, const std::vector<double>& angles) {
    std::vector<Point> path(points.size());
    path[0] = points[0];
    for (int i = 0; i < angles.size(); i++) {
        Point current = path[i];
        Point next;
        next.x = current.x + step * cos(angles[i]);
        next.y = current.y + step * sin(angles[i]);
        path[i + 1] = next;
    }
    return path;
}

std::vector<double> createRandomAngles(int l) {
    std::vector<double> angles(l, 0.0);
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-M_PI, M_PI);
    for (int i = 0; i < angles.size(); i++) {
        double d = distribution(generator);
        angles[i] = d;

    }
    return angles;
}

std::vector<double> crossover(const std::vector<double>& a, const std::vector<double>& b, int best_fitness) {
    int crossoverPoint = rand() % best_fitness;
    std::vector<double> child(a.size());
    for (int i = 0; i < crossoverPoint; i++) {
        child[i] = a[i];
    }
    for (int i = crossoverPoint; i < b.size(); i++) {
        child[i] = b[i];
    }
    return child;
}

std::vector<double> crossover_average(const std::vector<double>& a, const std::vector<double>& b) {
    int si = std::min(a.size(), b.size());
    std::vector<double> child(si);
    for (int i = 0; i < child.size(); i++) {
        child[i] = (a[i] * 0.5 + b[i] * 0.5);
    }

    return child;
}

std::vector<double> crossover_per(const std::vector<double>& a, const std::vector<double>& b) {
    int si = std::min(a.size(), b.size());

    std::vector<double> child(si);
    std::default_random_engine generator;
    int bias_a = 1;
    double bias = dist(generator);
    if (bias > 0.5) {
        bias_a = 0;
    }

    for (int i = 0; i < child.size(); i++) {

        double coin = dist(generator);
        if (coin <= 0.5) {
            if (bias_a == 1)
                child[i] = a[i];
            else
                child[i] = b[i];
        }
        else {
            if (bias_a == 1)
                child[i] = b[i];
            else
                child[i] = a[i];

        }
    }


    return child;
}

void mutate(std::vector<double>& angles) {

    if (rand() < MUTATION_RATE * RAND_MAX) {
        int a = rand() % angles.size();
        int b = rand() % angles.size();
        std::swap(angles[a], angles[b]);


    }

}

void make_similar(std::vector<double>& a, int best_fitness) {


    if (rand() < SIMILAR_RATE * RAND_MAX) {
        int a_p = rand() % best_fitness;
        if (a_p > 0 && a_p < a.size())
            a[a_p] = a[a_p - 1];
        if (a_p >= 0 && a_p < a.size() - 1)
            a[a_p + 1] = a[a_p];

    }


}

void make_short_angle(std::vector<double>& a, int best_fitness, double slope) {


    if (rand() < SIMILAR_RATE * RAND_MAX) {
        int a_p = rand() % best_fitness;
        if (a_p >= 0 && a_p < a.size())
            a[a_p] = slope;

    }


}

std::pair<int, int> tournament_selection(const std::vector<int>& fitnesses, int tournament_size) {
    std::vector<int> tournament_participants(tournament_size);
    std::generate(tournament_participants.begin(), tournament_participants.end(), []() { return rand() % POPULATION_SIZE; });

    int best_participant = tournament_participants[0];
    int changed = 0;
    for (int i = 1; i < tournament_size; i++) {
        if (fitnesses[tournament_participants[i]] > fitnesses[best_participant]) {
            best_participant = tournament_participants[i];
            changed = 1;
        }
    }

    int second_best_participant = tournament_participants[0];
    if (changed == 0)second_best_participant = tournament_participants[1];
    for (int i = 0; i < tournament_size; i++) {
        if (tournament_participants[i] != best_participant &&
            fitnesses[tournament_participants[i]] > fitnesses[second_best_participant]) {
            second_best_participant = tournament_participants[i];
        }
    }

    return std::make_pair(best_participant, second_best_participant);
}

using namespace std;
const int seed_manual = 10;
mt19937 rng(seed_manual);

int main() {

    // the input file contains the lower and upper ends of the rectangular objects, and the centroid coordinates
    stringstream name;
    name << "coordinates.txt"; 
    string na = name.str();
    ifstream infile(na.c_str());

    // data structures of the corner points and centroids 
    vector<Point>station_centroids;
    vector<vector<Point>>station_corners;

    // read the data from the file 
    int A = 0; // the number of objects 
    while (infile)
    {
        string s;
        if (!getline(infile, s))
            break;

        istringstream ss(s);
        vector<Point>v;
        Point p1 = { 0.0,0.0 }; Point p2 = { 0.0,0.0 }; Point p3 = { 0.0,0.0 };

        int c = 0;
        while (ss)
        {
            string s;
            if (!getline(ss, s, ',')) break;
            double f = stof(s);
            if (c == 0) {
                p1.x = f;
            }
            if (c == 1) {
                p1.y = f;
            }
            if (c == 2) {
                p2.x = f;
            }
            if (c == 3) {
                p2.y = f;
            }
            if (c == 4) {
                p3.x = f;
            }
            if (c == 5) {
                p3.y = f;
            }
            c++;

        }
        v.push_back(p1); v.push_back(p2);
        station_corners.push_back(v);
        cout << "corner " << p1.x << " " << p1.y << " and " << p2.x << " " << p2.y << endl;
        station_centroids.push_back(p3);
        A++;
    }

    
   
    int initial_index = 0; // select the object that path departs from 
    int terminal_index = 4; // select the object that path should lead to 


    // create the output file taht reports the best path found 
    FILE* pFile;
    stringstream nameSum;
    nameSum << "paths_between_rectangular_objects.csv"; 
    fopen_s(&pFile, nameSum.str().c_str(), "w");

  
            Point terminal = station_centroids[terminal_index];
            Point initial = station_centroids[initial_index];
            vector<Region>blocked; // treat other objects as blocked regions 

            for (int i = 0; i < A; i++) {
                if (i != initial_index && i != terminal_index) {
                    Point first = station_corners[i][0];
                    Point second = station_corners[i][1];
                    Region r = { {0.0, 0.0}, {0.0, 0.0} };
                    r.p1 = first;
                    r.p2 = second;
                    blocked.push_back(r);
                }

            }
           
          

            // the dummy solution staying at the origin  
            vector<Point> pointsInit;
            for (int i = 0; i < Tmax+1; i++) {
                pointsInit.push_back(initial);
            }

            // the path based on following the straight angle 
            vector<double> anglesStraight(Tmax, 0.0);
            double slope = atan2(terminal.y - initial.y, terminal.x - initial.x);
            for (int a = 0; a < Tmax; a++)
                anglesStraight[a] = slope;

            // population data 
            vector<vector<Point>> populationPoints;
            vector<vector<double>> populationAngles;
            vector<int>fitnesspopulation;

            // add the straight angle solution to the population 
            populationAngles.push_back(anglesStraight);
            vector<Point> pointsSt = decodePath(pointsInit, anglesStraight);
            populationPoints.push_back(pointsSt);
            int fst = fitness(pointsSt, blocked, terminal);
            fitnesspopulation.push_back(fst);


            // the rest of the population are generated randomly 
            for (int g = 1; g < POPULATION_SIZE; g++) {
                vector<double> angles = createRandomAngles(Tmax);
              populationAngles.push_back(angles);
                vector<Point> points = decodePath(pointsInit, angles);
                populationPoints.push_back(points);
                int f = fitness(points, blocked, terminal);
                fitnesspopulation.push_back(f);
            }

            // find the best fitness initial solution 
            vector<int> sortedFitness;
            for (size_t k = 0; k < fitnesspopulation.size(); ++k) {
                sortedFitness.push_back(k);
            }
            sort(sortedFitness.begin(), sortedFitness.end(), IdxCompareAscending(fitnesspopulation));
            int best_fitness = fitnesspopulation[sortedFitness[0]];
            int best_sol = sortedFitness[0]; // the solution index of the best solution in the population 
            int fitness_init = best_fitness;
            cout << "best fitness initial " << best_fitness << endl;

            

          
            // the generations start here 
            for (int g = 0; g < NUM_GENERATIONS; g++) {

                vector<vector<Point>> populationPointsNew;
                vector<vector<double>> populationAnglesNew;
                vector<int>fitnesspopulationNew(POPULATION_SIZE, infeas_penalty);

                // always pass the best fitness solution from the previous generation to the nest one 
                populationAnglesNew.push_back(populationAngles[best_sol]);
                populationPointsNew.push_back(populationPoints[best_sol]);
                fitnesspopulationNew[0] = best_fitness;
                best_sol = 0;

                // make new solutions
                for (int p = 1; p < POPULATION_SIZE; p++) {
                    vector<double>anglesChild = createRandomAngles(Tmax);
                   
                    if (p <= POPULATION_SIZE - n_random_sols) {
                        pair<int, int>indices_parents = tournament_selection(fitnesspopulation, tournament_size); 
                        double coin = dist(rng);

                        // apply one of the three crossover opers 
                        if (coin <= 0.9) {
                            anglesChild = crossover(populationAngles[indices_parents.first], populationAngles[indices_parents.second], min(50, best_fitness));

                        }
                        else if (coin > 0.9 && coin <= 0.95) {
                            anglesChild = crossover_per(populationAngles[indices_parents.first], populationAngles[indices_parents.second]);

                        }
                        else {
                            anglesChild = crossover_average(populationAngles[indices_parents.first], populationAngles[indices_parents.second]);

                        }

                    }


                    // apply other operators on the child solution 
                    for (int n = 0; n < n_make_similar; n++)
                        make_similar(anglesChild, Tmax/2);

                    for (int n = 0; n <n_seed_straight_angle; n++)
                        make_short_angle(anglesChild, Tmax / 2, slope);

                    mutate(anglesChild);


                    // then add it to the population 
                    populationAnglesNew.push_back(anglesChild);

                    // decode the new solution and calculate its fitness 
                    vector<Point> pointsChild = decodePath(pointsInit, anglesChild);
                    populationPointsNew.push_back(pointsChild);
                    int f = fitness(pointsChild, blocked, terminal);
                    fitnesspopulationNew[p] = f;

                    //best fitness update 
                    if (f < best_fitness) {
                        best_fitness = f;
                        best_sol = p;
                    }



                }
                
                // update the current population 
                populationAngles = populationAnglesNew;
                populationPoints = populationPointsNew;
                fitnesspopulation = fitnesspopulationNew;

                
                cout << "GENERATION " << g << " BEST FITNESS " << best_fitness << endl;
            }

            

             if (best_fitness <= Tmax) {

                 // print the best found path
                 for (int i = 0; i <= best_fitness; i++) {
                     fprintf(pFile, "%f, %f \n", populationPoints[best_sol][i].x, populationPoints[best_sol][i].y);

                 }

                 // visualize the objects and the path in the 2D environment 
                 int c = 100;

                 sf::RenderWindow window(sf::VideoMode(bound_x * c, bound_y * c), "2D Environment Visualization");

                 // Create rectangular objects
                 std::vector<RectObject> objects;
                 for (int i = 0; i < A; i++) {
                     float lower_x = station_corners[i][0].x;
                     float lower_y = station_corners[i][0].y;
                     float width = station_corners[i][1].x - station_corners[i][0].x;
                     float height = station_corners[i][1].y - station_corners[i][0].y;

                     objects.emplace_back(lower_x * c, lower_y * c, width * c, height * c);

                 }

                 // Create points
                 std::vector<Point_visual> points;
                 for (int i = 0; i <= best_fitness; i++) {
                     points.emplace_back(c * populationPoints[best_sol][i].x, c * populationPoints[best_sol][i].y);
                 }



                 while (window.isOpen()) {
                     sf::Event event;
                     while (window.pollEvent(event)) {
                         if (event.type == sf::Event::Closed)
                             window.close();
                     }

                     window.clear(sf::Color::White);

                     // Draw rectangular objects
                     for (const auto& object : objects) {
                         window.draw(object.shape);

                     }

                     // Draw points
                     for (const auto& point : points) {
                         window.draw(point.shape);
                     }

                     window.display();
                 }
             }

             return 0; 

}


