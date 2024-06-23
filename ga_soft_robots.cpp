#include <iostream>
#include <queue>
#include <random>
#include <fstream> 
#include <vector>
#include <iterator>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <unordered_map>

struct Point {
  double x, y, z;
  Point(double x, double y, double z) : x(x), y(y), z(z) {}
  Point() : x(0), y(0), z(0) {}

  bool operator==(const Point &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

template <>
struct std::hash<Point> {
  std::size_t operator()(const Point &p) const {
    using std::size_t;
    using std::hash;

    return ((hash<double>()(p.x)) ^ (hash<double>()(p.y)) ^ (hash<double>()(p.z)));
  }
};

// Global Variables
Point g = Point(0, 0, -9.81);
double dt = 0.00015;
double T = 0;
double k_f = 20;
double k_d = 0.9999;
double k_c = 100000;
double u_s = 1;
double u_k = 10;

class Mass {
public:
  Point pos;
  Point vel;
  Point forces;
  double mass;
  Mass *connection;

  Mass() : pos(Point()), vel(Point()), forces(Point()), mass(0), connection(nullptr) {}
  Mass(Point p, double m) : pos(p), mass(m), connection(nullptr) {}
};

class Spring {
public:
  Mass *m1;
  Mass *m2;
  double k;
  double *L;

  Spring() : m1(nullptr), m2(nullptr), k(0), L(nullptr) {}
  Spring(Mass *mass1, Mass *mass2, double k_spr, double *len) : m1(mass1), m2(mass2), k(k_spr), L(len) {}
};

class Cube {
public:
  std::vector<Mass*> mass_list;
  std::vector<Spring> spring_list;
  double a;
  double b;
  double c;

  Cube() : mass_list(8), spring_list(0), a(0), b(0), c(0) {}
  Cube(std::vector<Mass*> ml, std::vector<Spring> sl, double a_c, double b_c, double c_c) : mass_list(ml), spring_list(sl), a(a_c), b(b_c), c(c_c) {}
};

// Function to generate initial points
std::vector<Point> generatePoints(Point center) {
  std::vector<Point> points;
  
  points.emplace_back(center.x + 0, center.y + 0, center.z + 0);
  points.emplace_back(center.x + 0, center.y + 0, center.z + 1);
  points.emplace_back(center.x + 0, center.y + 1, center.z + 0);
  points.emplace_back(center.x + 0, center.y + 1, center.z + 1);
  points.emplace_back(center.x + 1, center.y + 0, center.z + 0);
  points.emplace_back(center.x + 1, center.y + 0, center.z + 1);
  points.emplace_back(center.x + 1, center.y + 1, center.z + 0);
  points.emplace_back(center.x + 1, center.y + 1, center.z + 1);
  return points;
}

// Function to calculate Euclidean Distance
double euclideanDistance(const Point& p1, const Point& p2) {
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double dz = p2.z - p1.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Function to generate initial springs
std::vector<std::pair<int, int>> generateSpringConnections() {
  std::vector<std::pair<int, int>> connections;

  for (int i=0; i<8; ++i) {
    for (int j=i+1; j<8; ++j) {
      connections.emplace_back(i, j);
    }
  }
  return connections;
}

// Function to pair m1 and m2 Masses with springs
void generateSprings(Cube &cube, const std::vector<std::pair<int, int>> &connections) {
  for (int i=0; i<connections.size(); i++) {
    int massIndex1 = connections[i].first;
    int massIndex2 = connections[i].second;

    Mass *mass1 = cube.mass_list[massIndex1];
    Mass *mass2 = cube.mass_list[massIndex2];

    // Calculate Euclidean Distances
    double *distance = (double*)malloc(sizeof(double*));
    *distance = euclideanDistance(mass1->pos, mass2->pos);

    cube.spring_list.emplace_back(mass1, mass2, k_f, distance);
  }
}

void writeToFile(std::ofstream &myfile, std::vector<Mass*> &mass_list) {
  // Format: (x, y, z)_m1, (x, y, z)_m2, ...
  for (int i=0; i<mass_list.size(); i++) {
    //std::cout << "(" << mass_list[i]->pos.x << ", " << mass_list[i]->pos.y << ", " << mass_list[i]->pos.z << ")" << std::endl;
    myfile << "(" << mass_list[i]->pos.x << ", " << mass_list[i]->pos.y << ", " << mass_list[i]->pos.z << ")";
    if (i != mass_list.size()-1) {
      myfile << "; ";
    }
  }
  myfile << "\n";
}

double dot(Point a, Point b) {
  double res = 0;
  res = res + a.x * b.x;
  res = res + a.y * b.y;
  res = res + a.z * b.z;
  return res;
}

double norm(Point a, Point b) {
  double dist = 0;
  double x = a.x - b.x;
  double y = a.y - b.y;
  double z = a.z - b.z;

  dist = std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2);
  dist = std::sqrt(dist);
  return dist;
}

double roundDouble(double value, int dec) {
  double factor = std::pow(10, dec);
  return std::round(value * factor) / factor;
}
/*
void connectCubes(Cube &cube1, Cube &cube2) {
  for (int i=0; i<cube1.mass_list.size(); i++) {
    cube1.spring_list.emplace_back(cube1.mass_list[i], cube2.mass_list[i], k_f, euclideanDistance(cube1.mass_list[i]->pos, cube2.mass_list[i]->pos));
  }
}
*/
double simulatePrint(std::vector<Cube*> *cube_list, double max_time) {
  // Set time variable
  T = 0;

  // Origin center of mass
  Point origin = (*cube_list)[0]->mass_list[0]->pos;
  //std::cout << "sim print" << std::endl;
  // Create mass tags
  std::vector<Mass *> total_masses;
  for (int i=0; i<(*cube_list).size(); i++) {
    for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
      total_masses.push_back((*cube_list)[i]->mass_list[j]);
    }
  }
  for (int i=0; i<total_masses.size(); i++) {
    for (int j=0; j<total_masses.size(); j++) {
      if (total_masses[i]->pos == total_masses[j]->pos && i != j) {
        total_masses[i]->connection = total_masses[j];
        total_masses[j]->connection = total_masses[i];
      }
    }
  }

  // Start simulation
  std::ofstream myfile;
  myfile.open("positions3.txt");

  int S = 0;
  while (T < max_time) {
    // Simulate each cube
    //std::cout << T << std::endl;
    for (int i=0; i<(*cube_list).size(); i++) {
      // Set up dictionaries
      std::unordered_map<Mass*, std::vector<Spring>> springMap;
      for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
        std::vector<Spring> springs;
        for (int k=0; k<(*cube_list)[i]->spring_list.size(); k++) {
          if ((*cube_list)[i]->spring_list[k].m1 == (*cube_list)[i]->mass_list[j]) {
            springs.push_back((*cube_list)[i]->spring_list[k]);
          } else if ((*cube_list)[i]->spring_list[k].m2 == (*cube_list)[i]->mass_list[j]) {
            springs.push_back((*cube_list)[i]->spring_list[k]);
          }
        }
        springMap[(*cube_list)[i]->mass_list[j]] = springs;
      }

      // Send current positions of masses to txt file
      if (S % 1000 == 0) {
        writeToFile(myfile, (*cube_list)[i]->mass_list);
      }

      // Check for changes in forces
      for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
        Point force(0, 0, 0);
        std::vector<Spring> springs = springMap[(*cube_list)[i]->mass_list[j]];

        // Spring forces
        for (int k=0; k<springs.size(); k++) {
          double length = *(springs[k].L) + (*cube_list)[i]->a + (*cube_list)[i]->b * sin(T * 2 * M_PI * (*cube_list)[i]->c)/10;

          Mass *mass1;
          Mass *mass2;
          if ((*cube_list)[i]->mass_list[j] == springs[k].m1) {
            mass1 = springs[k].m1;
            mass2 = springs[k].m2;
          } else {
            mass1 = springs[k].m2;
            mass2 = springs[k].m1;
          }

          Point unit(0, 0, 0);
          double unit_norm = norm(mass1->pos, mass2->pos);
          unit.x = (mass1->pos.x - mass2->pos.x) / norm(mass1->pos, mass2->pos);
          unit.y = (mass1->pos.y - mass2->pos.y) / norm(mass1->pos, mass2->pos);
          unit.z = (mass1->pos.z - mass2->pos.z) / norm(mass1->pos, mass2->pos);

          Point f_s(0, 0, 0);
          f_s.x = unit.x * (k_f * (norm(mass1->pos, mass2->pos) - length));
          f_s.y = unit.y * (k_f * (norm(mass1->pos, mass2->pos) - length));
          f_s.z = unit.z * (k_f * (norm(mass1->pos, mass2->pos) - length));

          force.x = force.x - f_s.x;
          force.y = force.y - f_s.y;
          force.z = force.z - f_s.z;
        }

        // Gravity forces
        double f_g = (*cube_list)[i]->mass_list[j]->mass * g.z;
        force.z = force.z + f_g;

        // Restoration forces
        double z_pos = (*cube_list)[i]->mass_list[j]->pos.z;
        if (z_pos < 0) {
          force.z = force.z - (k_c * z_pos);
        }

        // Friction forces
        if (z_pos <= 0) {
          double f_h = std::sqrt(std::pow(force.x, 2) + std::pow(force.y, 2));
          double f_n = (*cube_list)[i]->mass_list[j]->mass * g.z;
          if (f_n < 0) {
            if (f_h < -f_n * u_s) {
              force.x = 0;
              force.y = 0;
            } else if (f_h >= -f_n * u_s) {
              force.x = force.x - (u_k * f_n);
              force.y = force.y - (u_k * f_n);
            }
          }
        }

        (*cube_list)[i]->mass_list[j]->forces = force;
      }

      // Update mass pos, acc, vel
      for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
        Point acc(0, 0, 0);
        acc.x = roundDouble(((*cube_list)[i]->mass_list[j]->forces.x / (*cube_list)[i]->mass_list[j]->mass), 3);
        acc.y = roundDouble(((*cube_list)[i]->mass_list[j]->forces.y / (*cube_list)[i]->mass_list[j]->mass), 3);
        acc.z = roundDouble(((*cube_list)[i]->mass_list[j]->forces.z / (*cube_list)[i]->mass_list[j]->mass), 5);

        Point velocity(0, 0, 0);
        velocity.x = roundDouble(((*cube_list)[i]->mass_list[j]->vel.x + (acc.x * dt)) * k_d, 3);
        velocity.y = roundDouble(((*cube_list)[i]->mass_list[j]->vel.y + (acc.y * dt)) * k_d, 3);
        velocity.z = roundDouble(((*cube_list)[i]->mass_list[j]->vel.z + (acc.z * dt)) * k_d, 5);

        Point position(0, 0, 0);
        position.x = roundDouble((*cube_list)[i]->mass_list[j]->pos.x + (velocity.x * dt), 3);
        position.y = roundDouble((*cube_list)[i]->mass_list[j]->pos.y + (velocity.y * dt), 3);
        position.z = roundDouble((*cube_list)[i]->mass_list[j]->pos.z + (velocity.z * dt), 5);

        // Check tags
        if ((*cube_list)[i]->mass_list[j]->connection != NULL) {
          (*cube_list)[i]->mass_list[j]->vel = velocity;
          (*cube_list)[i]->mass_list[j]->pos = position;

          (*cube_list)[i]->mass_list[j]->connection->vel = velocity;
          (*cube_list)[i]->mass_list[j]->connection->pos = position;
        } else {
          (*cube_list)[i]->mass_list[j]->vel = velocity;
          (*cube_list)[i]->mass_list[j]->pos = position;
        }
      }
    }

    T += dt;
    S++;
  }
  // Get net distance traveled of origin
  Point end = (*cube_list)[0]->mass_list[0]->pos;
  double distance = norm(origin, end);

  myfile.close();

  return distance;
}


double simulate(std::vector<Cube*> *cube_list, double max_time) {
  // Set time variable
  T = 0;

  // Origin center of mass
  Point origin = (*cube_list)[0]->mass_list[0]->pos;
  
  // Create mass tags
  std::vector<Mass *> total_masses;
  for (int i=0; i<(* cube_list).size(); i++) {
    for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
      total_masses.push_back((*cube_list)[i]->mass_list[j]);
    }
  }
  for (int i=0; i<total_masses.size(); i++) {
    for (int j=0; j<total_masses.size(); j++) {
      if (total_masses[i]->pos == total_masses[j]->pos && i != j) {
        total_masses[i]->connection = total_masses[j];
        total_masses[j]->connection = total_masses[i];
      }
    }
  }

  // Copies original positions to reset after simulation
  std::vector<Point> original_pos;
  for (int i=0; i<total_masses.size(); i++) {
    original_pos.push_back(total_masses[i]->pos);
  }

  // Start simulation
  int S = 0;
  while (T < max_time) {
    // Simulate each cube
    //std::cout << T << std::endl;
    for (int i=0; i<(*cube_list).size(); i++) {
      // Set up dictionaries
      std::unordered_map<Mass*, std::vector<Spring>> springMap;
      for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
        std::vector<Spring> springs;
        for (int k=0; k<(*cube_list)[i]->spring_list.size(); k++) {
          if ((*cube_list)[i]->spring_list[k].m1 == (*cube_list)[i]->mass_list[j]) {
            springs.push_back((*cube_list)[i]->spring_list[k]);
          } else if ((*cube_list)[i]->spring_list[k].m2 == (*cube_list)[i]->mass_list[j]) {
            springs.push_back((*cube_list)[i]->spring_list[k]);
          }
        }
        springMap[(*cube_list)[i]->mass_list[j]] = springs;
      }

      // Check for changes in forces
      for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
        Point force(0, 0, 0);
        std::vector<Spring> springs = springMap[(*cube_list)[i]->mass_list[j]];

        // Spring forces
        for (int k=0; k<springs.size(); k++) {
          double length = *(springs[k].L) + (*cube_list)[i]->a + (*cube_list)[i]->b * sin(T * 2 * M_PI * (*cube_list)[i]->c)/10;

          Mass *mass1;
          Mass *mass2;
          if ((*cube_list)[i]->mass_list[j] == springs[k].m1) {
            mass1 = springs[k].m1;
            mass2 = springs[k].m2;
          } else {
            mass1 = springs[k].m2;
            mass2 = springs[k].m1;
          }

          Point unit(0, 0, 0);
          double unit_norm = norm(mass1->pos, mass2->pos);
          unit.x = (mass1->pos.x - mass2->pos.x) / norm(mass1->pos, mass2->pos);
          unit.y = (mass1->pos.y - mass2->pos.y) / norm(mass1->pos, mass2->pos);
          unit.z = (mass1->pos.z - mass2->pos.z) / norm(mass1->pos, mass2->pos);

          Point f_s(0, 0, 0);
          f_s.x = unit.x * (k_f * (norm(mass1->pos, mass2->pos) - length));
          f_s.y = unit.y * (k_f * (norm(mass1->pos, mass2->pos) - length));
          f_s.z = unit.z * (k_f * (norm(mass1->pos, mass2->pos) - length));

          force.x = force.x - f_s.x;
          force.y = force.y - f_s.y;
          force.z = force.z - f_s.z;
        }

        // Gravity forces
        double f_g = (*cube_list)[i]->mass_list[j]->mass * g.z;
        force.z = force.z + f_g;

        // Restoration forces
        double z_pos = (*cube_list)[i]->mass_list[j]->pos.z;
        if (z_pos < 0) {
          force.z = force.z - (k_c * z_pos);
        }

        // Friction forces
        if (z_pos <= 0) {
          double f_h = std::sqrt(std::pow(force.x, 2) + std::pow(force.y, 2));
          double f_n = (*cube_list)[i]->mass_list[j]->mass * g.z;
          if (f_n < 0) {
            if (f_h < -f_n * u_s) {
              force.x = 0;
              force.y = 0;
            } else if (f_h >= -f_n * u_s) {
              force.x = force.x - (u_k * f_n);
              force.y = force.y - (u_k * f_n);
            }
          }
        }

        (*cube_list)[i]->mass_list[j]->forces = force;
      }

      // Update mass pos, acc, vel
      for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
        Point acc(0, 0, 0);
        acc.x = roundDouble(((*cube_list)[i]->mass_list[j]->forces.x / (*cube_list)[i]->mass_list[j]->mass), 3);
        acc.y = roundDouble(((*cube_list)[i]->mass_list[j]->forces.y / (*cube_list)[i]->mass_list[j]->mass), 3);
        acc.z = roundDouble(((*cube_list)[i]->mass_list[j]->forces.z / (*cube_list)[i]->mass_list[j]->mass), 5);

        Point velocity(0, 0, 0);
        velocity.x = roundDouble(((*cube_list)[i]->mass_list[j]->vel.x + (acc.x * dt)) * k_d, 3);
        velocity.y = roundDouble(((*cube_list)[i]->mass_list[j]->vel.y + (acc.y * dt)) * k_d, 3);
        velocity.z = roundDouble(((*cube_list)[i]->mass_list[j]->vel.z + (acc.z * dt)) * k_d, 5);

        Point position(0, 0, 0);
        position.x = roundDouble((*cube_list)[i]->mass_list[j]->pos.x + (velocity.x * dt), 3);
        position.y = roundDouble((*cube_list)[i]->mass_list[j]->pos.y + (velocity.y * dt), 3);
        position.z = roundDouble((*cube_list)[i]->mass_list[j]->pos.z + (velocity.z * dt), 5);

        // Check tags
        if ((*cube_list)[i]->mass_list[j]->connection != NULL) {
          (*cube_list)[i]->mass_list[j]->vel = velocity;
          (*cube_list)[i]->mass_list[j]->pos = position;

          (*cube_list)[i]->mass_list[j]->connection->vel = velocity;
          (*cube_list)[i]->mass_list[j]->connection->pos = position;
        } else {
          (*cube_list)[i]->mass_list[j]->vel = velocity;
          (*cube_list)[i]->mass_list[j]->pos = position;
        }
      }
    }

    T += dt;
    S++;
  }
  // Get net distance traveled of origin
  Point end = (*cube_list)[0]->mass_list[0]->pos;
  double distance = norm(origin, end);

  int k=0;
  for (int i=0; i<cube_list->size(); i++) {
    for (int j=0; j<(*cube_list)[i]->mass_list.size(); j++) {
      (*cube_list)[i]->mass_list[j]->pos = original_pos[k++];
    }
  }

  return distance;
}

double getDouble() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> distr(0, 1);
  return distr(gen);
}

std::vector<Cube*> *createParams(const std::vector<Cube*> *cube_list) {
  std::vector<Cube*> *new_cube_list = new std::vector<Cube*>();
  for (const auto &cube : (*cube_list)) {
    double a = 0;//getDouble();
    double b = getDouble();
    double c = getDouble();
    Cube *tmp_cube = new Cube(cube->mass_list, cube->spring_list, a, b, c);
    new_cube_list->push_back(tmp_cube);
  }
  return new_cube_list;
}

std::vector<Cube*> *copy(const std::vector<Cube*> *cube_list) {
  std::vector<Cube*> *new_cube_list = new std::vector<Cube*>();
  for (const auto &cube : (*cube_list)) {
    Cube *tmp_cube = new Cube(cube->mass_list, cube->spring_list, cube->a, cube->b, cube->c);
    new_cube_list->push_back(tmp_cube);
  }
  return new_cube_list;
}

void mutate(std::vector<Cube*> *individual) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> distr(0, 2);
  std::uniform_int_distribution<int> dis(0, (*individual).size()-1);
  int rn = distr(gen);
  int cube = dis(gen);

  if (rn == 0) {
    (*individual)[cube]->a = 0;//getDouble();
  } else if (rn == 1) {
    (*individual)[cube]->b = getDouble();
  } else {
    (*individual)[cube]->c = getDouble();
  }
}

std::vector<Cube*> *recombine(const std::vector<Cube*> *parent1, const std::vector<Cube*> *parent2) {
  std::vector<Cube*> *child = new std::vector<Cube*>();
  for (int i=0; i<parent1->size(); i++) {
    if (i % 2 == 0) {
      // Parent 1
      child->push_back(new Cube((*parent1)[i]->mass_list, (*parent1)[i]->spring_list, (*parent1)[i]->a, (*parent1)[i]->b, (*parent1)[i]->c));
    } else {
      // Parent 2
      child->push_back(new Cube((*parent2)[i]->mass_list, (*parent2)[i]->spring_list, (*parent2)[i]->a, (*parent2)[i]->b, (*parent2)[i]->c));
    }
  }
  return child;
}

int compareDistance(const void *a, const void *b) {
  std::vector<Cube*> *pt_a = (std::vector<Cube*>*)a;
  std::vector<Cube*> *pt_b = (std::vector<Cube*>*)b;

  double sim_time = 2;
  double distance_a = simulate(pt_a, sim_time);
  double distance_b = simulate(pt_b, sim_time);

  if (distance_a > distance_b) {
    return 1;
  } else if (distance_a < distance_b) {
    return -1;
  } else {
    return 0;
  }

}

void deletePopulation(std::vector<std::vector<Cube*>*> &population) {
  for (auto &cube_list : population) {
    for (auto &cube : *cube_list) {
      delete cube;
    }
    delete cube_list;
  }
}

void deleteIndividual(std::vector<Cube*> *individual) {
  for (auto &cube : *individual) {
    delete cube;
  }
  delete individual;
}

void evolutionaryAlgorithm(std::vector<Cube*> *cube_list, int n_iterations, int pop_size) {
  std::vector<Cube*> *best = nullptr;
  double best_distance = 0;
  int n = 0;

  // Create initial population
  std::vector<std::vector<Cube*>*> population;
  for (int i=0; i<pop_size; i++) {
    std::vector<Cube*> *indiv_cube_list = createParams(cube_list);
    population.push_back(indiv_cube_list);
  }

  std::ofstream myfile;
  myfile.open("distances.txt");

  std::ofstream dot;
  dot.open("dot.txt");

  while (n < n_iterations) {
    std::cout << n << std::endl;
    // Recombine population
    std::vector<std::vector<Cube*>*> recombined_population;
    for (int i=0; i<pop_size; i++) {
      //std::vector<Cube*> *tmp_individual = copy(population[i]);
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> distr(0, pop_size-1);
      int idx1 = distr(gen);
      int idx2 = distr(gen);

      std::vector<Cube*> *parent1 = population[idx1];
      std::vector<Cube*> *parent2 = population[idx2];

      std::vector<Cube*> *child = recombine(parent1, parent2);
      mutate(child);
      recombined_population.push_back(child);
    }

    // Combine population
    std::vector<std::vector<Cube*>*> combined_population;
    for (int j=0; j<pop_size; j++) {
      combined_population.push_back(copy(population[j]));
    }
    int j=0;
    for (int k=pop_size; k<pop_size*2; k++) {
      combined_population.push_back(copy(recombined_population[j++]));
    }

    sort(combined_population.begin(), combined_population.end(), compareDistance);

    std::vector<std::vector<Cube*>*> ranked_population;
    for (int j=0; j<pop_size; j++) {
      ranked_population.push_back(copy(combined_population[j]));
    }
    for (int j=0; j<pop_size; j++) {
      double r = simulate(ranked_population[j], 5);
      dot << r << ", " << n + 1 << "\n";
    }

    double sim_time = 5;
    double distance = simulate(ranked_population[0], sim_time);
    if (distance > best_distance) {
      best_distance = distance;
      if (best == nullptr) {
        best = copy(ranked_population[0]);
      } else {
        deleteIndividual(best);
        best = copy(ranked_population[0]);
      }
    }

    deletePopulation(population);
    population = ranked_population;

    deletePopulation(recombined_population);
    deletePopulation(combined_population);

    std::cout << "best distance: " << best_distance << std::endl;
    myfile << best_distance << "\n";

    n++;
  }
  deletePopulation(population);

  double dist = simulatePrint(best, 5);
  std::cout << "final distance: " << dist << std::endl;
  deleteIndividual(best);

  myfile.close();
  dot.close();
}

void freeSpring(Cube &cube) {
  for (int i=0; i<cube.spring_list.size(); i++) {
    if (cube.spring_list[i].L != nullptr) {
      free(cube.spring_list[i].L);
    }
  }
  cube.spring_list.clear();
}

int main() {
  // Create Initial Points
  Point center1(0, 0, 0);
  Point center2(1, 0, 0);
  Point center3(-1, 0, 0);
  //Point center4(-1, 0, 0);
  //Point center5(1, 0, 0);
  std::vector<Point> initPoints1 = generatePoints(center1);
  std::vector<Point> initPoints2 = generatePoints(center2);
  std::vector<Point> initPoints3 = generatePoints(center3);
  //std::vector<Point> initPoints4 = generatePoints(center4);
  //std::vector<Point> initPoints5 = generatePoints(center5);

  // Initialize Cube
  Cube *cube1 = new Cube();
  Cube *cube2 = new Cube();
  Cube *cube3 = new Cube();
  //Cube *cube4 = new Cube();
  //Cube *cube5 = new Cube();
  
  // Initialize points, mass for Masses in mass_list
  for (int i=0; i<8; i++) {
    (*cube1).mass_list[i] = new Mass(initPoints1[i], 0.1);
    (*cube2).mass_list[i] = new Mass(initPoints2[i], 0.1);
    (*cube3).mass_list[i] = new Mass(initPoints3[i], 0.1);
    //(*cube4).mass_list[i] = new Mass(initPoints4[i], 0.1);
    //(*cube5).mass_list[i] = new Mass(initPoints5[i], 0.1);

    // Adding an external "push"
    //cube1.mass_list[i]->vel.x = 3;
    //cube2.mass_list[i]->vel.x = 3;
  }

  
  // Initialize Springs combinations
  std::vector<std::pair<int, int>> springConnections = generateSpringConnections();
  generateSprings((*cube1), springConnections);
  generateSprings((*cube2), springConnections);
  generateSprings((*cube3), springConnections);
  //generateSprings((*cube4), springConnections);
  //generateSprings((*cube5), springConnections);
  //generateSpringsBetweenCubes(cube1, cube2, springConnections);

  // Connect cubes
  //connectCubes(cube1, cube2);
  //std::cout << (*cube1).mass_list[1]->pos.z << std::endl;

  // Generate cube list
  std::vector<Cube*> cube_list;
  cube_list.push_back(cube1);
  cube_list.push_back(cube2);
  cube_list.push_back(cube3);
  //cube_list.push_back(cube4);
  //cube_list.push_back(cube5);

  // Evolutionary Algorithm
  int n_iterations = 5;
  int pop_size = 8;
  evolutionaryAlgorithm(&cube_list, n_iterations, pop_size);
  //simulatePrint(&cube_list, 5);
  //simulate(&cube_list, 10, 1);

  /*
  // Testing parameters
  cube_list[0].a = 0.1;
  cube_list[0].b = 0.5;
  cube_list[0].c = 0.99;

  cube_list[1].a = 0.1;
  cube_list[1].b = 0;
  cube_list[1].c = 0.98;

  // Simulate
  double max_time = 10;
  simulate(cube_list, max_time);
  */

  // Clean up allocated memory
  freeSpring(*cube1);
  freeSpring(*cube2);
  freeSpring(*cube3);
  //freeSpring(*cube4);
  //freeSpring(*cube5);
  
  for (int i=0; i<cube_list.size(); i++) {
    for (int j=0; j<8; j++) {
      delete cube_list[i]->mass_list[j];
    }
    delete cube_list[i];
  }
  

  return 0;
}
