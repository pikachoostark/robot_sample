#define _USE_MATH_DEFINES

#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <numeric>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

struct Particle {
    double x;
    double y;
    double angle;
    double weight;  // Вес частицы: вероятность того, что координаты частицы совпадают с координатами робота

    Particle(double x, double y, double angle, double weight) : x(x), y(y), angle(angle), weight(weight) {}
};

class ParticleFilter {
private:
    int num_particles;  // Число частиц
    std::vector<Particle> particles;
    std::vector<double> weights;

public:
    ParticleFilter(int num_particles) : num_particles(num_particles) {
        initializeParticles();
    }

    void initializeParticles() {
        // Инициализация частиц в случайных местах
        particles.clear();
        weights.clear();
        std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(0.0, 100.0);
        std::uniform_real_distribution<double> angle_distribution(-M_PI, M_PI);

        for (int i = 0; i < num_particles; ++i) {
            double initial_weight = 1.0 / num_particles;

            particles.emplace_back(distribution(generator), distribution(generator), angle_distribution(generator), initial_weight);
            weights.push_back(initial_weight);
        }
    }

    void predict(double turn_angle, double distance, double control_noise) {
        // Предсказание нового состояния частиц после движения
        std::default_random_engine generator;
        std::normal_distribution<double> control_dist(0, control_noise);

        for (Particle& particle : particles) {
            double noisy_turn_angle = turn_angle + control_dist(generator);
            double noisy_distance = distance + control_dist(generator);
            particle.angle += noisy_turn_angle;

            particle.x -= noisy_distance * sin(particle.angle);
            particle.y += noisy_distance * cos(particle.angle);
        }
    }

    void updateWeights(std::vector<double> measurements, double measurement_noise) {
        // Обновление весов частиц на основе измерений
        weights.clear();
        std::default_random_engine generator;
        std::normal_distribution<double> measurement_dist(0, measurement_noise);

        for (Particle& particle : particles) {
            double prob;

            std::vector<double> measurements_particle = {
                atan2(100 - particle.y, 100 - particle.x) + measurement_dist(generator),
                atan2(100 - particle.y, -particle.x) + measurement_dist(generator),
                atan2(-particle.y, 100 - particle.x) + measurement_dist(generator),
                atan2(-particle.y, -particle.x) + measurement_dist(generator)
            };

            if (particle.x < 0 || particle.x > 100 || particle.y < 0 || particle.y > 100) {
                prob = 0.0;
            }
            else {
                prob = 1.0;
                for (long unsigned int i = 0; i < measurements.size(); ++i) {
                    double error = measurements[i] - measurements_particle[i];
                    prob *= exp(-(error * error) / measurement_noise) / sqrt(2 * M_PI * measurement_noise);
                }
            }

            particle.weight = prob;
            weights.push_back(prob);
        }

        double s = std::accumulate(weights.begin(), weights.end(), 0.0);
        for (int i = 0; i < num_particles; ++i) {
            weights[i] /= s;
            particles[i].weight = weights[i];
        }
    }

    void resample() {
        // Ресемплирование частиц на основе их весов
        std::vector<Particle> resampled_particles;
        std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        double max_weight = *std::max_element(weights.begin(), weights.end());
        double beta = 0.0;
        int index = static_cast<int>(distribution(generator) * num_particles);

        for (int i = 0; i < num_particles; ++i) {
            beta += distribution(generator) * 2.0 * max_weight;
            while (beta > weights[index]) {
                beta -= weights[index];
                index = (index + 1) % num_particles;
            }
            resampled_particles.push_back(particles[index]);
        }

        particles = resampled_particles;

        double s = std::accumulate(weights.begin(), weights.end(), 0.0);
        for (int i = 0; i < num_particles; ++i) {
            weights[i] /= s;
            particles[i].weight = weights[i];
        }
    }

    std::vector<double> estimateState() {
        // Оценка наиболее вероятного состояния робота на основе частиц
        // Можно использовать различные методы для оценки, например, вычисление среднего или максимального веса частиц

        // Пример: оценка как среднее из положений частиц
        double avg_x = 0.0;
        double avg_y = 0.0;

        for (const Particle& particle : particles) {
            avg_x += particle.x * particle.weight;
            avg_y += particle.y * particle.weight;
        }

        return { avg_x, avg_y };
    }
};

class Robot {
private:
    double x; // текущая координата x
    double y; // текущая координата y
    double angle; // текущий угол поворота

    double control_noise; // шум управления
    double measurement_noise; // шум измерения

public:
    Robot(double initial_x, double initial_y, double initial_angle, double control_noise, double measurement_noise) :
        x(initial_x), y(initial_y), angle(initial_angle),
        control_noise(control_noise), measurement_noise(measurement_noise) {}

    void move(double turn_angle, double distance) {
        // Симуляция шума управления
        std::default_random_engine generator;
        std::normal_distribution<double> control_dist(0, control_noise);

        turn_angle += control_dist(generator); // добавление шума к углу поворота
        distance += control_dist(generator);   // добавление шума к движению

        // Пересчет координат после движения
        double new_angle = angle + turn_angle;

        double new_x = x - distance * sin(new_angle);
        double new_y = y + distance * cos(new_angle);

        // Проверка на выход за границы
        if (new_x >= 0 && new_x <= 100 && new_y >= 0 && new_y <= 100) {
            x = new_x;
            y = new_y;
            angle = new_angle;
        }
        else {
            // Обработка выхода за границы 
            std::cout << "Robot can't move outside the grid!" << std::endl;
        }
    }

    std::vector<double> sense() {
        // Симуляция шума измерения
        std::default_random_engine generator;
        std::normal_distribution<double> measurement_dist(0, measurement_noise);

        std::vector<double> measurements = {
            atan2(100 - y, 100 - x) + measurement_dist(generator),
            atan2(100 - y, -x) + measurement_dist(generator),
            atan2(-y, 100 - x) + measurement_dist(generator),
            atan2(-y, -x) + measurement_dist(generator)
        };

        return measurements;
    }

    std::vector<double> measurement_prob(ParticleFilter pf) {
        return pf.estimateState();
    }

    double get_x() {
        return this->x;
    }

    double get_y() {
        return this->y;
    }
};

int main(int argc, char* argv[]) {
    std::vector< std::vector<double> > move = {
        {0.0, 10},
        {M_PI / 2, 10},
        {M_PI / 2, 10},
        {0.0, 10},
        {M_PI / 2, 10},
        {0.0, 10},
        {M_PI / 2, 10},
        {0.0, 10},
        {0.0, 10},
        {M_PI / 2, 10}
    };

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher_node");

    auto truth_publisher = node->create_publisher<std_msgs::msg::String>("/ground_truth", 10);
    auto position_publisher = node->create_publisher<std_msgs::msg::String>("/position", 10);

    auto truth_msg = std_msgs::msg::String();
    auto position_msg = std_msgs::msg::String();

    Robot robot(50, 50, 0, 0.1, 0.1);
    ParticleFilter pf(100); // Создание фильтра

    std::string initial_state = "Initial state: x = " + std::to_string(robot.get_x()) + ", y = " + std::to_string(robot.get_y());
    std::cout << initial_state << std::endl;
    truth_msg.data = initial_state;
    truth_publisher->publish(truth_msg);

    std::vector<double> measurements = robot.sense(); // Измерения пеленга
    pf.updateWeights(measurements, 0.1); // Обновление весов
    pf.resample(); // Ресемплирование
    pf.estimateState(); // Оценка состояния

    std::string true_state, estimated_state;
    std::vector<double> estimate;
    for (long unsigned int i = 0; i < move.size(); i++) {

        // Uncomment this for listen topic manually:
        // std::this_thread::sleep_for(5s);
        std::cout << "Movement " << i << std::endl;

        robot.move(move[i][0], move[i][1]);
        std::vector<double> measurements = robot.sense(); // Измерения пеленга

        true_state = "True state: x = " + std::to_string(robot.get_x()) + ", y = " + std::to_string(robot.get_y());
        std::cout << true_state << std::endl;
        truth_msg.data = true_state;
        truth_publisher->publish(truth_msg);

        pf.predict(move[i][0], move[i][1], 0.1);
        pf.updateWeights(measurements, 0.1); // Обновление весов
        pf.resample(); // Ресемплирование
        estimate = pf.estimateState(); // Оценка состояния

        estimated_state = "Estimated state: x = " + std::to_string(estimate[0]) + ", y = " + std::to_string(estimate[1]);
        std::cout << estimated_state << std::endl;
        position_msg.data = estimated_state;
        position_publisher->publish(position_msg);
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}