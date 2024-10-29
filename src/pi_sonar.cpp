/*
 * Copyright (c) 2018, Ubiquity Robotics
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * ...
 * 
 * (Copyright notice and license terms)
 * 
 */

#if defined(__arm__) || defined(__aarch64__)

#include <stdio.h>
#include <unistd.h>
#include <pigpiod_if2.h>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"

double min_freq = 0.5;
double max_freq = 60;

int gpio = -1;
pthread_t* sonarthread;

class Sonar {
public:
    // BCM GPIO pins
    int trigger_pin;
    int echo_pin;
    int id;

    uint32_t start_tick;
    uint32_t elapsed_ticks;

    std::string frame;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub;

    bool range_error = false;

    std::shared_ptr<diagnostic_updater::Updater> updater;
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> pub_freq;

    Sonar(int trigger_pin, int echo_pin, int id, rclcpp::Node::SharedPtr node)
    : trigger_pin(trigger_pin), echo_pin(echo_pin), id(id)
    {
        start_tick = 0;
        elapsed_ticks = 0;

        frame = "pi_sonar/sonar_" + std::to_string(id);
        pub = node->create_publisher<sensor_msgs::msg::Range>(frame, rclcpp::QoS(1));

        updater = std::make_shared<diagnostic_updater::Updater>(
            node->get_node_base_interface(),
            node->get_node_clock_interface(),
            node->get_node_logging_interface(),
            node->get_node_parameters_interface(),
            node->get_node_timers_interface(),
            node->get_node_topics_interface()
        );
        updater->setHardwareIDf("sonar_%d", id);
        pub_freq = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
            "pi_sonar/sonar_" + std::to_string(id), 
            *updater,
            diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10)
        );
        updater->add("Sonar " + std::to_string(id) + " Range Checker",
            this,
            &Sonar::range_check
        );
    }

    void range_check(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (range_error) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Range out of bounds!");
        }
        else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Range within bounds!");
        }
    }
};

static std::vector<Sonar> sonars;

/* Trigger the next sonar */
void sonar_trigger()
{
    static int sonar = 0;

    int pin = sonars[sonar].trigger_pin;
    gpio_write(gpio, pin, PI_ON);

    int waittime = get_current_tick(gpio);
    while(get_current_tick(gpio) - waittime < 10){
        /* wait for 10us trigger pulse */
    }

    gpio_write(gpio, pin, PI_OFF);

    sonar++;
    if (static_cast<size_t>(sonar) >= sonars.size()) {  // Fix signed-unsigned comparison warning
        sonar = 0;
    }
}

/* Sonar pulsing thread */ 
void* sonar_thread([[maybe_unused]] void* data) 
{
    /* every 50ms, with probably garbage accuracy */
    while (1){
        time_sleep(0.05);
        sonar_trigger();
    }
    return NULL;
}

/* Handle pin change */
void echo_callback([[maybe_unused]] int pigpio, uint32_t pin, uint32_t level, uint32_t tick)
{
    for (auto& sonar : sonars) {
        if (pin == static_cast<uint32_t>(sonar.echo_pin)) {  // Fix signed-unsigned comparison warning
            if (level == PI_ON) {
                sonar.start_tick = tick;
            }
            else if (level == PI_OFF) {
                uint32_t elapsed = tick - sonar.start_tick;
                sonar.elapsed_ticks = elapsed;
            }
            return;
         }
     }
     printf("Unexpected GPIO pin event, pin %d\n", pin);
}

int setup_gpio()
{
    gpio = pigpio_start(NULL, NULL);
    if (gpio < 0) {
        return false;
    }

    for (const auto& sonar : sonars) {
        set_mode(gpio, sonar.trigger_pin, PI_OUTPUT);
        gpio_write(gpio, sonar.trigger_pin, PI_OFF);

        set_mode(gpio, sonar.echo_pin, PI_INPUT);

        /* monitor sonar echos */
        callback(gpio, sonar.echo_pin, EITHER_EDGE, echo_callback);
    }

    /* update sonar 20 times a second, timer #0 */
    void* extra_param_because_reasons = 0;
    sonarthread = start_thread(sonar_thread, extra_param_because_reasons);

    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pi_sonar");

    double field_of_view;
    double min_range;
    double max_range;

    node->declare_parameter("field_of_view", 0.43632347);
    node->declare_parameter("min_range", 0.05);
    node->declare_parameter("max_range", 10.0);

    node->get_parameter("field_of_view", field_of_view);
    node->get_parameter("min_range", min_range);
    node->get_parameter("max_range", max_range);

    auto pub = node->create_publisher<sensor_msgs::msg::Range>("/sonars", 5);

    // pin numbers are specific to the hardware
    sonars.push_back(Sonar(20, 21, 0, node));
    sonars.push_back(Sonar(12, 16, 1, node));
    sonars.push_back(Sonar(23, 24, 2, node));
    sonars.push_back(Sonar(27, 22, 3, node));
    sonars.push_back(Sonar(19, 26, 4, node));

    if (!setup_gpio()) {
        RCLCPP_ERROR(node->get_logger(), "Cannot initialize GPIO");
        return 1;
    }

    rclcpp::Rate rate(50);
    sensor_msgs::msg::Range msg;
    msg.field_of_view = field_of_view;
    msg.min_range = min_range;
    msg.max_range = max_range;
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;

    RCLCPP_INFO(node->get_logger(), "Pi Sonar node ready");

    while (rclcpp::ok()) {
        rclcpp::spin_some(node); // Process any incoming messages

        for (auto& sonar: sonars) {
            uint32_t elapsed_ticks = sonar.elapsed_ticks;
            if (elapsed_ticks != 0) {
                sonar.elapsed_ticks = 0;
                msg.range = (float)elapsed_ticks * 0.0001715;
                sonar.range_error = (msg.range < min_range || msg.range > max_range);
                msg.header.stamp = rclcpp::Time();
                msg.header.frame_id = sonar.frame;
                pub->publish(msg);
                sonar.pub->publish(msg);
                sonar.pub_freq->tick();
            }
            sonar.updater->force_update();
        }
        rate.sleep();
    }

    stop_thread(sonarthread);
    pigpio_stop(gpio);

    rclcpp::shutdown();
    return 0;
}

#else

#include <stdio.h>

int main(int argc, char **argv) {
    fprintf(stderr, "pi_sonar only works on the Raspberry Pi\n");
    return 1;
}

#endif // __arm__