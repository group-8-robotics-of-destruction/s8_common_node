/**
 * The Node class is meant to serve as a base class for nodes. It will create a private namespaces node handle
 * and will provide a convinient interface for handling ros params.
 */

#ifndef __S8_NODE_H
#define __S8_NODE_H

#include <vector>
#include <ros/ros.h>
#include <boost/variant.hpp>

#define PRINT_PADDING       5

namespace s8 {
    class Node {
        struct Param {
            std::string name;

            enum Type {
                BOOL,
                INT,
                DOUBLE,
                STRING
            };

            boost::variant<bool, int, double, std::string> default_value;
            boost::variant<bool*, int*, double*, std::string*> destination;

            template<typename T>
            Param(const std::string & name, const T & default_value, T * destination) {
                this->name = name;
                this->default_value = default_value;
                this->destination = destination;
            }
        };

    private:
        std::vector<Param> params;
        int longest_name_length;
        const int print_padding;

    protected:
        ros::NodeHandle nh;

    public:
        Node(int print_padding = PRINT_PADDING) : nh("~"), longest_name_length(0), print_padding(print_padding) {}

        /**
         * This functions register a static ros paramater. The node will try to fetch the value from the rosparam server.
         * If the value doesn't exist in the param server, the default value will be sent to the param server and set to the
         * local parameter. The read/write will only occur once (i.e. when this function is called).
         */
        template<typename T>
        void add_param(const std::string & name, T & destination, const T & default_value) {
            params.push_back(Param(name, default_value, &destination));
            init_param(name, destination, default_value);

            if(name.length() > longest_name_length) {
                longest_name_length = name.length();
            }
        }

        //TODO: update_param.

        /**
         * Prints all registered params this node is operating on. Will also print the current values of the params.
         */
        void print_params() {
            ROS_INFO("--Params--");

            std::string padding;

            auto bool_to_str = [](bool value) {
                return value ? "true" : "false";
            };

            for(auto param : params) {
                int missing = longest_name_length + print_padding - param.name.length();
                padding.assign(missing, ' ');

                switch(Param::Type(param.default_value.which())) {
                case Param::Type::BOOL:
                    ROS_INFO("%s:%s%s (%s)", param.name.c_str(), padding.c_str(), bool_to_str(*boost::get<bool*>(param.destination)), bool_to_str(boost::get<bool>(param.default_value)));
                    break;
                case Param::Type::INT:
                    ROS_INFO("%s:%s%d (%d)", param.name.c_str(), padding.c_str(), *boost::get<int*>(param.destination), boost::get<int>(param.default_value));
                    break;
                case Param::Type::DOUBLE:
                    ROS_INFO("%s:%s%lf (%lf)", param.name.c_str(), padding.c_str(), *boost::get<double*>(param.destination), boost::get<double>(param.default_value));
                    break;
                case Param::Type::STRING:
                    ROS_INFO("%s:%s%s (%s)", param.name.c_str(), padding.c_str(), (*boost::get<std::string*>(param.destination)).c_str(), boost::get<std::string>(param.default_value).c_str());
                    break;
                }
            }

            ROS_INFO("");
        }

    private:
        template<class T>
        void init_param(const std::string & name, T & destination, const T & default_value) {
            if(!nh.hasParam(name)) {
                nh.setParam(name, default_value);
            }

            if(!nh.getParam(name, destination)) {
                ROS_WARN("Failed to get parameter %s from param server. Falling back to default value.", name.c_str());
                destination = default_value;
            }
        }
    };
};

#endif