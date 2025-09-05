// src/finger_mapper/src/finger_mapper_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_value.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <inspire_interfaces/msg/set_angle1.hpp>

#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>

using json = nlohmann::json;

// ---------- 数据结构 ----------
struct Coeff {
    std::unordered_map<std::string, int> exponents; // var -> power
    double coefficient = 0.0;
};

struct PolyModel {
    std::vector<std::string> joint_order; 
    std::vector<Coeff> coeffs;
};

// ---------- 节点 ----------
class FingerMapperNode : public rclcpp::Node {
public:
    FingerMapperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("finger_mapper", options)
    {
        // JSON 配置路径
        std::string pkg_share;
        try {
            pkg_share = ament_index_cpp::get_package_share_directory("finger_mapper");
        } catch (const std::exception &e) {
            pkg_share = ".";
        }
        std::string default_path = pkg_share + "/config/finger_config.json";

        this->declare_parameter("config_file", rclcpp::ParameterValue(default_path));
        std::string json_path;
        this->get_parameter("config_file", json_path);

        loadJson(json_path);

        // 订阅 joint_states
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&FingerMapperNode::jointCallback, this, std::placeholders::_1)
        );

        // 发布 /set_angle_data
        pub_ = this->create_publisher<inspire_interfaces::msg::SetAngle1>(
            "/set_angle_data", 10
        );
    }

private:
    void loadJson(const std::string &path) {
        std::ifstream ifs(path);
        if (!ifs.is_open()) return;

        json j;
        try { ifs >> j; } catch (...) { return; }

        if (j.contains("joint_map")) {
            for (auto &el : j["joint_map"].items()) {
                const std::string joint_name = el.key();
                const json &info = el.value();
                if (info.contains("finger") && info.contains("joint_idx")) {
                    std::string finger = info["finger"].get<std::string>();
                    std::string joint_type = info["joint_idx"].get<std::string>();
                    joint_map_[joint_name] = {finger, joint_type};
                }
            }
        }

        if (j.contains("poly_models")) {
            for (auto &el : j["poly_models"].items()) {
                const std::string finger = el.key();
                const json &model = el.value();
                PolyModel pm;

                if (model.contains("joint_order") && model["joint_order"].is_array()) {
                    for (const auto &s : model["joint_order"])
                        pm.joint_order.push_back(s.get<std::string>());
                }

                if (model.contains("coeffs") && model["coeffs"].is_array()) {
                    for (const auto &c : model["coeffs"]) {
                        Coeff cf;
                        for (const auto &var : pm.joint_order)
                            cf.exponents[var] = c.contains(var) ? c[var].get<int>() : 0;
                        cf.coefficient = c.contains("coefficient") ? c["coefficient"].get<double>() : 0.0;
                        pm.coeffs.push_back(cf);
                    }
                }

                poly_models_[finger] = pm;
            }
        }
    }

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::unordered_map<std::string, std::unordered_map<std::string, double>> finger_inputs;

        size_t N = std::min(msg->name.size(), msg->position.size());
        for (size_t i = 0; i < N; ++i) {
            const std::string &jname = msg->name[i];
            double pos = msg->position[i];

            auto it = joint_map_.find(jname);
            if (it != joint_map_.end()) {
                finger_inputs[it->second.first][it->second.second] = pos;
            }
        }

        // 计算输出值
        std::vector<std::string> target_fingers = {"index","middle","ring","little","thumb_1","thumb_2"};
        std::unordered_map<std::string,double> outputs_map;
        for (const auto &f : target_fingers) {
            if (poly_models_.find(f) == poly_models_.end()) {
                outputs_map[f] = 0.0;
                continue;
            }
            double y = evaluatePoly(poly_models_[f], finger_inputs[f]);
            outputs_map[f] = clamp(y, 0.0, 1000.0); // 限制范围
        }

        // 按 little,ring,middle,index,thumb_1,thumb_2 顺序发布
        inspire_interfaces::msg::SetAngle1 msg_out;
        msg_out.finger_ids = {1,2,3,4,5,6};
        msg_out.angles = {
            static_cast<int>(outputs_map["little"]),
            static_cast<int>(outputs_map["ring"]),
            static_cast<int>(outputs_map["middle"]),
            static_cast<int>(outputs_map["index"]),
            static_cast<int>(outputs_map["thumb_1"]),
            static_cast<int>(outputs_map["thumb_2"])
        };

        pub_->publish(msg_out);
    }

    double evaluatePoly(const PolyModel &model,
                        const std::unordered_map<std::string,double> &inputs) {
        double sum = 0.0;
        for (const auto &term : model.coeffs) {
            double prod = term.coefficient;
            bool zero_term = false;
            for (const auto &kv : term.exponents) {
                const std::string &var = kv.first;
                int exp = kv.second;
                auto it = inputs.find(var);
                if (it == inputs.end()) {
                    if (exp != 0) { zero_term = true; break; }
                } else {
                    prod *= std::pow(it->second, exp);
                }
            }
            if (!zero_term) sum += prod;
        }
        return sum;
    }

    double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }

    // members
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<inspire_interfaces::msg::SetAngle1>::SharedPtr pub_;
    std::unordered_map<std::string, std::pair<std::string,std::string>> joint_map_;
    std::unordered_map<std::string, PolyModel> poly_models_;
};

// ---------- main ----------
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FingerMapperNode>());
    rclcpp::shutdown();
    return 0;
}
