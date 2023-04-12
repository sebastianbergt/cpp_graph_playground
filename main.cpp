#include <sciplot/sciplot.hpp>
#include <vector>
#include <iostream>

struct Pose
{
    float x;   // in meters
    float y;   // in meters
    float yaw; // in radians
};

struct State
{
    Pose pose;
    float speed; // in meters per second
    float time;  // in seconds
};

struct Node
{
    State state;
    std::vector<Node> nodes;
};

constexpr float YAW_CHANGE{0.1F}; // in radians
constexpr float TIME_DELTA{0.1F}; // in seconds
constexpr std::int32_t NUM_PREDICTED_STATES{10};
constexpr std::int32_t NUM_PREDICTED_STATES_2{NUM_PREDICTED_STATES / 2};

State applyDeltaYaw(State state, const float delta_yaw)
{
    state.pose.yaw += delta_yaw;
    return state;
}

State predictState(const State state, const float delta_time)
{
    State predicted_state{};

    // Calculate the displacement in the x and y directions
    float dx = state.speed * cos(state.pose.yaw) * delta_time;
    float dy = state.speed * sin(state.pose.yaw) * delta_time;

    // Update the position based on the displacement
    predicted_state.pose.x = state.pose.x + dx;
    predicted_state.pose.y = state.pose.y + dy;

    predicted_state.time = state.time + delta_time;

    // Keep the yaw and speed unchanged
    predicted_state.pose.yaw = state.pose.yaw;
    predicted_state.speed = state.speed;

    return predicted_state;
}

std::vector<Node> predictNodes(const Node &node, const float delta_time)
{
    std::vector<Node> nodes{};
    for (std::int32_t i = -NUM_PREDICTED_STATES_2; i < NUM_PREDICTED_STATES_2; ++i)
    {
        Node next{};
        next.state = predictState(applyDeltaYaw(node.state, i * YAW_CHANGE), TIME_DELTA);
        nodes.emplace_back(std::move(next));
    }
    return nodes;
}

std::vector<Node> predictNodesMaxTime(const Node &node, const float delta_time, const float remaining_time)
{
    const auto time_left = remaining_time - delta_time;
    if(time_left < 0.0) {
        return {};
    }
    auto nodes = predictNodes(node, delta_time);
    for(auto& child_node: nodes) {
        child_node.nodes = predictNodesMaxTime(child_node, delta_time, time_left);
    }
    return nodes;
}

void drawSingleNode(const Node node, sciplot::Plot2D &plot)
{
    constexpr float ARROW_LENGTH{0.03F};
    const auto &pose = node.state.pose;
    std::vector<float> x{pose.x, pose.x + std::cos(pose.yaw) * ARROW_LENGTH};
    std::vector<float> y{pose.y, pose.y + std::sin(pose.yaw) * ARROW_LENGTH};
    plot.drawCurve(x, y);
}

void drawNodeRecursively(const Node node, sciplot::Plot2D &plot) {
    drawSingleNode(node, plot);
    for(auto& child_node: node.nodes) {
        drawNodeRecursively(child_node, plot);
    }
}

int main(int argc, char **argv)
{
    Node root{};
    root.state.speed = 0.5;
    root.nodes = predictNodesMaxTime(root, TIME_DELTA, 3*TIME_DELTA);
    std::cout << root.nodes.size();

    using namespace sciplot;
    // Create a vector with values from 0 to pi divived into 200 uniform intervals for the x-axis

    // Create a Plot object
    Plot2D plot;

    // Set the x and y labels
    plot.xlabel("x");
    plot.ylabel("y");

    // Set the x and y ranges
    plot.xrange(-0.05, 0.2);
    plot.yrange(-0.2, 0.2);

    // Set the legend to be on the bottom along the horizontal
    plot.legend().hide();
    //     .atOutsideBottom()
    //     .displayHorizontal()
    //     .displayExpandWidthBy(2);

    drawNodeRecursively(root, plot);

    // Create figure to hold plot
    Figure fig = {{plot}};
    // Create canvas to hold figure
    Canvas canvas = {{fig}};

    // Show the plot in a pop-up window
    canvas.show();

    // Save the plot to a PDF file
    // canvas.save("example-sine-functions.pdf");
}