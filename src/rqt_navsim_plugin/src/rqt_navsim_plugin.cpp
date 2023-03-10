#include <rqt_navsim_plugin/rqt_navsim_plugin.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QStringList>

using std::placeholders::_1;
using std::placeholders::_2;

namespace rqt_plugin
{
NavsimLaunchGUI::NavsimLaunchGUI()
:   rqt_gui_cpp::Plugin()
,   widget_(0)
{
}

void NavsimLaunchGUI::initPlugin(qt_gui_cpp::PluginContext& context)
{
    QStringList argv = context.argv();

    widget_ = new QWidget();
    ui_.setupUi(widget_);
    context.addWidget(widget_);

    connect(ui_.load_sim_button, SIGNAL(pressed()), this, SLOT(load_button_callback()));
    connect(ui_.start_sim_button, SIGNAL(pressed()), this, SLOT(start_button_callback()));
}

void NavsimLaunchGUI::shutdownPlugin()
{
    ;
}

void NavsimLaunchGUI::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    ;
}

void NavsimLaunchGUI::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    ;
}

NavsimLaunchGUI::~NavsimLaunchGUI()
{
    if(widget_)
        delete widget_;
}

void NavsimLaunchGUI::load_button_callback(){

    std::string file_path = ui_.sim_file_loader->getOpenFileName().toStdString();

    goal_.path = file_path;
}

void NavsimLaunchGUI::start_button_callback(){
    if(goal_.path.empty()){
        return;
    }

    std::thread thread(std::bind(&NavsimLaunchGUI::send_goal, this));

    thread.detach();
}

void NavsimLaunchGUI::send_goal(){
    if(!client_ptr_){
        client_ptr_ = rclcpp_action::create_client<LoadSimulation>(node_, "load_simulation");
    }

    if(!client_ptr_->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        return;
    }


    auto send_goal_options = rclcpp_action::Client<LoadSimulation>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(&NavsimLaunchGUI::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&NavsimLaunchGUI::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&NavsimLaunchGUI::result_callback, this, _1);

    client_ptr_->async_send_goal(goal_, send_goal_options);
}

void NavsimLaunchGUI::goal_response_callback(std::shared_future<GoalHandleLoadSimulation::SharedPtr> future){}

void NavsimLaunchGUI::feedback_callback(GoalHandleLoadSimulation::SharedPtr, 
    const std::shared_ptr<const LoadSimulation::Feedback> feedback){}

void NavsimLaunchGUI::result_callback(const GoalHandleLoadSimulation::WrappedResult & result){}

} // namespace rqt_plugin

PLUGINLIB_EXPORT_CLASS(rqt_plugin::NavsimLaunchGUI, rqt_gui_cpp::Plugin)
