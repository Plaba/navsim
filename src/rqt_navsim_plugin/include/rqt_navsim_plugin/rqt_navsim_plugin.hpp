#ifndef RQT_NAVSIM_PLUGIN_HPP
#define RQT_NAVSIM_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <simulation_loader_msgs/action/load_simulation.hpp>

#include <rqt_gui_cpp/plugin.h>
#include <rqt_navsim_plugin/ui_rqt_navsim_plugin.h>
#include <QWidget>

namespace rqt_plugin
{
  using LoadSimulation=simulation_loader_msgs::action::LoadSimulation;
  using GoalHandleLoadSimulation=rclcpp_action::ClientGoalHandle<LoadSimulation>;

  class NavsimLaunchGUI : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
    public:
      NavsimLaunchGUI();

      virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;

      virtual void shutdownPlugin() override;

      virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;

      virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

      virtual ~NavsimLaunchGUI();

      private:
        Ui::gui ui_;
        QWidget * widget_;

        rclcpp_action::Client<LoadSimulation>::SharedPtr client_ptr_;

        void send_goal();

        void goal_response_callback(std::shared_future<GoalHandleLoadSimulation::SharedPtr> future);

        void feedback_callback(GoalHandleLoadSimulation::SharedPtr goal_handle, 
          const std::shared_ptr<const LoadSimulation::Feedback> feedback);

        void result_callback(const GoalHandleLoadSimulation::WrappedResult & result);

      protected:
        
        LoadSimulation::Goal goal_;
      
      protected slots:
        void load_button_callback();
        void start_button_callback();
  };
}  // namespace rqt_plugin

#endif  // RQT_NAVSIM_PLUGIN_HPP
