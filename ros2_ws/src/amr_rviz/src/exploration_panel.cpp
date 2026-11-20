#include <atomic>
#include <memory>

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <explore_lite_msgs/msg/explore_status.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/bool.hpp>

namespace amr_rviz
{
class ExplorationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ExplorationPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent),
    status_(new QLabel(this)),
    explore_button_(new QPushButton(tr("Explore"), this)),
    goals_button_(new QPushButton(tr("RViz Goals"), this))
  {
    auto * buttons = new QHBoxLayout;
    buttons->addWidget(explore_button_);
    buttons->addWidget(goals_button_);
    auto * layout = new QVBoxLayout(this);
    layout->addLayout(buttons);
    layout->addWidget(status_);
    status_->setWordWrap(true);

    connect(explore_button_, &QPushButton::clicked, this, [this]() {requestMode(true);});
    connect(goals_button_, &QPushButton::clicked, this, [this]() {requestMode(false);});
    refresh();
  }

  void onInitialize() override
  {
    const auto rviz_node = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!rviz_node) {
      return;
    }
    const auto node = rviz_node->get_raw_node();
    command_ = node->create_publisher<std_msgs::msg::Bool>("/explore/resume", 10);
    report_ = node->create_subscription<ExploreStatus>(
      "/explore/status", rclcpp::QoS(1).transient_local(),
      [mode = mode_](ExploreStatus::ConstSharedPtr message) {
        const auto & status = message->status;
        if (status == ExploreStatus::EXPLORATION_PAUSED ||
          status == ExploreStatus::EXPLORATION_COMPLETE ||
          status == ExploreStatus::RETURNED_TO_ORIGIN)
        {
          mode->store(Mode::Goals);
        } else if (status == ExploreStatus::EXPLORATION_STARTED ||
          status == ExploreStatus::EXPLORATION_IN_PROGRESS ||
          status == ExploreStatus::RETURNING_TO_ORIGIN)
        {
          mode->store(Mode::Explore);
        } else {
          mode->store(Mode::Unknown);
        }
      });

    // Read discovery and status from the GUI thread before touching Qt widgets.
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {refresh();});
    timer->start(250);
  }

private:
  using ExploreStatus = explore_lite_msgs::msg::ExploreStatus;
  enum class Mode { Unknown, Explore, Goals };

  void requestMode(bool explore)
  {
    if (command_ && command_->get_subscription_count() > 0) {
      std_msgs::msg::Bool command;
      command.data = explore;
      command_->publish(command);
    }
  }

  void refresh()
  {
    const bool connected = command_ && report_ && command_->get_subscription_count() > 0 &&
      report_->get_publisher_count() > 0;
    if (!connected) {
      status_->setText(tr("Waiting for explorer"));
      explore_button_->setEnabled(false);
      goals_button_->setEnabled(false);
      return;
    }

    const auto mode = mode_->load();
    explore_button_->setEnabled(mode != Mode::Explore);
    goals_button_->setEnabled(mode != Mode::Goals);
    if (mode == Mode::Explore) {
      status_->setText(tr("Exploring"));
    } else if (mode == Mode::Goals) {
      status_->setText(tr("Explorer paused. Use Nav2 Goal to pick a destination."));
    } else {
      status_->setText(tr("Waiting for exploration status"));
    }
  }

  QLabel * status_;
  QPushButton * explore_button_;
  QPushButton * goals_button_;
  std::shared_ptr<std::atomic<Mode>> mode_ = std::make_shared<std::atomic<Mode>>(Mode::Unknown);
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr command_;
  rclcpp::Subscription<ExploreStatus>::SharedPtr report_;
};
}  // namespace amr_rviz

PLUGINLIB_EXPORT_CLASS(amr_rviz::ExplorationPanel, rviz_common::Panel)

#include "exploration_panel.moc"
