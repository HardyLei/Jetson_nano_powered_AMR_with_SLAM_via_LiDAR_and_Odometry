#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPushButton>
#include <QSet>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>

#include <amr_interfaces/msg/drive_mode.hpp>
#include <amr_interfaces/msg/manual_drive.hpp>
#include <amr_interfaces/srv/set_manual_drive.hpp>
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
    goals_button_(new QPushButton(tr("RViz Goals"), this)),
    manual_button_(new QPushButton(tr("Manual Drive"), this)),
    left_button_(new QPushButton(tr("Rotate Left"), this)),
    right_button_(new QPushButton(tr("Rotate Right"), this))
  {
    auto * buttons = new QHBoxLayout;
    buttons->addWidget(explore_button_);
    buttons->addWidget(goals_button_);
    buttons->addWidget(manual_button_);
    auto * turn_buttons = new QHBoxLayout;
    turn_buttons->addWidget(left_button_);
    turn_buttons->addWidget(right_button_);
    auto * layout = new QVBoxLayout(this);
    layout->addLayout(buttons);
    layout->addLayout(turn_buttons);
    layout->addWidget(status_);
    status_->setWordWrap(true);
    setFocusPolicy(Qt::StrongFocus);
    left_button_->setFocusPolicy(Qt::NoFocus);
    right_button_->setFocusPolicy(Qt::NoFocus);

    connect(explore_button_, &QPushButton::clicked, this, [this]() {requestMode(Target::Explore);});
    connect(goals_button_, &QPushButton::clicked, this, [this]() {requestMode(Target::Goals);});
    connect(manual_button_, &QPushButton::clicked, this, [this]() {requestMode(Target::Manual);});
    connect(left_button_, &QPushButton::pressed, this, [this]() {rotate(1);});
    connect(right_button_, &QPushButton::pressed, this, [this]() {rotate(-1);});
    connect(left_button_, &QPushButton::released, this, [this]() {rotate(0);});
    connect(right_button_, &QPushButton::released, this, [this]() {rotate(0);});
    refresh();
  }

  void onInitialize() override
  {
    const auto rviz_node = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!rviz_node) {
      return;
    }
    const auto node = rviz_node->get_raw_node();
    mode_client_ = node->create_client<SetManualDrive>("/amr/set_manual_drive");
    auto manual_qos = rclcpp::QoS(1).best_effort();
    manual_qos.lifespan(rclcpp::Duration::from_seconds(0.2));
    manual_pub_ = node->create_publisher<ManualDrive>("/amr/manual_drive", manual_qos);
    drive_report_ = node->create_subscription<DriveMode>(
      "/amr/drive_mode", rclcpp::QoS(1).transient_local(),
      [feedback = drive_feedback_](DriveMode::ConstSharedPtr message) {
        const std::lock_guard<std::mutex> lock(feedback->mutex);
        feedback->mode = *message;
      });
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
    timer->start(50);
  }

protected:
  void keyPressEvent(QKeyEvent * event) override
  {
    if (manual_granted_ && driveKey(event->key())) {
      if (!event->isAutoRepeat()) {
        keys_.insert(event->key());
        publishManual();
      }
      event->accept();
    } else {
      rviz_common::Panel::keyPressEvent(event);
    }
  }

  void keyReleaseEvent(QKeyEvent * event) override
  {
    if (manual_granted_ && driveKey(event->key())) {
      if (!event->isAutoRepeat()) {
        keys_.remove(event->key());
        publishManual();
      }
      event->accept();
    } else {
      rviz_common::Panel::keyReleaseEvent(event);
    }
  }

  void focusOutEvent(QFocusEvent * event) override
  {
    clearMotion();
    rviz_common::Panel::focusOutEvent(event);
  }

  void hideEvent(QHideEvent * event) override
  {
    clearMotion();
    rviz_common::Panel::hideEvent(event);
  }

private:
  using ExploreStatus = explore_lite_msgs::msg::ExploreStatus;
  using DriveMode = amr_interfaces::msg::DriveMode;
  using ManualDrive = amr_interfaces::msg::ManualDrive;
  using SetManualDrive = amr_interfaces::srv::SetManualDrive;
  enum class Mode { Unknown, Explore, Goals };
  enum class Target { Manual, Explore, Goals };
  struct DriveFeedback
  {
    std::mutex mutex;
    std::optional<DriveMode> mode;
  };

  void requestMode(Target target)
  {
    if (pending_ || !mode_client_ || !mode_client_->service_is_ready()) {
      return;
    }
    clearMotion();
    notice_.clear();
    if (target == Target::Manual || session_.empty()) {
      session_ = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    }
    auto request = std::make_shared<SetManualDrive::Request>();
    request->session_id = session_;
    request->enable = target == Target::Manual;
    requested_mode_ = target;
    request_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    pending_.emplace(mode_client_->async_send_request(request));
    refresh();
  }

  void finishModeRequest()
  {
    if (pending_->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() >= request_deadline_) {
        mode_client_->remove_pending_request(pending_->request_id);
        pending_.reset();
        notice_ = tr("The Jetson did not confirm the mode change.");
      }
      return;
    }
    const auto reply = pending_->get();
    pending_.reset();
    if (!reply->success) {
      notice_ = QString::fromStdString(reply->message);
      return;
    }
    if (requested_mode_ == Target::Manual) {
      setFocus(Qt::OtherFocusReason);
      return;
    }
    session_.clear();
    manual_granted_ = false;
    publishExplorerMode(requested_mode_ == Target::Explore);
  }

  void publishExplorerMode(bool explore)
  {
    if (command_ && command_->get_subscription_count() > 0) {
      std_msgs::msg::Bool command;
      command.data = explore;
      command_->publish(command);
    }
  }

  static bool driveKey(int key)
  {
    return key == Qt::Key_Space || key == Qt::Key_Shift ||
           key == Qt::Key_Up || key == Qt::Key_Down ||
           key == Qt::Key_Left || key == Qt::Key_Right;
  }

  void rotate(int direction)
  {
    if (manual_granted_ && !pending_) {
      if (direction != 0) {
        setFocus(Qt::MouseFocusReason);
        keys_.clear();
      }
      rotation_ = direction;
      publishManual();
    }
  }

  void clearMotion()
  {
    keys_.clear();
    rotation_ = 0;
    publishManual();
  }

  void publishManual()
  {
    if (!manual_granted_) {
      return;
    }
    constexpr double drive_speed = 0.07;
    constexpr double turn_speed = 0.30;
    ManualDrive command;
    command.session_id = session_;
    if (!pending_ && hasFocus() && isVisible() && window()->isActiveWindow()) {
      if (rotation_ != 0) {
        command.velocity.angular.z = turn_speed * rotation_;
      } else if (keys_.contains(Qt::Key_Space)) {
        const int forward = keys_.contains(Qt::Key_Up) - keys_.contains(Qt::Key_Down);
        const int left = keys_.contains(Qt::Key_Left) - keys_.contains(Qt::Key_Right);
        command.velocity.linear.x = drive_speed * forward;
        if (keys_.contains(Qt::Key_Shift)) {
          command.velocity.linear.y = drive_speed * left;
        } else {
          command.velocity.angular.z = turn_speed * left;
        }
      }
    } else {
      keys_.clear();
      rotation_ = 0;
    }
    manual_pub_->publish(command);
  }

  void refresh()
  {
    if (pending_) {
      finishModeRequest();
    }
    std::optional<DriveMode> drive_mode;
    {
      const std::lock_guard<std::mutex> lock(drive_feedback_->mutex);
      drive_mode = drive_feedback_->mode;
    }
    const bool drive_connected = mode_client_ && drive_report_ && drive_mode &&
      drive_report_->get_publisher_count() > 0 && mode_client_->service_is_ready();
    const bool explorer_connected = command_ && report_ && command_->get_subscription_count() > 0 &&
      report_->get_publisher_count() > 0;
    const bool owned = drive_connected && !session_.empty() &&
      drive_mode->manual_session == session_;
    if (manual_granted_ && !owned) {
      clearMotion();
    }
    manual_granted_ = owned;
    const bool other_owner = drive_connected && !drive_mode->manual_session.empty() && !owned;
    const bool navigation = drive_connected && drive_mode->navigation_enabled;
    const bool can_switch = drive_connected && !other_owner && !pending_;
    const auto mode = mode_->load();
    manual_button_->setEnabled(can_switch && !owned);
    explore_button_->setEnabled(can_switch && explorer_connected &&
      (!navigation || mode != Mode::Explore));
    goals_button_->setEnabled(can_switch &&
      (!navigation || (explorer_connected && mode != Mode::Goals)));
    left_button_->setEnabled(owned && !pending_);
    right_button_->setEnabled(owned && !pending_);
    if (pending_) {
      status_->setText(tr("Switching drive mode…"));
    } else if (!notice_.isEmpty()) {
      status_->setText(notice_);
    } else if (!drive_connected) {
      status_->setText(tr("Waiting for the Jetson drive controller"));
    } else if (owned) {
      status_->setText(tr("Manual drive. Hold Space and an arrow, or hold a Rotate button."));
    } else if (other_owner) {
      status_->setText(tr("Another RViz window has manual control."));
    } else if (!navigation) {
      status_->setText(tr("Drive paused. Choose Manual Drive, RViz Goals, or Explore."));
    } else if (!explorer_connected) {
      status_->setText(tr("Use Nav2 Goal to pick a destination."));
    } else if (mode == Mode::Explore) {
      status_->setText(tr("Exploring"));
    } else if (mode == Mode::Goals) {
      status_->setText(tr("Explorer paused. Use Nav2 Goal to pick a destination."));
    } else {
      status_->setText(tr("Waiting for exploration status"));
    }
    publishManual();
  }

  QLabel * status_;
  QPushButton * explore_button_;
  QPushButton * goals_button_;
  QPushButton * manual_button_;
  QPushButton * left_button_;
  QPushButton * right_button_;
  QSet<int> keys_;
  QString notice_;
  int rotation_ = 0;
  bool manual_granted_ = false;
  std::string session_;
  Target requested_mode_ = Target::Goals;
  std::chrono::steady_clock::time_point request_deadline_;
  std::optional<rclcpp::Client<SetManualDrive>::FutureAndRequestId> pending_;
  std::shared_ptr<DriveFeedback> drive_feedback_ = std::make_shared<DriveFeedback>();
  rclcpp::Client<SetManualDrive>::SharedPtr mode_client_;
  rclcpp::Publisher<ManualDrive>::SharedPtr manual_pub_;
  rclcpp::Subscription<DriveMode>::SharedPtr drive_report_;
  std::shared_ptr<std::atomic<Mode>> mode_ = std::make_shared<std::atomic<Mode>>(Mode::Unknown);
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr command_;
  rclcpp::Subscription<ExploreStatus>::SharedPtr report_;
};
}  // namespace amr_rviz

PLUGINLIB_EXPORT_CLASS(amr_rviz::ExplorationPanel, rviz_common::Panel)

#include "exploration_panel.moc"
