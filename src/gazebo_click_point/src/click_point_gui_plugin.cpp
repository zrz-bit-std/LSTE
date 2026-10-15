#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <cstdlib>
#include <memory>
#include <string>

#include <QCursor>
#include <QHBoxLayout>
#include <QKeySequence>
#include <QShortcut>
#include <QToolButton>
#include <QToolTip>
#include <QTimer>

#include <gazebo/common/MouseEvent.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
class ClickPointGuiPlugin : public GUIPlugin
{
public:
  ClickPointGuiPlugin()
  {
    int argc = 0;
    char **argv = nullptr;
    if (!ros::isInitialized())
      ros::init(argc, argv, "gazebo_click_point", ros::init_options::NoSigintHandler);

    this->node_ = std::make_unique<ros::NodeHandle>();
    this->node_->setCallbackQueue(&this->callbackQueue_);
    this->publisher_ = this->node_->advertise<geometry_msgs::PointStamped>("/clicked_point", 1);
    this->goalPublisher_ = this->node_->advertise<geometry_msgs::PoseStamped>("/move_base/current_goal", 1);
    this->controllerTogglePublisher_ =
        this->node_->advertise<std_msgs::Empty>("/lste/controller_toggle", 1);
    this->controllerModeSubscriber_ = this->node_->subscribe(
        "/lste/controller_mode", 1, &ClickPointGuiPlugin::OnControllerMode, this);
    this->gazeboNode_ = transport::NodePtr(new transport::Node());
    this->gazeboNode_->Init();
    this->saveControlPublisher_ =
        this->gazeboNode_->Advertise<msgs::ServerControl>("/gazebo/server/control");
    this->pauseClient_ = this->node_->serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->unpauseClient_ = this->node_->serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    this->controllerButton_ = new QToolButton(this);
    this->controllerButton_->setText("RL / KB");
    this->controllerButton_->setToolTip("Switch controller (Ctrl+Shift+K)");
    this->controllerButton_->setFixedSize(72, 28);
    QObject::connect(this->controllerButton_, &QToolButton::clicked,
                     [this]() { this->PublishControllerToggle(); });

    this->saveButton_ = new QToolButton(this);
    this->saveButton_->setText("Save");
    this->saveButton_->setToolTip("Overwrite the current world file");
    this->saveButton_->setFixedSize(52, 28);
    QObject::connect(this->saveButton_, &QToolButton::clicked,
                     [this]() { this->SaveCurrentWorld(); });

    // Pause/resume: the Space bar or this button toggles Gazebo physics so the
    // operator can freeze the scene and explain a problem while the debug
    // console inspects the paused state.  The same state is exposed on
    // /lste/pause (std_msgs/Bool) so the console can pause/resume remotely.
    this->pauseButton_ = new QToolButton(this);
    this->pauseButton_->setText("Pause");
    this->pauseButton_->setToolTip("Pause / resume simulation (Space)");
    this->pauseButton_->setCheckable(true);
    this->pauseButton_->setChecked(false);
    this->pauseButton_->setFixedSize(52, 28);
    QObject::connect(this->pauseButton_, &QToolButton::clicked,
                     [this](bool checked) { this->TogglePause(checked); });

    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);
    layout->addWidget(this->controllerButton_);
    layout->addWidget(this->saveButton_);
    layout->addWidget(this->pauseButton_);
    this->setLayout(layout);
    this->setFixedSize(185, 28);
    this->move(10, 10);

    this->controllerShortcut_ = new QShortcut(QKeySequence("Ctrl+Shift+K"), this);
    this->controllerShortcut_->setContext(Qt::ApplicationShortcut);
    QObject::connect(this->controllerShortcut_, &QShortcut::activated,
                     [this]() { this->PublishControllerToggle(); });

    this->pauseShortcut_ = new QShortcut(QKeySequence(Qt::Key_Space), this);
    this->pauseShortcut_->setContext(Qt::ApplicationShortcut);
    QObject::connect(this->pauseShortcut_, &QShortcut::activated, [this]() {
      this->TogglePause(!this->isPaused_);
    });
    this->pauseSubscriber_ = this->node_->subscribe(
        "/lste/pause", 1, &ClickPointGuiPlugin::OnPauseCommand, this);

    this->rosSpinTimer_ = new QTimer(this);
    QObject::connect(this->rosSpinTimer_, &QTimer::timeout, [this]()
    {
      this->callbackQueue_.callAvailable(ros::WallDuration(0.0));
    });
    this->rosSpinTimer_->start(50);

    gui::MouseEventHandler::Instance()->AddPressFilter(
        "gazebo_click_point", [this](const common::MouseEvent &event)
        {
          return this->OnMousePress(event);
        });
    gui::MouseEventHandler::Instance()->AddReleaseFilter(
        "gazebo_click_point", [](const common::MouseEvent &event)
        {
          if (event.Button() == common::MouseEvent::LEFT)
            QToolTip::hideText();
          return false;
        });
    gui::MouseEventHandler::Instance()->AddMoveFilter(
        "gazebo_click_point", [this](const common::MouseEvent &event)
        {
          if (event.Buttons() & common::MouseEvent::LEFT)
            this->SelectPoint(event);
          return false;
        });
  }

  ~ClickPointGuiPlugin() override
  {
    gui::MouseEventHandler::Instance()->RemovePressFilter("gazebo_click_point");
    gui::MouseEventHandler::Instance()->RemoveReleaseFilter("gazebo_click_point");
    gui::MouseEventHandler::Instance()->RemoveMoveFilter("gazebo_click_point");
  }

  void Load(sdf::ElementPtr) override
  {
    this->setVisible(true);
  }

private:
  void SaveCurrentWorld()
  {
    const char *worldPath = std::getenv("LSTE_WORLD");
    const std::string worldName = gui::get_world();
    if (!worldPath || std::string(worldPath).empty() || worldName.empty())
    {
      QToolTip::showText(QCursor::pos(), "Save unavailable: current world is not ready", this);
      QTimer::singleShot(1800, []() { QToolTip::hideText(); });
      return;
    }

    msgs::ServerControl message;
    message.set_save_world_name(worldName);
    message.set_save_filename(worldPath);
    this->saveControlPublisher_->Publish(message);
    QToolTip::showText(QCursor::pos(), "Saved layout", this);
    QTimer::singleShot(1200, []() { QToolTip::hideText(); });
  }

  void PublishControllerToggle()
  {
    this->pendingControllerMode_ =
        this->currentControllerMode_ == "sappo" ? "teleop" : "sappo";
    std_msgs::Empty message;
    this->controllerTogglePublisher_.publish(message);
    QToolTip::showText(QCursor::pos(), "Switching controller...");
  }

  void TogglePause(bool pause)
  {
    this->isPaused_ = pause;
    this->pauseButton_->setChecked(pause);
    // std_srvs::Empty is the service type; call(srv) uses srv.request/response
    // internally.  Passing two service objects to call() would try to
    // serialize the service container, which has no serialization methods.
    std_srvs::Empty srv;
    const auto client =
        pause ? &this->pauseClient_ : &this->unpauseClient_;
    if (!client->isValid() || !client->call(srv))
      return;
    const auto text = pause ? QString("Simulation paused (Space to resume)")
                            : QString("Simulation resumed");
    QToolTip::showText(QCursor::pos(), text, this);
    QTimer::singleShot(1200, []() { QToolTip::hideText(); });
  }

  void OnPauseCommand(const std_msgs::Bool::ConstPtr &message)
  {
    this->TogglePause(message->data);
  }

  void OnControllerMode(const std_msgs::String::ConstPtr &message)
  {
    this->currentControllerMode_ = message->data;
    if (this->pendingControllerMode_.empty() ||
        this->currentControllerMode_ != this->pendingControllerMode_)
      return;

    const auto text = this->currentControllerMode_ == "teleop"
                          ? QString("Switched to keyboard control")
                          : QString("Switched to SA-PPO");
    this->pendingControllerMode_.clear();
    QToolTip::showText(QCursor::pos(), text, this);
    QTimer::singleShot(1600, []() { QToolTip::hideText(); });
  }

  bool OnMousePress(const common::MouseEvent &event)
  {
    if (event.Button() == common::MouseEvent::LEFT && event.Shift() &&
        this->hasSelectedPoint_)
    {
      this->PublishGoal();
      return true;
    }

    if (event.Button() == common::MouseEvent::LEFT)
      return this->SelectPoint(event);
    return false;
  }

  void PublishGoal()
  {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = this->selectedPoint_.X();
    goal.pose.position.y = this->selectedPoint_.Y();
    goal.pose.position.z = this->selectedPoint_.Z();
    goal.pose.orientation.w = 1.0;
    this->goalPublisher_.publish(goal);

    const auto text = QString("Goal set: x=%1  y=%2")
                          .arg(this->selectedPoint_.X(), 0, 'f', 2)
                          .arg(this->selectedPoint_.Y(), 0, 'f', 2);
    QToolTip::showText(QCursor::pos(), text, this);
    QTimer::singleShot(1200, []() { QToolTip::hideText(); });
  }

  bool SelectPoint(const common::MouseEvent &event)
  {
    if (event.Dragging() && !(event.Buttons() & common::MouseEvent::LEFT))
      return false;

    auto camera = gui::get_active_camera();
    if (!camera)
      return false;

    const auto position = event.Pos();
    ignition::math::Vector3d point;
    const ignition::math::Planed ground(ignition::math::Vector3d::UnitZ, 0.0);
    if (!camera->WorldPointOnPlane(position.X(), position.Y(), ground, point))
      return false;

    geometry_msgs::PointStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";
    message.point.x = point.X();
    message.point.y = point.Y();
    message.point.z = point.Z();
    this->publisher_.publish(message);
    this->selectedPoint_ = point;
    this->hasSelectedPoint_ = true;

    const auto text = QString("Selected: x=%1  y=%2  z=%3\nShift + left-click to set goal")
                          .arg(point.X(), 0, 'f', 2)
                          .arg(point.Y(), 0, 'f', 2)
                          .arg(point.Z(), 0, 'f', 2);
    QToolTip::showText(QCursor::pos(), text, this);
    return false;
  }

  std::unique_ptr<ros::NodeHandle> node_;
  ros::CallbackQueue callbackQueue_;
  ros::Publisher publisher_;
  ros::Publisher goalPublisher_;
  ros::Publisher controllerTogglePublisher_;
  ros::Subscriber controllerModeSubscriber_;
  ros::Subscriber pauseSubscriber_;
  transport::NodePtr gazeboNode_;
  transport::PublisherPtr saveControlPublisher_;
  ros::ServiceClient pauseClient_;
  ros::ServiceClient unpauseClient_;
  QToolButton *controllerButton_ = nullptr;
  QToolButton *saveButton_ = nullptr;
  QToolButton *pauseButton_ = nullptr;
  QShortcut *controllerShortcut_ = nullptr;
  QShortcut *pauseShortcut_ = nullptr;
  QTimer *rosSpinTimer_ = nullptr;
  bool isPaused_ = false;
  std::string currentControllerMode_;
  std::string pendingControllerMode_;
  ignition::math::Vector3d selectedPoint_;
  bool hasSelectedPoint_ = false;
};

GZ_REGISTER_GUI_PLUGIN(ClickPointGuiPlugin)
}
