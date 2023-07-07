#pragma once

#include <QPushButton>
#include <QStackedLayout>
#include <QWidget>

#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"


const int btn_size = 192;
const int img_size = (btn_size / 4) * 3;


// ***** onroad widgets *****
class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {};
  void updateAlert(const Alert &a);

protected:
  void paintEvent(QPaintEvent*) override;

private:
  QColor bg;
  Alert alert = {};
};

class ExperimentalButton : public QPushButton {
  Q_OBJECT

public:
  explicit ExperimentalButton(QWidget *parent = 0);
  void updateState(const UIState &s);

private:
  void paintEvent(QPaintEvent *event) override;
  void changeMode();

  Params params;
  QPixmap engage_img;
  QPixmap experimental_img;
  bool experimental_mode;
  bool engageable;
};

// container window for the NVG UI
class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT
  Q_PROPERTY(float speed MEMBER speed);
  Q_PROPERTY(QString speedUnit MEMBER speedUnit);
  Q_PROPERTY(float setSpeed MEMBER setSpeed);
  Q_PROPERTY(float speedLimit MEMBER speedLimit);
  Q_PROPERTY(bool is_cruise_set MEMBER is_cruise_set);
  Q_PROPERTY(bool has_eu_speed_limit MEMBER has_eu_speed_limit);
  Q_PROPERTY(bool has_us_speed_limit MEMBER has_us_speed_limit);
  Q_PROPERTY(bool is_metric MEMBER is_metric);

  Q_PROPERTY(bool dmActive MEMBER dmActive);
  Q_PROPERTY(bool hideDM MEMBER hideDM);
  Q_PROPERTY(bool rightHandDM MEMBER rightHandDM);
  Q_PROPERTY(int status MEMBER status);

  Q_PROPERTY(bool blindSpotLeft MEMBER blindSpotLeft);
  Q_PROPERTY(bool blindSpotRight MEMBER blindSpotRight);
  Q_PROPERTY(bool drivingPersonalitiesUIWheel MEMBER drivingPersonalitiesUIWheel);
  Q_PROPERTY(bool drivingPersonalitiesViaUICar MEMBER drivingPersonalitiesViaUICar);
  Q_PROPERTY(bool experimentalMode MEMBER experimentalMode);
  Q_PROPERTY(bool timSignals MEMBER timSignals);
  Q_PROPERTY(bool muteDM MEMBER muteDM);
  Q_PROPERTY(bool turnSignalLeft MEMBER turnSignalLeft);
  Q_PROPERTY(bool turnSignalRight MEMBER turnSignalRight);
  Q_PROPERTY(QString roadName MEMBER roadName);
  Q_PROPERTY(int personalityProfile MEMBER personalityProfile);

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

private:
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawDrivingPersonalities(QPainter &p);
  void drawTimSignals(QPainter &p);
  void drawCenteredText(QPainter &p, int x, int y, const QString &text, QColor color);

  ExperimentalButton *experimental_btn;
  QPixmap dm_img;
  QPixmap map_img;
  float speed;
  const int subsign_img_size = 35;
  QString speedUnit;
  float setSpeed;
  float speedLimit;
  bool is_cruise_set = false;
  bool is_metric = false;
  bool dmActive = false;
  bool hideDM = false;
  bool rightHandDM = false;
  float dm_fade_state = 1.0;
  bool has_us_speed_limit = false;
  bool has_eu_speed_limit = false;
  bool v_ego_cluster_seen = false;
  bool blindSpotLeft;
  bool blindSpotRight;
  bool drivingPersonalitiesUIWheel;
  bool drivingPersonalitiesViaUICar;
  bool experimentalMode;
  bool timSignals;
  bool muteDM;
  bool turnSignalLeft;
  bool turnSignalRight;
  int animationFrameIndex;
  int personalityProfile;
  QVector<std::pair<QPixmap, QString>> profile_data;
  static constexpr int totalFrames = 8;
  std::vector<QPixmap> signalImgVector;

  int status = STATUS_DISENGAGED;
  std::unique_ptr<PubMaster> pm;

  int skip_frame_count = 0;
  bool wide_cam_requested = false;

  QString roadName;


protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat() override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd , int num);
  void drawHud(QPainter &p);
  void drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num  /*不使用, size_t leads_num , const cereal::RadarState::LeadData::Reader &lead0, const cereal::RadarState::LeadData::Reader &lead1 */);
  void drawDriverState(QPainter &painter, const UIState *s);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

signals:
  void mapWindowShown();

private:
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;
  bool navDisabled = false;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
