#include "selfdrive/frogpilot/ui/qt/offroad/model_settings.h"

FrogPilotModelPanel::FrogPilotModelPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent) {
  const std::vector<std::tuple<QString, QString, QString, QString>> modelToggles {
    {"AutomaticallyUpdateModels", tr("Automatically Update and Download Models"), tr("Automatically downloads new models and updates them if needed."), ""},

    {"ModelRandomizer", tr("Model Randomizer"), tr("Randomly selects a model each drive and brings up a prompt at the end of the drive to review the model if it's longer than 15 minutes to help find your preferred model."), ""},
    {"ManageBlacklistedModels", tr("Manage Model Blacklist"), tr("Controls which models are blacklisted and won't be used for future drives."), ""},
    {"ResetScores", tr("Reset Model Scores"), tr("Clears the ratings you've given to the driving models."), ""},
    {"ReviewScores", tr("Review Model Scores"), tr("Displays the ratings you've assigned to the driving models."), ""},

    {"DeleteModel", tr("Delete Model"), tr("Removes the selected driving model from your device."), ""},
    {"DownloadModel", tr("Download Model"), tr("Downloads the selected driving model."), ""},
    {"DownloadAllModels", tr("Download All Models"), tr("Downloads all undownloaded driving models."), ""},
    {"SelectModel", tr("Select Model"), tr("Selects which model openpilot uses to drive."), ""},
  };

  for (const auto &[param, title, desc, icon] : modelToggles) {
    AbstractControl *modelToggle;

    if (param == "ModelRandomizer") {
      FrogPilotParamManageControl *modelRandomizerToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(modelRandomizerToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        modelRandomizerOpen = true;
        showToggles(modelRandomizerKeys);
      });
      modelToggle = modelRandomizerToggle;

    } else {
      modelToggle = new ParamControl(param, title, desc, icon);
    }

    addItem(modelToggle);
    toggles[param] = modelToggle;

    if (FrogPilotParamManageControl *frogPilotManageToggle = qobject_cast<FrogPilotParamManageControl*>(modelToggle)) {
      QObject::connect(frogPilotManageToggle, &FrogPilotParamManageControl::manageButtonClicked, this, &FrogPilotModelPanel::openParentToggle);
    }

    QObject::connect(modelToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  QObject::connect(parent, &FrogPilotSettingsWindow::closeParentToggle, this, &FrogPilotModelPanel::hideToggles);
  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubParentToggle, this, &FrogPilotModelPanel::hideSubToggles);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotModelPanel::updateState);
}

void FrogPilotModelPanel::showEvent(QShowEvent *event) {
  frogpilotToggleLevels = parent->frogpilotToggleLevels;
  tuningLevel = parent->tuningLevel;

  QString currentModel = QString::fromStdString(params.get("Model")) + ".thneed";

  availableModelNames = QString::fromStdString(params.get("AvailableModelsNames")).split(",");
  availableModels = QString::fromStdString(params.get("AvailableModels")).split(",");
  experimentalModels = QString::fromStdString(params.get("ExperimentalModels")).split(",");

  modelRandomizer = params.getBool("ModelRandomizer");
  modelsDownloaded = params.getBool("ModelsDownloaded");

  QStringList modelFiles = modelDir.entryList({"*.thneed"}, QDir::Files);
  modelFiles.removeAll(currentModel);
  haveModelsDownloaded = modelFiles.size() > 1;

  hideToggles();
}

void FrogPilotModelPanel::updateState(const UIState &s) {
  if (!isVisible()) return;

  downloadAllModelsBtn->setText(modelDownloading && allModelsDownloading ? tr("CANCEL") : tr("DOWNLOAD"));
  downloadModelBtn->setText(modelDownloading && !allModelsDownloading ? tr("CANCEL") : tr("DOWNLOAD"));

  deleteModelBtn->setEnabled(!modelDeleting && !modelDownloading);
  downloadAllModelsBtn->setEnabled(s.scene.online && !cancellingDownload && !modelDeleting && (!modelDownloading || allModelsDownloading) && !modelsDownloaded);
  downloadModelBtn->setEnabled(s.scene.online && !cancellingDownload && !modelDeleting && !allModelsDownloading && !modelsDownloaded);
  selectModelBtn->setEnabled(!modelDeleting && !modelDownloading && !modelRandomizer);

  started = s.scene.started;
}

void FrogPilotModelPanel::showToggles(const std::set<QString> &keys) {
  setUpdatesEnabled(false);

  for (auto &[key, toggle] : toggles) {
    toggle->setVisible(keys.find(key) != keys.end() && tuningLevel >= frogpilotToggleLevels[key].toDouble());
  }

  setUpdatesEnabled(true);
  update();
}

void FrogPilotModelPanel::hideToggles() {
  setUpdatesEnabled(false);

  modelRandomizerOpen = false;

  for (LabelControl *label : labelControls) {
    label->setVisible(false);
  }

  for (auto &[key, toggle] : toggles) {
    bool subToggles = modelRandomizerKeys.find(key) != modelRandomizerKeys.end();

    toggle->setVisible(!subToggles && tuningLevel >= frogpilotToggleLevels[key].toDouble());
  }

  setUpdatesEnabled(true);
  update();
}

void FrogPilotModelPanel::hideSubToggles() {
  setUpdatesEnabled(false);

  if (modelRandomizerOpen) {
    for (LabelControl *label : labelControls) {
      label->setVisible(false);
    }

    for (auto &[key, toggle] : toggles) {
      toggle->setVisible(modelRandomizerKeys.find(key) != modelRandomizerKeys.end());
    }
  }

  setUpdatesEnabled(true);
  update();
}
